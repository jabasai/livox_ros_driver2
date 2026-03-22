/**
 * livox_scan  --  Discover all Livox LiDAR devices visible on the network
 *                and report their settings.
 *
 * Usage:
 *   livox_scan [host_ip [timeout_seconds]]
 *
 * Examples:
 *   # Scan using auto-detected host NIC for 10 seconds (default):
 *   livox_scan
 *
 *   # Scan from a specific NIC:
 *   livox_scan 192.168.1.10
 *
 *   # Scan from a specific NIC with a 5-second timeout:
 *   livox_scan 192.168.1.10 5
 *
 * Build:
 *   See CMakeLists.txt
 */

#include <livox_lidar_api.h>
#include <livox_lidar_def.h>

#include <algorithm>
#include <atomic>
#include <arpa/inet.h>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

// --------------------------------------------------------------------------
// Per-device information collected during the scan
// --------------------------------------------------------------------------

struct NetworkConfig {
  std::string lidar_ip;
  std::string lidar_netmask;
  std::string lidar_gateway;
  
  std::string point_host_ip;
  uint16_t    point_host_port;
  uint16_t    point_lidar_port;
  
  std::string imu_host_ip;
  uint16_t    imu_host_port;
  uint16_t    imu_lidar_port;
  
  std::string state_host_ip;
  uint16_t    state_host_port;
  uint16_t    state_lidar_port;
  
  std::string ctl_host_ip;
  uint16_t    ctl_host_port;
  uint16_t    ctl_lidar_port;
  
  std::string log_host_ip;
  uint16_t    log_host_port;
  uint16_t    log_lidar_port;
  
  NetworkConfig() : point_host_port(0), point_lidar_port(0),
                    imu_host_port(0), imu_lidar_port(0),
                    state_host_port(0), state_lidar_port(0),
                    ctl_host_port(0), ctl_lidar_port(0),
                    log_host_port(0), log_lidar_port(0) {}
};

struct ConfigInfo {
  std::string   firmware_version;
  std::string   work_mode;
  int32_t       pcl_data_type;
  int32_t       scan_pattern;
  uint32_t      blind_spot;
  bool          dual_emit;
  bool          imu_enabled;
  bool          config_queried;
  NetworkConfig network;
  
  ConfigInfo() : pcl_data_type(-1), scan_pattern(-1), blind_spot(0),
                 dual_emit(false), imu_enabled(false), config_queried(false) {}
};

struct DeviceEntry {
  uint32_t    handle;
  std::string lidar_ip;
  std::string sn;
  uint8_t     dev_type;
  ConfigInfo  config;
};

// --------------------------------------------------------------------------
// Global state
// --------------------------------------------------------------------------

struct ScanState {
  std::mutex               mtx;
  std::vector<DeviceEntry> devices;
  std::atomic<int>         pending_queries{0};
};

static ScanState g_scan;

// --------------------------------------------------------------------------
// Helper: map numeric device type to a human-readable string
// --------------------------------------------------------------------------
static const char* DevTypeName(uint8_t dev_type) {
  switch (dev_type) {
    case kLivoxLidarTypeHub:           return "Hub";
    case kLivoxLidarTypeMid40:         return "Mid-40";
    case kLivoxLidarTypeTele:          return "Tele-15";
    case kLivoxLidarTypeHorizon:       return "Horizon";
    case kLivoxLidarTypeMid70:         return "Mid-70";
    case kLivoxLidarTypeAvia:          return "Avia";
    case kLivoxLidarTypeMid360:        return "Mid-360";
    case kLivoxLidarTypeIndustrialHAP: return "Industrial HAP";
    case kLivoxLidarTypeHAP:           return "HAP";
    case kLivoxLidarTypePA:            return "PA";
    default:                           return "Unknown";
  }
}

// --------------------------------------------------------------------------
// Helper: convert point data type enum to string
// --------------------------------------------------------------------------
static const char* DataTypeName(int32_t type) {
  switch (type) {
    case 0x01: return "Cartesian High";
    case 0x02: return "Cartesian Low";
    case 0x03: return "Spherical";
    default:   return "Unknown";
  }
}

// --------------------------------------------------------------------------
// Helper: convert scan pattern enum to string
// --------------------------------------------------------------------------
static const char* ScanPatternName(int32_t pattern) {
  switch (pattern) {
    case 0x00: return "Non-Repetitive";
    case 0x01: return "Repetitive";
    case 0x02: return "Repetitive Low FPS";
    default:   return "Unknown";
  }
}

// --------------------------------------------------------------------------
// Helper: find device entry by handle
// --------------------------------------------------------------------------
static DeviceEntry* FindDeviceByHandle(uint32_t handle) {
  std::lock_guard<std::mutex> lock(g_scan.mtx);
  for (auto& d : g_scan.devices) {
    if (d.handle == handle) return &d;
  }
  return nullptr;
}

// --------------------------------------------------------------------------
// Callback: firmware version query response
// --------------------------------------------------------------------------
static void OnFirmwareVersionQuery(livox_status status, 
                                   uint32_t handle,
                                   LivoxLidarDiagInternalInfoResponse* response,
                                   void* /*client_data*/) {
  DeviceEntry* dev = FindDeviceByHandle(handle);
  
  if (dev && status == kLivoxLidarStatusSuccess && response && response->ret_code == 0) {
    uint16_t off = 0;
    for (uint8_t i = 0; i < response->param_num; ++i) {
      LivoxLidarKeyValueParam* kv = reinterpret_cast<LivoxLidarKeyValueParam*>(&response->data[off]);
      if (kv->key == kKeyVersionApp && kv->length >= 4) {
        // version_app is uint8_t[4]: [major, minor, patch, build]
        char buf[32];
        snprintf(buf, sizeof(buf), "%u.%u.%u.%u",
                 (uint8_t)kv->value[0], (uint8_t)kv->value[1],
                 (uint8_t)kv->value[2], (uint8_t)kv->value[3]);
        dev->config.firmware_version = buf;
        break;
      }
      off += sizeof(uint16_t) * 2 + kv->length;
    }
    std::cout << "[info] Firmware query response for " << dev->lidar_ip 
              << ": " << dev->config.firmware_version << "\n";
  }
  
  g_scan.pending_queries--;
}

// --------------------------------------------------------------------------
// Callback: internal info query response  
// --------------------------------------------------------------------------
static void OnInternalInfoQuery(livox_status status,
                               uint32_t handle, 
                               LivoxLidarDiagInternalInfoResponse* response,
                               void* /*client_data*/) {
  DeviceEntry* dev = FindDeviceByHandle(handle);
  
  if (!dev) {
    std::cout << "[error] Internal info callback for unknown device handle=" << handle << "\n";
    g_scan.pending_queries--;
    return;
  }
  
  if (status != kLivoxLidarStatusSuccess || response == nullptr || response->ret_code != 0) {
    std::cout << "[warn] Internal config query failed for " << dev->lidar_ip 
              << " (status=" << status;
    if (response) std::cout << ", ret_code=" << (int)response->ret_code;
    std::cout << ")\n";
    g_scan.pending_queries--;
    return;
  }

  dev->config.config_queried = true;

  // Iterate key-value pairs using the LivoxLidarKeyValueParam struct, exactly
  // as shown in the Livox SDK2 sample (livox_lidar_quick_start/main.cpp).
  // Each entry: key(2B) + length(2B) + value(length B).
  uint16_t off = 0;
  for (uint8_t i = 0; i < response->param_num; ++i) {
    LivoxLidarKeyValueParam* kv = reinterpret_cast<LivoxLidarKeyValueParam*>(&response->data[off]);

    // Helper: format 4-byte little-endian IP stored in kv->value[idx..idx+3]
    auto parse_ip = [&](int idx) -> std::string {
      char buf[20];
      snprintf(buf, sizeof(buf), "%u.%u.%u.%u",
               (uint8_t)kv->value[idx],   (uint8_t)kv->value[idx+1],
               (uint8_t)kv->value[idx+2], (uint8_t)kv->value[idx+3]);
      return buf;
    };

    switch (kv->key) {
      case kKeyPclDataType:
        if (kv->length >= 1) dev->config.pcl_data_type = kv->value[0];
        break;

      case kKeyPatternMode:
        if (kv->length >= 1) dev->config.scan_pattern = kv->value[0];
        break;

      case kKeyDualEmitEn:
        if (kv->length >= 1) dev->config.dual_emit = (kv->value[0] != 0);
        break;

      case kKeyImuDataEn:
        if (kv->length >= 1) dev->config.imu_enabled = (kv->value[0] != 0);
        break;

      case kKeyBlindSpotSet:
        if (kv->length >= 4) memcpy(&dev->config.blind_spot, &kv->value[0], 4);
        break;

      case kKeyWorkMode:
        if (kv->length >= 1) {
          switch (kv->value[0]) {
            case 0x01: dev->config.work_mode = "Normal";  break;
            case 0x02: dev->config.work_mode = "WakeUp";  break;
            case 0x03: dev->config.work_mode = "Sleep";   break;
            default:   dev->config.work_mode = "Unknown"; break;
          }
        }
        break;

      // LiDAR IP config: ip[4] + netmask[4] + gateway[4]  (12 bytes on wire)
      case kKeyLidarIpCfg:
        if (kv->length >= 12) {
          dev->config.network.lidar_ip      = parse_ip(0);
          dev->config.network.lidar_netmask = parse_ip(4);
          dev->config.network.lidar_gateway = parse_ip(8);
        }
        break;

      // Host IP configs: ip[4] + host_port[2] + lidar_port[2]  (8 bytes on wire)
      case kKeyStateInfoHostIpCfg:
        if (kv->length >= 8) {
          dev->config.network.state_host_ip = parse_ip(0);
          memcpy(&dev->config.network.state_host_port,  &kv->value[4], 2);
          memcpy(&dev->config.network.state_lidar_port, &kv->value[6], 2);
        }
        break;

      case kKeyLidarPointDataHostIpCfg:
        if (kv->length >= 8) {
          dev->config.network.point_host_ip = parse_ip(0);
          memcpy(&dev->config.network.point_host_port,  &kv->value[4], 2);
          memcpy(&dev->config.network.point_lidar_port, &kv->value[6], 2);
        }
        break;

      case kKeyLidarImuHostIpCfg:
        if (kv->length >= 8) {
          dev->config.network.imu_host_ip = parse_ip(0);
          memcpy(&dev->config.network.imu_host_port,  &kv->value[4], 2);
          memcpy(&dev->config.network.imu_lidar_port, &kv->value[6], 2);
        }
        break;

      case kKeyCtlHostIpCfg:
        if (kv->length >= 8) {
          dev->config.network.ctl_host_ip = parse_ip(0);
          memcpy(&dev->config.network.ctl_host_port,  &kv->value[4], 2);
          memcpy(&dev->config.network.ctl_lidar_port, &kv->value[6], 2);
        }
        break;

      case kKeyLogHostIpCfg:
        if (kv->length >= 8) {
          dev->config.network.log_host_ip = parse_ip(0);
          memcpy(&dev->config.network.log_host_port,  &kv->value[4], 2);
          memcpy(&dev->config.network.log_lidar_port, &kv->value[6], 2);
        }
        break;

      default:
        break;
    }

    off += sizeof(uint16_t) * 2;  // key(2) + length(2)
    off += kv->length;
  }
  
  g_scan.pending_queries--;
  std::cout << "[info] Pending queries: " << g_scan.pending_queries.load() << "\n";
}

// --------------------------------------------------------------------------
// Callback: called by the SDK whenever a lidar is discovered or its info
//           changes (multiple calls for the same device are de-duplicated)
// --------------------------------------------------------------------------
static void OnLidarInfoChange(const uint32_t handle,
                              const LivoxLidarInfo* info,
                              void* /*client_data*/) {
  if (!info) return;

  bool should_query = false;
  {
    std::lock_guard<std::mutex> lock(g_scan.mtx);

    // De-duplicate by handle
    for (const auto& d : g_scan.devices) {
      if (d.handle == handle) return;
    }

    DeviceEntry entry;
    entry.handle   = handle;
    entry.lidar_ip = info->lidar_ip;
    entry.sn       = info->sn;
    entry.dev_type = info->dev_type;

    std::cout << "[found] " << entry.lidar_ip
              << "  type=" << DevTypeName(entry.dev_type)
              << "  sn=" << entry.sn
              << "\n";

    g_scan.devices.push_back(std::move(entry));
    should_query = true;
  }
  
  // Query device configuration outside the lock
  if (should_query) {
    std::cout << "[info] Querying configuration for " << info->lidar_ip << " ...\n";
    
    // Increment counter before making queries
    g_scan.pending_queries += 2;
    
    // Query firmware version
    QueryLivoxLidarFirmwareVer(handle, OnFirmwareVersionQuery, nullptr);
    
    // Query internal info (includes various config parameters)
    QueryLivoxLidarInternalInfo(handle, OnInternalInfoQuery, nullptr);
  }
}

// --------------------------------------------------------------------------
// Write a minimal Livox SDK config to a temp file so that the SDK has a
// valid device-info map structure before handling discovery responses.
// Passing nullptr for the config causes GetFirmwareType() to dereference
// an uninitialised map entry → SIGSEGV when any lidar replies.
// --------------------------------------------------------------------------
static std::string WriteTempConfig(const std::string& host_ip) {
  // Use /tmp with a unique-ish name
  std::string path = "/tmp/livox_scan_config_" + std::to_string(getpid()) + ".json";

  // Choose a non-empty host IP for the config; fall back to a harmless value
  // if the caller didn't provide one (the SDK will still auto-pick the NIC).
  const std::string ip = host_ip.empty() ? "0.0.0.0" : host_ip;

  std::ofstream f(path);
  if (!f.is_open()) return "";

  // lidar_type 8 = MID360 (kLivoxLidarTypeMid360).
  // lidar_configs is intentionally empty — discovery finds whatever is present.
  f << "{\n"
    << "  \"lidar_summary_info\": { \"lidar_type\": 8 },\n"
    << "  \"MID360\": {\n"
    << "    \"lidar_net_info\": {\n"
    << "      \"cmd_data_port\": 56100,\n"
    << "      \"push_msg_port\": 56200,\n"
    << "      \"point_data_port\": 56300,\n"
    << "      \"imu_data_port\": 56400,\n"
    << "      \"log_data_port\": 56500\n"
    << "    },\n"
    << "    \"host_net_info\": {\n"
    << "      \"cmd_data_ip\": \"" << ip << "\",\n"
    << "      \"cmd_data_port\": 56101,\n"
    << "      \"push_msg_ip\": \"" << ip << "\",\n"
    << "      \"push_msg_port\": 56201,\n"
    << "      \"point_data_ip\": \"" << ip << "\",\n"
    << "      \"point_data_port\": 56301,\n"
    << "      \"imu_data_ip\": \"" << ip << "\",\n"
    << "      \"imu_data_port\": 56401,\n"
    << "      \"log_data_ip\": \"\",\n"
    << "      \"log_data_port\": 56501\n"
    << "    }\n"
    << "  },\n"
    << "  \"lidar_configs\": []\n"
    << "}\n";

  return f.good() ? path : "";
}

// --------------------------------------------------------------------------
// Usage
// --------------------------------------------------------------------------
static void PrintUsage(const char* prog, std::ostream& out = std::cout) {
  out
    << "Usage: " << prog << " [OPTIONS] [host_ip [timeout_seconds]]\n\n"
    << "Options:\n"
    << "  -h, --help       Show this help message and exit\n\n"
    << "Arguments:\n"
    << "  host_ip          [optional] IP of the host NIC facing the LiDARs\n"
    << "                   (omit to let the SDK auto-detect)\n"
    << "  timeout_seconds  [optional] How long to scan in seconds (default: 10)\n\n"
    << "Examples:\n"
    << "  " << prog << "\n"
    << "  " << prog << " --help\n"
    << "  " << prog << " 192.168.1.10\n"
    << "  " << prog << " 192.168.1.10 5\n";
}

// --------------------------------------------------------------------------
// main
// --------------------------------------------------------------------------
int main(int argc, char** argv) {
  // Handle --help / -h before any other processing
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      PrintUsage(argv[0]);
      return 0;
    }
  }

  if (argc > 3) {
    PrintUsage(argv[0], std::cerr);
    return 1;
  }

  std::string host_ip      = (argc >= 2) ? argv[1] : "";
  int         timeout_secs = 10;

  if (argc == 3) {
    try {
      timeout_secs = std::stoi(argv[2]);
      if (timeout_secs <= 0) throw std::invalid_argument("non-positive");
    } catch (...) {
      std::cerr << "[err] Invalid timeout value: " << argv[2] << "\n";
      PrintUsage(argv[0], std::cerr);
      return 1;
    }
  }

  std::cout << "=== Livox LiDAR scanner ===\n";
  if (!host_ip.empty()) {
    std::cout << "  Host NIC : " << host_ip << "\n";
  } else {
    std::cout << "  Host NIC : (auto-detect)\n";
  }
  std::cout << "  Timeout  : " << timeout_secs << " s\n\n";

  // Silence the SDK's own console output so our messages are uncluttered
  DisableLivoxSdkConsoleLogger();

  // Register the discovery callback BEFORE initialising the SDK
  SetLivoxLidarInfoChangeCallback(OnLidarInfoChange, nullptr);

  // Write a minimal config so that the SDK's internal DeviceInfo map is
  // properly initialised before discovery responses arrive.  Passing nullptr
  // here causes GetFirmwareType() to dereference an uninitialised entry
  // → SIGSEGV (reproduced on liblivox_lidar_sdk_shared.so).
  std::string tmp_cfg = WriteTempConfig(host_ip);
  if (tmp_cfg.empty()) {
    std::cerr << "[warn] Could not write temporary config; "
                 "SDK may crash on discovery responses.\n";
  }

  const char* cfg_path = tmp_cfg.empty() ? nullptr : tmp_cfg.c_str();

  if (!LivoxLidarSdkInit(cfg_path, host_ip.c_str())) {
    std::cerr << "[err] LivoxLidarSdkInit() failed. "
                 "Check that the host IP / NIC is correct.\n";
    if (!tmp_cfg.empty()) std::remove(tmp_cfg.c_str());
    return 1;
  }

  std::cout << "[info] Scanning for " << timeout_secs << " second(s) ...\n\n";

  std::this_thread::sleep_for(std::chrono::seconds(timeout_secs));

  // Wait for pending queries to complete (with timeout)
  int wait_count = 0;
  const int max_wait = 50; // 5 seconds max
  while (g_scan.pending_queries.load() > 0 && wait_count < max_wait) {
    std::cout << "[info] Waiting for " << g_scan.pending_queries.load() 
              << " pending queries...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    wait_count++;
  }
  
  if (g_scan.pending_queries.load() > 0) {
    std::cout << "[warn] Timed out waiting for queries, " 
              << g_scan.pending_queries.load() << " still pending\n";
  } else {
    std::cout << "[info] All queries completed\n";
  }

  LivoxLidarSdkUninit();

  // Remove the temporary config file written to avoid the SDK SIGSEGV
  if (!tmp_cfg.empty()) std::remove(tmp_cfg.c_str());

  // -----------------------------------------------------------------------
  // Print summary table
  // -----------------------------------------------------------------------
  const auto& devs = g_scan.devices;

  std::cout << "\n";

  if (devs.empty()) {
    std::cout << "[result] No LiDAR devices found.\n"
              << "         Make sure the device is powered, the host NIC is in "
                 "the same subnet,\n"
              << "         and that no firewall is blocking UDP traffic.\n";
    return 1;
  }

  std::cout << "[result] Found " << devs.size()
            << " device(s):\n\n";

  // Print basic info table
  const int w_ip   = 16;
  const int w_type = 16;
  const int w_sn   = 18;

  auto hline = [&]() {
    std::cout << "+" << std::string(w_ip   + 2, '-')
              << "+" << std::string(w_type + 2, '-')
              << "+" << std::string(w_sn   + 2, '-')
              << "+\n";
  };

  hline();
  std::cout << "| " << std::left << std::setw(w_ip)   << "IP Address"
            << " | "             << std::setw(w_type) << "Device Type"
            << " | "             << std::setw(w_sn)   << "Serial Number"
            << " |\n";
  hline();

  for (const auto& d : devs) {
    std::cout << "| " << std::left << std::setw(w_ip)   << d.lidar_ip
              << " | "             << std::setw(w_type) << DevTypeName(d.dev_type)
              << " | "             << std::setw(w_sn)   << d.sn
              << " |\n";
  }

  hline();
  std::cout << "\n";

  // Print detailed configuration for each device
  std::cout << "=== Device Configuration Details ===\n\n";
  
  for (const auto& d : devs) {
    std::cout << "Device: " << d.lidar_ip << " (" << d.sn << ")\n";
    std::cout << "  Type:              " << DevTypeName(d.dev_type) << "\n";
    
    if (!d.config.firmware_version.empty()) {
      std::cout << "  Firmware Version:  " << d.config.firmware_version << "\n";
    }
    
    if (d.config.pcl_data_type >= 0) {
      std::cout << "  Point Data Type:   " << DataTypeName(d.config.pcl_data_type) 
                << " (" << d.config.pcl_data_type << ")\n";
    }
    
    if (d.config.scan_pattern >= 0) {
      std::cout << "  Scan Pattern:      " << ScanPatternName(d.config.scan_pattern)
                << " (" << d.config.scan_pattern << ")\n";
    }
    
    if (d.config.blind_spot > 0) {
      std::cout << "  Blind Spot:        " << d.config.blind_spot << " cm\n";
    }
    
    std::cout << "  Dual Emit:         " << (d.config.dual_emit ? "Enabled" : "Disabled") << "\n";
    std::cout << "  IMU Data:          " << (d.config.imu_enabled ? "Enabled" : "Disabled") << "\n";
    
    if (!d.config.work_mode.empty()) {
      std::cout << "  Work Mode:         " << d.config.work_mode << "\n";
    }
    
    // Network Configuration
    std::cout << "\n  Network Configuration:\n";
    
    if (!d.config.network.lidar_ip.empty()) {
      std::cout << "    LiDAR IP:        " << d.config.network.lidar_ip << "\n";
      if (!d.config.network.lidar_netmask.empty()) {
        std::cout << "    Netmask:         " << d.config.network.lidar_netmask << "\n";
      }
      if (!d.config.network.lidar_gateway.empty()) {
        std::cout << "    Gateway:         " << d.config.network.lidar_gateway << "\n";
      }
    }
    
    if (!d.config.network.point_host_ip.empty()) {
      std::cout << "\n    Point Cloud:\n";
      std::cout << "      Host IP:       " << d.config.network.point_host_ip << "\n";
      std::cout << "      Host Port:     " << d.config.network.point_host_port << "\n";
      std::cout << "      LiDAR Port:    " << d.config.network.point_lidar_port << "\n";
    }
    
    if (!d.config.network.imu_host_ip.empty()) {
      std::cout << "\n    IMU Data:\n";
      std::cout << "      Host IP:       " << d.config.network.imu_host_ip << "\n";
      std::cout << "      Host Port:     " << d.config.network.imu_host_port << "\n";
      std::cout << "      LiDAR Port:    " << d.config.network.imu_lidar_port << "\n";
    }
    
    if (!d.config.network.state_host_ip.empty()) {
      std::cout << "\n    State Info:\n";
      std::cout << "      Host IP:       " << d.config.network.state_host_ip << "\n";
      std::cout << "      Host Port:     " << d.config.network.state_host_port << "\n";
      std::cout << "      LiDAR Port:    " << d.config.network.state_lidar_port << "\n";
    }
    
    if (!d.config.network.ctl_host_ip.empty()) {
      std::cout << "\n    Control:\n";
      std::cout << "      Host IP:       " << d.config.network.ctl_host_ip << "\n";
      std::cout << "      Host Port:     " << d.config.network.ctl_host_port << "\n";
      std::cout << "      LiDAR Port:    " << d.config.network.ctl_lidar_port << "\n";
    }
    
    if (!d.config.network.log_host_ip.empty()) {
      std::cout << "\n    Logging:\n";
      std::cout << "      Host IP:       " << d.config.network.log_host_ip << "\n";
      std::cout << "      Host Port:     " << d.config.network.log_host_port << "\n";
      std::cout << "      LiDAR Port:    " << d.config.network.log_lidar_port << "\n";
    }
    
    std::cout << "\n";
  }

  return 0;
}
