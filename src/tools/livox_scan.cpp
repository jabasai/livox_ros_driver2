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

struct DeviceEntry {
  uint32_t    handle;
  std::string lidar_ip;
  std::string sn;
  uint8_t     dev_type;
};

// --------------------------------------------------------------------------
// Global state
// --------------------------------------------------------------------------

struct ScanState {
  std::mutex              mtx;
  std::vector<DeviceEntry> devices;
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
// Callback: called by the SDK whenever a lidar is discovered or its info
//           changes (multiple calls for the same device are de-duplicated)
// --------------------------------------------------------------------------
static void OnLidarInfoChange(const uint32_t handle,
                              const LivoxLidarInfo* info,
                              void* /*client_data*/) {
  if (!info) return;

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

  // Column widths
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

  return 0;
}
