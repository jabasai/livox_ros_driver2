/**
 * livox_set_ip  --  Set the static IP of a Livox MID-360 (or any Livox-SDK2 device)
 *
 * Usage:
 *   livox_set_ip <current_lidar_ip> <new_ip> <netmask> <gateway> [host_ip]
 *
 * Examples:
 *   # Connect from host 192.168.1.10, change lidar from factory IP to static IP:
 *   livox_set_ip 192.168.1.100 192.168.1.167 255.255.255.0 192.168.1.1 192.168.1.10
 *
 *   # If host IP is not specified, SDK will auto-detect it:
 *   livox_set_ip 192.168.1.167 192.168.1.184 255.255.255.0 192.168.1.1
 *
 * Build:
 *   See CMakeLists.txt
 *
 * IMPORTANT: Power-cycle the lidar after running this tool for the new IP to take effect.
 */

#include <livox_lidar_api.h>
#include <livox_lidar_def.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>

// --------------------------------------------------------------------------
// Global state shared between callbacks and main thread
// --------------------------------------------------------------------------

struct AppState {
  std::string target_lidar_ip;   // The lidar we want to configure (current IP)
  std::string new_ip;
  std::string new_netmask;
  std::string new_gateway;

  std::atomic<bool> lidar_found{false};
  std::atomic<bool> ip_set_done{false};
  std::atomic<bool> ip_set_ok{false};

  std::mutex              mtx;
  std::condition_variable cv;
};

static AppState g_state;

// --------------------------------------------------------------------------
// Helper: convert uint32_t handle (packed IPv4) back to dotted string
// --------------------------------------------------------------------------
static std::string HandleToIp(uint32_t handle) {
  // SDK encodes the lidar IP as a big-endian uint32 handle
  char buf[20];
  snprintf(buf, sizeof(buf), "%u.%u.%u.%u",
           (handle >> 24) & 0xFF,
           (handle >> 16) & 0xFF,
           (handle >>  8) & 0xFF,
           (handle      ) & 0xFF);
  return std::string(buf);
}

// --------------------------------------------------------------------------
// Callback: called by the SDK whenever a lidar is discovered or its info
//           changes (connects, state change, etc.)
// --------------------------------------------------------------------------
static void OnLidarInfoChange(const uint32_t handle,
                              const LivoxLidarInfo* info,
                              void* /*client_data*/) {
  if (!info) return;

  std::string discovered_ip(info->lidar_ip);

  std::cout << "[info] Discovered lidar  ip=" << discovered_ip
            << "  sn=" << info->sn
            << "  dev_type=" << static_cast<int>(info->dev_type)
            << "\n";

  // Is this the lidar we are looking for?
  if (discovered_ip != g_state.target_lidar_ip) {
    return;
  }

  if (g_state.lidar_found.exchange(true)) {
    return;  // already being handled
  }

  std::cout << "[info] Target lidar found, sending IP-change command ...\n";

  // Build the new IP config
  LivoxLidarIpInfo ip_cfg{};
  strncpy(ip_cfg.ip_addr,  g_state.new_ip.c_str(),      sizeof(ip_cfg.ip_addr)  - 1);
  strncpy(ip_cfg.net_mask, g_state.new_netmask.c_str(),  sizeof(ip_cfg.net_mask) - 1);
  strncpy(ip_cfg.gw_addr,  g_state.new_gateway.c_str(),  sizeof(ip_cfg.gw_addr)  - 1);

  // Async callback for the IP-set command response
  auto ip_set_cb = [](livox_status status,
                      uint32_t /*handle*/,
                      LivoxLidarAsyncControlResponse* resp,
                      void* /*client_data*/) {
    if (status == kLivoxLidarStatusSuccess && resp && resp->ret_code == 0) {
      std::cout << "[ok]  IP change accepted by lidar. "
                   "Power-cycle the device to apply the new IP.\n";
      g_state.ip_set_ok = true;
    } else {
      std::cout << "[err] IP change failed."
                << "  sdk_status=" << status;
      if (resp) {
        std::cout << "  ret_code=" << static_cast<int>(resp->ret_code)
                  << "  error_key=" << resp->error_key;
      }
      std::cout << "\n";
      g_state.ip_set_ok = false;
    }
    g_state.ip_set_done = true;
    g_state.cv.notify_all();
  };

  livox_status rc = SetLivoxLidarIp(handle, &ip_cfg, ip_set_cb, nullptr);
  if (rc != kLivoxLidarStatusSuccess) {
    std::cerr << "[err] SetLivoxLidarIp() returned error code " << rc << "\n";
    g_state.ip_set_ok   = false;
    g_state.ip_set_done = true;
    g_state.cv.notify_all();
  }
}

// --------------------------------------------------------------------------
// Write a minimal Livox SDK config to a temp file so that the SDK has a
// valid device-info map structure before handling discovery responses.
// Passing nullptr for the config causes GetFirmwareType() to dereference
// an uninitialised map entry → SIGSEGV when any lidar replies.
// --------------------------------------------------------------------------
static std::string WriteTempConfig(const std::string& host_ip) {
  std::string path = "/tmp/livox_set_ip_config_" + std::to_string(getpid()) + ".json";
  const std::string ip = host_ip.empty() ? "0.0.0.0" : host_ip;

  std::ofstream f(path);
  if (!f.is_open()) return "";

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
static void PrintUsage(const char* prog) {
  std::cerr
    << "Usage: " << prog
    << " <current_lidar_ip> <new_ip> <netmask> <gateway> [host_ip]\n\n"
    << "  current_lidar_ip  Current IP of the lidar to configure\n"
    << "  new_ip            New static IP to assign to the lidar\n"
    << "  netmask           Subnet mask  (e.g. 255.255.255.0)\n"
    << "  gateway           Gateway address (e.g. 192.168.1.1)\n"
    << "  host_ip           [optional] IP of the NIC connected to the lidar\n"
    << "                    (omit to let the SDK auto-detect)\n\n"
    << "Example:\n"
    << "  " << prog
    << " 192.168.1.111 192.168.1.167 255.255.255.0 192.168.1.1 192.168.1.10\n\n"
    << "IMPORTANT: Power-cycle the lidar after this tool completes.\n";
}

// --------------------------------------------------------------------------
// main
// --------------------------------------------------------------------------
int main(int argc, char** argv) {
  if (argc < 5 || argc > 6) {
    PrintUsage(argv[0]);
    return 1;
  }

  g_state.target_lidar_ip = argv[1];
  g_state.new_ip          = argv[2];
  g_state.new_netmask     = argv[3];
  g_state.new_gateway     = argv[4];
  std::string host_ip     = (argc == 6) ? argv[5] : "";

  std::cout << "=== Livox IP configuration tool ===\n"
            << "  Target lidar (current IP) : " << g_state.target_lidar_ip << "\n"
            << "  New IP                    : " << g_state.new_ip          << "\n"
            << "  Netmask                   : " << g_state.new_netmask      << "\n"
            << "  Gateway                   : " << g_state.new_gateway      << "\n";
  if (!host_ip.empty()) {
    std::cout << "  Host NIC IP               : " << host_ip << "\n";
  }
  std::cout << "\n";

  // Silence the SDK's verbose console output
  DisableLivoxSdkConsoleLogger();

  // Register the device-discovery callback BEFORE initialising the SDK
  SetLivoxLidarInfoChangeCallback(OnLidarInfoChange, nullptr);

  // Write a minimal config so the SDK's internal DeviceInfo map is properly
  // initialised before discovery responses arrive.  Passing nullptr here
  // causes GetFirmwareType() to dereference an uninitialised entry → SIGSEGV.
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

  std::cout << "[info] SDK initialised, scanning for lidar at "
            << g_state.target_lidar_ip << " ...\n";

  // Wait up to 15 seconds for the IP-set command to complete
  constexpr int kTimeoutSeconds = 15;
  std::unique_lock<std::mutex> lock(g_state.mtx);
  bool completed = g_state.cv.wait_for(
      lock,
      std::chrono::seconds(kTimeoutSeconds),
      [] { return g_state.ip_set_done.load(); });

  LivoxLidarSdkUninit();

  // Remove the temporary config file written to avoid the SDK SIGSEGV
  if (!tmp_cfg.empty()) std::remove(tmp_cfg.c_str());

  if (!completed) {
    std::cerr << "[err] Timed out after " << kTimeoutSeconds << "s. "
                 "Lidar not found or not responding.\n"
              << "      Make sure the lidar is powered, reachable at "
              << g_state.target_lidar_ip
              << ",\n      and that your host NIC is in the same subnet.\n";
    return 1;
  }

  if (!g_state.ip_set_ok) {
    std::cerr << "[err] IP configuration FAILED.\n";
    return 1;
  }

  std::cout << "\n[ok] Done. New IP: " << g_state.new_ip << "\n"
            << "     Please POWER-CYCLE the lidar now.\n";
  return 0;
}