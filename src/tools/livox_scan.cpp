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
#include <cstring>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
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

  if (!LivoxLidarSdkInit(nullptr, host_ip.c_str())) {
    std::cerr << "[err] LivoxLidarSdkInit() failed. "
                 "Check that the host IP / NIC is correct.\n";
    return 1;
  }

  std::cout << "[info] Scanning for " << timeout_secs << " second(s) ...\n\n";

  std::this_thread::sleep_for(std::chrono::seconds(timeout_secs));

  LivoxLidarSdkUninit();

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
