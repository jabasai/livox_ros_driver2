/**
 * livox_reboot  --  Soft-reboot or reset one or more Livox LiDAR devices
 *
 * Usage:
 *   livox_reboot <ip1> [ip2 ...]  [--reset] [--host-ip <host_ip>]
 *   livox_reboot --all            [--reset] [--host-ip <host_ip>]
 *
 * Options:
 *   --reset         Send a Reset command instead of a Reboot.
 *                   Reset clears device configuration/state; Reboot is a
 *                   full firmware restart equivalent to a power-cycle.
 *   --host-ip <ip>  IP of the host NIC connected to the lidar(s).
 *                   Omit to let the SDK auto-detect.
 *   --all           Target every device discovered on the network.
 *                   Use with caution!
 *
 * Examples:
 *   # Reboot a single device
 *   livox_reboot 192.168.1.167
 *
 *   # Reboot two devices simultaneously
 *   livox_reboot 192.168.1.167 192.168.1.184
 *
 *   # Reset all discovered devices
 *   livox_reboot --all --reset
 *
 *   # Reboot with an explicit host NIC address
 *   livox_reboot 192.168.1.167 --host-ip 192.168.1.10
 *
 * Build:
 *   See CMakeLists.txt
 *
 * NOTE: This issues a soft reboot (firmware-triggered restart) over the
 *       network — it does NOT cut and restore physical power.  After a
 *       reboot the device typically comes back online within ~30 seconds.
 *       If you need a true power-cycle, use external hardware (smart PDU /
 *       relay board).
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
#include <set>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

// ---------------------------------------------------------------------------
// Per-device job state
// ---------------------------------------------------------------------------

struct DeviceJob {
  std::string ip;
  bool        triggered{false};   // reboot/reset command has been sent
  bool        completed{false};   // callback received
  bool        success{false};     // command reported success
};

// ---------------------------------------------------------------------------
// Global application state
// ---------------------------------------------------------------------------

struct AppState {
  std::set<std::string> target_ips;  // empty when --all is used
  bool                  reboot_all{false};
  bool                  use_reset{false};

  std::vector<DeviceJob> jobs;
  std::mutex             mtx;
  std::condition_variable cv;
};

static AppState g_state;

// ---------------------------------------------------------------------------
// Helper: convert uint32_t handle (packed IPv4) to dotted-decimal string
// ---------------------------------------------------------------------------
static std::string HandleToIp(uint32_t handle) {
  char buf[20];
  snprintf(buf, sizeof(buf), "%u.%u.%u.%u",
           (handle >> 24) & 0xFF,
           (handle >> 16) & 0xFF,
           (handle >>  8) & 0xFF,
           (handle      ) & 0xFF);
  return std::string(buf);
}

// ---------------------------------------------------------------------------
// Callbacks for Reboot and Reset
// ---------------------------------------------------------------------------

static void OnRebootResponse(livox_status status, uint32_t handle,
                             LivoxLidarRebootResponse* response,
                             void* /*client_data*/) {
  std::string ip = HandleToIp(handle);
  std::lock_guard<std::mutex> lock(g_state.mtx);

  for (auto& job : g_state.jobs) {
    if (job.ip == ip && !job.completed) {
      job.completed = true;
      if (status == kLivoxLidarStatusSuccess && response &&
          response->ret_code == 0) {
        job.success = true;
        std::cout << "[ok] " << ip << "  reboot command accepted.\n";
      } else {
        job.success = false;
        int ret = response ? static_cast<int>(response->ret_code) : -1;
        std::cerr << "[err] " << ip << "  reboot failed"
                  << "  status=" << status
                  << "  ret_code=" << ret << "\n";
      }
      break;
    }
  }
  g_state.cv.notify_all();
}

static void OnResetResponse(livox_status status, uint32_t handle,
                            LivoxLidarResetResponse* response,
                            void* /*client_data*/) {
  std::string ip = HandleToIp(handle);
  std::lock_guard<std::mutex> lock(g_state.mtx);

  for (auto& job : g_state.jobs) {
    if (job.ip == ip && !job.completed) {
      job.completed = true;
      if (status == kLivoxLidarStatusSuccess && response &&
          response->ret_code == 0) {
        job.success = true;
        std::cout << "[ok] " << ip << "  reset command accepted.\n";
      } else {
        job.success = false;
        int ret = response ? static_cast<int>(response->ret_code) : -1;
        std::cerr << "[err] " << ip << "  reset failed"
                  << "  status=" << status
                  << "  ret_code=" << ret << "\n";
      }
      break;
    }
  }
  g_state.cv.notify_all();
}

// ---------------------------------------------------------------------------
// Discovery callback: fires when a lidar is found (or its state changes).
// If the discovered device is in our target list (or --all is set) and we
// haven't yet sent it a command, do so now.
// ---------------------------------------------------------------------------
static void OnLidarInfoChange(const uint32_t handle,
                              const LivoxLidarInfo* info,
                              void* /*client_data*/) {
  if (!info) return;

  std::string discovered_ip(info->lidar_ip);
  std::cout << "[info] Discovered lidar  ip=" << discovered_ip << "\n";

  std::lock_guard<std::mutex> lock(g_state.mtx);

  // Decide whether this device is a target
  bool is_target = g_state.reboot_all ||
      (g_state.target_ips.count(discovered_ip) > 0);
  if (!is_target) return;

  // Find (or create) the job for this IP
  DeviceJob* job = nullptr;
  for (auto& j : g_state.jobs) {
    if (j.ip == discovered_ip) { job = &j; break; }
  }
  if (!job) {
    // --all path: add a new job on first discovery
    g_state.jobs.push_back(DeviceJob{discovered_ip});
    job = &g_state.jobs.back();
  }

  if (job->triggered) return;  // already sent — don't double-send
  job->triggered = true;

  // Send the command (outside the lock is safer, but the SDK callbacks are
  // async so holding the mutex for the send is fine here)
  livox_status rc;
  if (g_state.use_reset) {
    std::cout << "[info] Sending Reset  to " << discovered_ip << " ...\n";
    rc = LivoxLidarRequestReset(handle, OnResetResponse, nullptr);
  } else {
    std::cout << "[info] Sending Reboot to " << discovered_ip << " ...\n";
    rc = LivoxLidarRequestReboot(handle, OnRebootResponse, nullptr);
  }

  if (rc != kLivoxLidarStatusSuccess) {
    std::cerr << "[err] Command send failed for " << discovered_ip
              << "  rc=" << rc << "\n";
    job->completed = true;
    job->success   = false;
    g_state.cv.notify_all();
  }
}

// ---------------------------------------------------------------------------
// Write a minimal SDK config to a temp file to prevent a SIGSEGV that occurs
// when the SDK tries to look up an uninitialised device-info map entry.
// ---------------------------------------------------------------------------
static std::string WriteTempConfig(const std::string& host_ip) {
  std::string path = "/tmp/livox_reboot_config_" +
                     std::to_string(getpid()) + ".json";
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

// ---------------------------------------------------------------------------
// Usage
// ---------------------------------------------------------------------------
static void PrintUsage(const char* prog) {
  std::cerr
    << "Usage:\n"
    << "  " << prog << " <ip1> [ip2 ...]  [--reset] [--host-ip <host_ip>]\n"
    << "  " << prog << " --all            [--reset] [--host-ip <host_ip>]\n\n"
    << "Options:\n"
    << "  --reset         Send Reset instead of Reboot\n"
    << "  --host-ip <ip>  Host NIC IP (auto-detected if omitted)\n"
    << "  --all           Target all discovered devices\n\n"
    << "Examples:\n"
    << "  " << prog << " 192.168.1.167\n"
    << "  " << prog << " 192.168.1.167 192.168.1.184\n"
    << "  " << prog << " --all --reset\n"
    << "  " << prog << " 192.168.1.167 --host-ip 192.168.1.10\n";
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
  if (argc < 2) {
    PrintUsage(argv[0]);
    return 1;
  }

  std::string host_ip;
  bool show_help = false;

  // Parse arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg == "--help" || arg == "-h") {
      show_help = true;
    } else if (arg == "--all") {
      g_state.reboot_all = true;
    } else if (arg == "--reset") {
      g_state.use_reset = true;
    } else if (arg == "--host-ip") {
      if (i + 1 >= argc) {
        std::cerr << "[err] --host-ip requires an argument.\n";
        return 1;
      }
      host_ip = argv[++i];
    } else if (arg.rfind("--", 0) == 0) {
      std::cerr << "[err] Unknown option: " << arg << "\n";
      PrintUsage(argv[0]);
      return 1;
    } else {
      // Treat as a lidar IP address
      g_state.target_ips.insert(arg);
      g_state.jobs.push_back(DeviceJob{arg});
    }
  }

  if (show_help) {
    PrintUsage(argv[0]);
    return 0;
  }

  if (!g_state.reboot_all && g_state.target_ips.empty()) {
    std::cerr << "[err] No target IP addresses specified.\n";
    PrintUsage(argv[0]);
    return 1;
  }

  const char* cmd = g_state.use_reset ? "Reset" : "Reboot";

  std::cout << "=== Livox LiDAR " << cmd << " tool ===\n";
  if (g_state.reboot_all) {
    std::cout << "  Targets  : all discovered devices\n";
  } else {
    for (const auto& ip : g_state.target_ips) {
      std::cout << "  Target   : " << ip << "\n";
    }
  }
  if (!host_ip.empty()) {
    std::cout << "  Host NIC : " << host_ip << "\n";
  }
  std::cout << "  Command  : " << cmd << "\n\n";

  // Silence the SDK's own console output
  DisableLivoxSdkConsoleLogger();

  // Register the discovery callback BEFORE initialising the SDK
  SetLivoxLidarInfoChangeCallback(OnLidarInfoChange, nullptr);

  // Write a minimal config so that the SDK's internal DeviceInfo map is
  // properly initialised before discovery responses arrive (prevents SIGSEGV).
  std::string tmp_cfg = WriteTempConfig(host_ip);
  if (tmp_cfg.empty()) {
    std::cerr << "[warn] Could not write temporary config; "
                 "SDK may crash on discovery responses.\n";
  }

  const char* cfg_path = tmp_cfg.empty() ? nullptr : tmp_cfg.c_str();
  const char* sdk_host = host_ip.empty() ? nullptr : host_ip.c_str();

  if (!LivoxLidarSdkInit(cfg_path, sdk_host)) {
    std::cerr << "[err] LivoxLidarSdkInit() failed. "
                 "Check that the host IP / NIC is correct.\n";
    if (!tmp_cfg.empty()) std::remove(tmp_cfg.c_str());
    return 1;
  }

  // Wait for all jobs to complete (or timeout).
  // For specific IPs: exit as soon as every job has a response.
  // For --all: allow a discovery grace period so the SDK can find all
  // devices before we start checking for completion, then exit early
  // once every discovered device has received its response.
  constexpr int kTimeoutSeconds = 15;

  // --all grace period: sleep without holding any lock so callbacks can
  // update g_state.jobs freely during device discovery.
  if (g_state.reboot_all) {
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }

  auto all_done = [&]() -> bool {
    if (g_state.jobs.empty()) return false;  // nothing found yet
    for (const auto& job : g_state.jobs) {
      if (!job.completed) return false;
    }
    return true;
  };

  // Remaining timeout after the optional discovery grace period.
  int remaining_s = g_state.reboot_all ? (kTimeoutSeconds - 5) : kTimeoutSeconds;

  std::unique_lock<std::mutex> lock(g_state.mtx);
  g_state.cv.wait_for(lock, std::chrono::seconds(remaining_s), all_done);

  LivoxLidarSdkUninit();

  if (!tmp_cfg.empty()) std::remove(tmp_cfg.c_str());

  // Print summary
  std::cout << "\n=== Summary ===\n";
  int ok_count   = 0;
  int fail_count = 0;

  for (const auto& job : g_state.jobs) {
    if (!job.triggered) {
      std::cerr << "[err] " << job.ip
                << "  — device not found (not reachable or wrong IP?)\n";
      ++fail_count;
    } else if (!job.completed) {
      std::cerr << "[err] " << job.ip << "  — timed out waiting for response.\n";
      ++fail_count;
    } else if (!job.success) {
      ++fail_count;  // already printed by callback
    } else {
      ++ok_count;
    }
  }

  if (g_state.reboot_all && g_state.jobs.empty()) {
    std::cerr << "[err] No devices discovered within " << kTimeoutSeconds
              << " s.\n";
    return 1;
  }

  std::cout << "  " << ok_count   << " device(s) successfully commanded.\n";
  if (fail_count > 0) {
    std::cout << "  " << fail_count << " device(s) failed.\n";
  }
  if (!g_state.use_reset) {
    std::cout << "\nNote: after a reboot, allow ~30 s for the device(s) to "
                 "come back online.\n";
  }

  return fail_count > 0 ? 1 : 0;
}
