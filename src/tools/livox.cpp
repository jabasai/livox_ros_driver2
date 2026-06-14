/**
 * livox  --  Unified Livox LiDAR command-line tool
 *
 * Usage:
 *   livox <verb> [options]
 *
 * Verbs:
 *   search           Discover all Livox devices visible on the network.
 *   setup            Assign a static IP address to a lidar.
 *   reboot           Soft-reboot one or more lidars (firmware restart).
 *   reset            Reset one or more lidars (config/state reset).
 *   firmware_upgrade Upgrade the firmware on one or more lidars.
 *
 * Run  livox <verb> --help  for per-verb usage.
 *
 * Examples:
 *   livox search
 *   livox search 192.168.1.10 5
 *   livox setup  192.168.1.111 192.168.1.167 255.255.255.0 192.168.1.1
 *   livox reboot 192.168.1.167
 *   livox reboot 192.168.1.167 192.168.1.184
 *   livox reset  --all
 *   livox firmware_upgrade /path/to/firmware.bin 192.168.1.167
 *   livox firmware_upgrade /path/to/firmware.bin --all
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
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

// ---------------------------------------------------------------------------
// Common helpers
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

static uint32_t IpToHandle(const std::string& ip) {
  // inet_addr returns IP in network byte order; the SDK handle uses the same
  // encoding that HandleToIp() above decodes, which is big-endian uint32.
  // On little-endian hosts inet_addr and HandleToIp are consistent.
  struct in_addr addr{};
  if (inet_pton(AF_INET, ip.c_str(), &addr) != 1) return 0;
  uint32_t n = ntohl(addr.s_addr);
  return n;
}

static std::string WriteTempConfig(const std::string& host_ip) {
  std::string path = "/tmp/livox_tool_config_" + std::to_string(getpid()) + ".json";
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

static bool InitSdk(const std::string& host_ip,
                    LivoxLidarInfoChangeCallback info_cb, void* cb_data,
                    std::string& out_tmp_cfg) {
  DisableLivoxSdkConsoleLogger();
  SetLivoxLidarInfoChangeCallback(info_cb, cb_data);

  out_tmp_cfg = WriteTempConfig(host_ip);
  if (out_tmp_cfg.empty()) {
    std::cerr << "[warn] Could not write temporary config; SDK may crash on discovery.\n";
  }

  const char* cfg  = out_tmp_cfg.empty()  ? nullptr : out_tmp_cfg.c_str();
  const char* host = host_ip.empty() ? nullptr : host_ip.c_str();

  if (!LivoxLidarSdkInit(cfg, host)) {
    std::cerr << "[err] LivoxLidarSdkInit() failed. "
                 "Check that the host NIC IP is correct.\n";
    if (!out_tmp_cfg.empty()) std::remove(out_tmp_cfg.c_str());
    out_tmp_cfg.clear();
    return false;
  }
  return true;
}

static void CleanupSdk(const std::string& tmp_cfg) {
  LivoxLidarSdkUninit();
  if (!tmp_cfg.empty()) std::remove(tmp_cfg.c_str());
}

// ---------------------------------------------------------------------------
// verb: search
// ---------------------------------------------------------------------------

static const char* kSearchUsage = R"(
Usage:
  livox search [host_ip [timeout_seconds]]

Options:
  host_ip           NIC address to use for scanning (auto-detected if omitted)
  timeout_seconds   How long to scan (default: 10)

Examples:
  livox search
  livox search 192.168.1.10
  livox search 192.168.1.10 5
)";

struct DeviceEntry {
  uint32_t    handle;
  std::string ip;
  std::string sn;
  uint8_t     dev_type;
};

struct SearchState {
  std::mutex               mtx;
  std::vector<DeviceEntry> devices;
};

static SearchState g_search;

static void OnSearchLidarInfo(const uint32_t handle,
                              const LivoxLidarInfo* info,
                              void* /*client_data*/) {
  if (!info) return;
  std::lock_guard<std::mutex> lock(g_search.mtx);
  for (const auto& d : g_search.devices)
    if (d.handle == handle) return;  // already known

  DeviceEntry e;
  e.handle   = handle;
  e.ip       = info->lidar_ip;
  e.sn       = info->sn;
  e.dev_type = info->dev_type;
  g_search.devices.push_back(e);
  std::cout << "[found] ip=" << e.ip
            << "  sn="       << e.sn
            << "  dev_type=" << static_cast<int>(e.dev_type) << "\n";
}

static int verb_search(const std::vector<std::string>& args) {
  if (!args.empty() && (args[0] == "--help" || args[0] == "-h")) {
    std::cout << kSearchUsage;
    return 0;
  }

  std::string host_ip;
  int timeout_s = 10;

  if (args.size() >= 1) host_ip   = args[0];
  if (args.size() >= 2) timeout_s = std::stoi(args[1]);

  if (host_ip.empty()) {
    std::cout << "[info] No host IP specified; SDK will auto-detect the NIC.\n";
  } else {
    std::cout << "[info] Host NIC: " << host_ip << "\n";
  }
  std::cout << "[info] Scanning for " << timeout_s << " second(s)...\n\n";

  g_search.devices.clear();
  std::string tmp_cfg;
  if (!InitSdk(host_ip, OnSearchLidarInfo, nullptr, tmp_cfg)) return 1;

  std::this_thread::sleep_for(std::chrono::seconds(timeout_s));

  CleanupSdk(tmp_cfg);

  std::lock_guard<std::mutex> lock(g_search.mtx);
  if (g_search.devices.empty()) {
    std::cout << "\n[result] No devices found.\n";
    return 1;
  }

  std::cout << "\n[result] " << g_search.devices.size() << " device(s) found:\n";
  std::cout << std::left
            << std::setw(18) << "IP"
            << std::setw(20) << "Serial Number"
            << "Dev Type\n"
            << std::string(50, '-') << "\n";
  for (const auto& d : g_search.devices) {
    std::cout << std::left
              << std::setw(18) << d.ip
              << std::setw(20) << d.sn
              << static_cast<int>(d.dev_type) << "\n";
  }
  return 0;
}

// ---------------------------------------------------------------------------
// verb: setup (set static IP)
// ---------------------------------------------------------------------------

static const char* kSetupUsage = R"(
Usage:
  livox setup <current_ip> <new_ip> <netmask> <gateway> [host_ip]

Arguments:
  current_ip   Current IP address of the lidar
  new_ip       New static IP to assign
  netmask      Subnet mask (e.g. 255.255.255.0)
  gateway      Gateway address (e.g. 192.168.1.1)
  host_ip      [optional] IP of the host NIC (auto-detected if omitted)

Examples:
  livox setup 192.168.1.100 192.168.1.167 255.255.255.0 192.168.1.1
  livox setup 192.168.1.100 192.168.1.167 255.255.255.0 192.168.1.1 192.168.1.10

IMPORTANT: Power-cycle the lidar after this command for the new IP to take effect.
)";

struct SetupState {
  std::string target_ip;
  std::string new_ip;
  std::string new_netmask;
  std::string new_gateway;

  std::atomic<bool> done{false};
  std::atomic<bool> ok{false};

  std::mutex              mtx;
  std::condition_variable cv;
};

static SetupState g_setup;

static void OnSetupIpResult(livox_status status, uint32_t /*handle*/,
                             LivoxLidarAsyncControlResponse* resp,
                             void* /*client_data*/) {
  if (status == kLivoxLidarStatusSuccess && resp && resp->ret_code == 0) {
    std::cout << "[ok] IP updated to " << g_setup.new_ip << "\n";
    g_setup.ok = true;
  } else {
    int ret = resp ? static_cast<int>(resp->ret_code) : -1;
    std::cerr << "[err] IP set failed  status=" << status
              << "  ret_code=" << ret << "\n";
  }
  g_setup.done = true;
  g_setup.cv.notify_all();
}

static void OnSetupLidarInfo(const uint32_t handle,
                              const LivoxLidarInfo* info,
                              void* /*client_data*/) {
  if (!info) return;
  if (std::string(info->lidar_ip) != g_setup.target_ip) return;

  std::cout << "[info] Found target lidar at " << g_setup.target_ip << "\n";

  LivoxLidarIpInfo cfg{};
  strncpy(cfg.ip_addr,   g_setup.new_ip.c_str(),      sizeof(cfg.ip_addr) - 1);
  strncpy(cfg.net_mask,  g_setup.new_netmask.c_str(), sizeof(cfg.net_mask) - 1);
  strncpy(cfg.gw_addr,   g_setup.new_gateway.c_str(), sizeof(cfg.gw_addr) - 1);

  livox_status rc = SetLivoxLidarIp(handle, &cfg, OnSetupIpResult, nullptr);
  if (rc != kLivoxLidarStatusSuccess) {
    std::cerr << "[err] SetLivoxLidarIp() failed  rc=" << rc << "\n";
    g_setup.done = true;
    g_setup.cv.notify_all();
  }
}

static int verb_setup(const std::vector<std::string>& args) {
  if (args.empty() || args[0] == "--help" || args[0] == "-h") {
    std::cout << kSetupUsage;
    return 0;
  }
  if (args.size() < 4) {
    std::cerr << "[err] setup requires at least 4 arguments.\n" << kSetupUsage;
    return 1;
  }

  g_setup.target_ip   = args[0];
  g_setup.new_ip      = args[1];
  g_setup.new_netmask = args[2];
  g_setup.new_gateway = args[3];
  std::string host_ip = (args.size() >= 5) ? args[4] : "";

  std::cout << "=== Livox IP setup ===\n"
            << "  Current IP : " << g_setup.target_ip   << "\n"
            << "  New IP     : " << g_setup.new_ip       << "\n"
            << "  Netmask    : " << g_setup.new_netmask  << "\n"
            << "  Gateway    : " << g_setup.new_gateway  << "\n";
  if (!host_ip.empty()) std::cout << "  Host NIC   : " << host_ip << "\n";
  std::cout << "\n";

  std::string tmp_cfg;
  if (!InitSdk(host_ip, OnSetupLidarInfo, nullptr, tmp_cfg)) return 1;

  constexpr int kTimeoutS = 15;
  std::unique_lock<std::mutex> lock(g_setup.mtx);
  bool completed = g_setup.cv.wait_for(lock, std::chrono::seconds(kTimeoutS),
                                       [] { return g_setup.done.load(); });

  CleanupSdk(tmp_cfg);

  if (!completed) {
    std::cerr << "[err] Timed out after " << kTimeoutS << " s. "
                 "Lidar not found or not responding.\n";
    return 1;
  }
  if (!g_setup.ok) {
    std::cerr << "[err] IP configuration FAILED.\n";
    return 1;
  }
  std::cout << "[ok] Done.  POWER-CYCLE the lidar for the new IP to take effect.\n";
  return 0;
}

// ---------------------------------------------------------------------------
// verb: reboot / reset
// ---------------------------------------------------------------------------

static const char* kRebootUsage = R"(
Usage:
  livox reboot <ip1> [ip2 ...]  [--host-ip <host_ip>]
  livox reboot --all            [--host-ip <host_ip>]

  livox reset  <ip1> [ip2 ...]  [--host-ip <host_ip>]
  livox reset  --all            [--host-ip <host_ip>]

Options:
  --host-ip <ip>  Host NIC IP (auto-detected if omitted)
  --all           Target all discovered devices

NOTE: reboot = firmware restart (~30 s to come back online)
      reset  = config/state reset (not a full reboot)
)";

struct DeviceJob {
  std::string ip;
  bool triggered{false};
  bool completed{false};
  bool success{false};
};

struct RebootState {
  std::set<std::string> target_ips;
  bool                  target_all{false};
  bool                  use_reset{false};

  std::vector<DeviceJob> jobs;
  std::mutex             mtx;
  std::condition_variable cv;
};

static RebootState g_reboot;

static void OnRebootResponse(livox_status status, uint32_t handle,
                              LivoxLidarRebootResponse* resp,
                              void* /*client_data*/) {
  std::string ip = HandleToIp(handle);
  std::lock_guard<std::mutex> lock(g_reboot.mtx);
  for (auto& j : g_reboot.jobs) {
    if (j.ip == ip && !j.completed) {
      j.completed = true;
      j.success = (status == kLivoxLidarStatusSuccess && resp && resp->ret_code == 0);
      if (j.success)
        std::cout << "[ok] " << ip << "  reboot command accepted.\n";
      else
        std::cerr << "[err] " << ip << "  reboot failed"
                  << "  status=" << status
                  << "  ret_code=" << (resp ? (int)resp->ret_code : -1) << "\n";
      break;
    }
  }
  g_reboot.cv.notify_all();
}

static void OnResetResponse(livox_status status, uint32_t handle,
                             LivoxLidarResetResponse* resp,
                             void* /*client_data*/) {
  std::string ip = HandleToIp(handle);
  std::lock_guard<std::mutex> lock(g_reboot.mtx);
  for (auto& j : g_reboot.jobs) {
    if (j.ip == ip && !j.completed) {
      j.completed = true;
      j.success = (status == kLivoxLidarStatusSuccess && resp && resp->ret_code == 0);
      if (j.success)
        std::cout << "[ok] " << ip << "  reset command accepted.\n";
      else
        std::cerr << "[err] " << ip << "  reset failed"
                  << "  status=" << status
                  << "  ret_code=" << (resp ? (int)resp->ret_code : -1) << "\n";
      break;
    }
  }
  g_reboot.cv.notify_all();
}

static void OnRebootLidarInfo(const uint32_t handle,
                               const LivoxLidarInfo* info,
                               void* /*client_data*/) {
  if (!info) return;
  std::string discovered_ip(info->lidar_ip);

  std::lock_guard<std::mutex> lock(g_reboot.mtx);
  bool is_target = g_reboot.target_all ||
      g_reboot.target_ips.count(discovered_ip) > 0;
  if (!is_target) return;

  DeviceJob* job = nullptr;
  for (auto& j : g_reboot.jobs)
    if (j.ip == discovered_ip) { job = &j; break; }
  if (!job) {
    g_reboot.jobs.push_back(DeviceJob{discovered_ip});
    job = &g_reboot.jobs.back();
  }
  if (job->triggered) return;
  job->triggered = true;

  livox_status rc;
  if (g_reboot.use_reset) {
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
    g_reboot.cv.notify_all();
  }
}

static int verb_reboot_or_reset(const std::vector<std::string>& args,
                                 bool use_reset) {
  if (!args.empty() && (args[0] == "--help" || args[0] == "-h")) {
    std::cout << kRebootUsage;
    return 0;
  }

  g_reboot.target_ips.clear();
  g_reboot.jobs.clear();
  g_reboot.target_all = false;
  g_reboot.use_reset  = use_reset;

  std::string host_ip;
  for (std::size_t i = 0; i < args.size(); ++i) {
    if (args[i] == "--all") {
      g_reboot.target_all = true;
    } else if (args[i] == "--host-ip" && i + 1 < args.size()) {
      host_ip = args[++i];
    } else if (args[i].rfind("--", 0) == 0) {
      std::cerr << "[err] Unknown option: " << args[i] << "\n";
      return 1;
    } else {
      g_reboot.target_ips.insert(args[i]);
      g_reboot.jobs.push_back(DeviceJob{args[i]});
    }
  }

  if (!g_reboot.target_all && g_reboot.target_ips.empty()) {
    std::cerr << "[err] No target IP addresses specified.\n" << kRebootUsage;
    return 1;
  }

  const char* cmd = use_reset ? "Reset" : "Reboot";
  std::cout << "=== Livox LiDAR " << cmd << " ===\n";
  if (g_reboot.target_all)
    std::cout << "  Targets: all discovered devices\n";
  else
    for (const auto& ip : g_reboot.target_ips)
      std::cout << "  Target: " << ip << "\n";
  if (!host_ip.empty()) std::cout << "  Host NIC: " << host_ip << "\n";
  std::cout << "\n";

  std::string tmp_cfg;
  if (!InitSdk(host_ip, OnRebootLidarInfo, nullptr, tmp_cfg)) return 1;

  // For --all, sleep a discovery grace period before checking completion.
  if (g_reboot.target_all) {
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }

  constexpr int kTimeoutS = 15;
  auto all_done = [&]() -> bool {
    if (g_reboot.jobs.empty()) return false;
    for (const auto& j : g_reboot.jobs)
      if (!j.completed) return false;
    return true;
  };

  int remaining_s = g_reboot.target_all ? (kTimeoutS - 5) : kTimeoutS;
  std::unique_lock<std::mutex> lock(g_reboot.mtx);
  g_reboot.cv.wait_for(lock, std::chrono::seconds(remaining_s), all_done);

  CleanupSdk(tmp_cfg);

  std::cout << "\n=== Summary ===\n";
  int ok_count = 0, fail_count = 0;
  for (const auto& j : g_reboot.jobs) {
    if (!j.triggered) {
      std::cerr << "[err] " << j.ip << "  — device not found.\n";
      ++fail_count;
    } else if (!j.completed) {
      std::cerr << "[err] " << j.ip << "  — timed out.\n";
      ++fail_count;
    } else if (!j.success) {
      ++fail_count;
    } else {
      ++ok_count;
    }
  }
  if (g_reboot.target_all && g_reboot.jobs.empty()) {
    std::cerr << "[err] No devices discovered.\n";
    return 1;
  }
  std::cout << "  " << ok_count << " device(s) successfully commanded.\n";
  if (fail_count > 0)
    std::cout << "  " << fail_count << " device(s) failed.\n";
  if (!use_reset)
    std::cout << "\nNote: allow ~30 s for rebooted device(s) to come back online.\n";

  return fail_count > 0 ? 1 : 0;
}

// ---------------------------------------------------------------------------
// verb: firmware_upgrade
// ---------------------------------------------------------------------------

static const char* kUpgradeUsage = R"(
Usage:
  livox firmware_upgrade <firmware.bin> [ip1 ip2 ...] [--all] [--host-ip <ip>]

Arguments:
  firmware.bin    Path to the firmware binary file (.bin)
  ip1 ip2 ...     Specific lidar IPs to upgrade (omit to use --all)

Options:
  --all           Upgrade all discovered devices
  --host-ip <ip>  Host NIC IP (auto-detected if omitted)

Examples:
  livox firmware_upgrade /path/to/MID360_FW_v13.18.0244.bin 192.168.1.167
  livox firmware_upgrade /path/to/firmware.bin --all
  livox firmware_upgrade /path/to/firmware.bin 192.168.1.167 192.168.1.184

NOTE: The lidar reboots automatically after a successful firmware upgrade.
      Do not power off the device during the upgrade process.
)";

struct UpgradeJob {
  std::string ip;
  uint32_t    handle{0};
  bool        triggered{false};
  bool        completed{false};
  bool        success{false};
  int         progress{0};
};

struct UpgradeState {
  std::string firmware_path;
  std::set<std::string>  target_ips;
  bool                   target_all{false};

  std::vector<UpgradeJob> jobs;
  std::mutex              mtx;
  std::condition_variable cv;
};

static UpgradeState g_upgrade;

static void OnUpgradeWorkMode(livox_status status, uint32_t handle,
                               LivoxLidarAsyncControlResponse* resp,
                               void* /*client_data*/) {
  if (status != kLivoxLidarStatusSuccess || !resp || resp->ret_code != 0) {
    std::cerr << "[warn] SetWorkMode(Upgrade) returned error for handle "
              << handle << "\n";
  }
}

static void OnUpgradeProgress(uint32_t handle, LivoxLidarUpgradeState state,
                               void* /*client_data*/) {
  std::string ip = HandleToIp(handle);
  std::lock_guard<std::mutex> lock(g_upgrade.mtx);

  for (auto& j : g_upgrade.jobs) {
    if (j.handle != handle) continue;
    j.progress = state.progress;

    if (state.state == kLivoxLidarUpgradeComplete || state.progress == 100) {
      if (!j.completed) {
        j.completed = true;
        j.success   = true;
        std::cout << "[ok] " << ip << "  firmware upgrade complete.\n";
        g_upgrade.cv.notify_all();
      }
    } else if (state.state == kLivoxLidarUpgradeErr ||
               state.state == kLivoxLidarUpgradeTimeout) {
      if (!j.completed) {
        j.completed = true;
        j.success   = false;
        std::cerr << "[err] " << ip << "  firmware upgrade FAILED (state="
                  << state.state << ").\n";
        g_upgrade.cv.notify_all();
      }
    } else {
      // Print progress bar
      std::cout << "\r[info] " << ip << "  upgrading...  "
                << std::setw(3) << state.progress << "%" << std::flush;
    }
    break;
  }
}

static void OnUpgradeLidarInfo(const uint32_t handle,
                                const LivoxLidarInfo* info,
                                void* /*client_data*/) {
  if (!info) return;
  std::string discovered_ip(info->lidar_ip);

  bool is_target = g_upgrade.target_all ||
      g_upgrade.target_ips.count(discovered_ip) > 0;
  if (!is_target) return;

  std::lock_guard<std::mutex> lock(g_upgrade.mtx);

  UpgradeJob* job = nullptr;
  for (auto& j : g_upgrade.jobs)
    if (j.ip == discovered_ip) { job = &j; break; }
  if (!job) {
    g_upgrade.jobs.push_back(UpgradeJob{discovered_ip});
    job = &g_upgrade.jobs.back();
  }
  if (job->triggered) return;
  job->handle    = handle;
  job->triggered = true;

  std::cout << "[info] Found " << discovered_ip
            << "  — setting upgrade mode ...\n";

  // Put the lidar into upgrade mode before calling UpgradeLivoxLidars
  SetLivoxLidarWorkMode(handle, kLivoxLidarUpgrade,
                        OnUpgradeWorkMode, nullptr);
}

static int verb_firmware_upgrade(const std::vector<std::string>& args) {
  if (args.empty() || args[0] == "--help" || args[0] == "-h") {
    std::cout << kUpgradeUsage;
    return 0;
  }

  g_upgrade.firmware_path.clear();
  g_upgrade.target_ips.clear();
  g_upgrade.jobs.clear();
  g_upgrade.target_all = false;

  std::string host_ip;

  // First positional arg is the firmware file
  std::size_t idx = 0;
  if (args[idx].rfind("--", 0) != 0) {
    g_upgrade.firmware_path = args[idx++];
  }

  for (; idx < args.size(); ++idx) {
    if (args[idx] == "--all") {
      g_upgrade.target_all = true;
    } else if (args[idx] == "--host-ip" && idx + 1 < args.size()) {
      host_ip = args[++idx];
    } else if (args[idx].rfind("--", 0) == 0) {
      std::cerr << "[err] Unknown option: " << args[idx] << "\n";
      return 1;
    } else {
      g_upgrade.target_ips.insert(args[idx]);
      UpgradeJob j;
      j.ip = args[idx];
      g_upgrade.jobs.push_back(j);
    }
  }

  if (g_upgrade.firmware_path.empty()) {
    std::cerr << "[err] No firmware file specified.\n" << kUpgradeUsage;
    return 1;
  }
  if (!g_upgrade.target_all && g_upgrade.target_ips.empty()) {
    std::cerr << "[err] No target IP addresses and --all not specified.\n"
              << kUpgradeUsage;
    return 1;
  }

  // Verify the firmware file exists
  {
    std::ifstream test(g_upgrade.firmware_path);
    if (!test.is_open()) {
      std::cerr << "[err] Firmware file not found: "
                << g_upgrade.firmware_path << "\n";
      return 1;
    }
  }

  std::cout << "=== Livox Firmware Upgrade ===\n"
            << "  Firmware : " << g_upgrade.firmware_path << "\n";
  if (g_upgrade.target_all)
    std::cout << "  Targets  : all discovered devices\n";
  else
    for (const auto& ip : g_upgrade.target_ips)
      std::cout << "  Target   : " << ip << "\n";
  if (!host_ip.empty()) std::cout << "  Host NIC : " << host_ip << "\n";
  std::cout << "\nWARNING: Do NOT power off the device(s) during upgrade!\n\n";

  std::string tmp_cfg;
  if (!InitSdk(host_ip, OnUpgradeLidarInfo, nullptr, tmp_cfg)) return 1;

  if (!SetLivoxLidarUpgradeFirmwarePath(g_upgrade.firmware_path.c_str())) {
    std::cerr << "[err] SetLivoxLidarUpgradeFirmwarePath() failed. "
                 "Check that the firmware file path is valid.\n";
    CleanupSdk(tmp_cfg);
    return 1;
  }
  SetLivoxLidarUpgradeProgressCallback(OnUpgradeProgress, nullptr);

  // Discovery grace period: wait for devices to connect and enter upgrade mode
  constexpr int kDiscoveryS = 8;
  std::cout << "[info] Waiting " << kDiscoveryS
            << " s for device(s) to enter upgrade mode...\n";
  std::this_thread::sleep_for(std::chrono::seconds(kDiscoveryS));

  // Start the upgrade for all discovered target handles
  {
    std::lock_guard<std::mutex> lock(g_upgrade.mtx);
    std::vector<uint32_t> handles;
    for (const auto& j : g_upgrade.jobs) {
      if (j.triggered && j.handle != 0) handles.push_back(j.handle);
    }
    if (handles.empty()) {
      std::cerr << "[err] No target device(s) found after " << kDiscoveryS
                << " s. Aborting.\n";
      CleanupSdk(tmp_cfg);
      return 1;
    }
    std::cout << "[info] Starting upgrade for "
              << handles.size() << " device(s)...\n";
    UpgradeLivoxLidars(handles.data(), static_cast<uint8_t>(handles.size()));
  }

  // Wait for all upgrades to finish (max 10 minutes)
  constexpr int kUpgradeTimeoutS = 600;
  auto all_done = [&]() -> bool {
    for (const auto& j : g_upgrade.jobs)
      if (j.triggered && !j.completed) return false;
    return !g_upgrade.jobs.empty();
  };

  std::unique_lock<std::mutex> lock(g_upgrade.mtx);
  bool completed = g_upgrade.cv.wait_for(
      lock, std::chrono::seconds(kUpgradeTimeoutS), all_done);
  lock.unlock();

  std::cout << "\n";  // newline after progress bar
  CleanupSdk(tmp_cfg);

  std::cout << "\n=== Upgrade Summary ===\n";
  int ok_count = 0, fail_count = 0;
  for (const auto& j : g_upgrade.jobs) {
    if (!j.triggered) {
      std::cerr << "[err] " << j.ip << "  — device not found.\n";
      ++fail_count;
    } else if (!j.completed) {
      std::cerr << "[err] " << j.ip << "  — timed out.\n";
      ++fail_count;
    } else if (!j.success) {
      ++fail_count;
    } else {
      ++ok_count;
    }
  }
  if (!completed) {
    std::cerr << "[err] Upgrade timed out after " << kUpgradeTimeoutS << " s.\n";
  }
  std::cout << "  " << ok_count << " device(s) upgraded successfully.\n";
  if (fail_count > 0)
    std::cout << "  " << fail_count << " device(s) failed.\n";

  return (fail_count > 0 || !completed) ? 1 : 0;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

static const char* kTopLevelUsage = R"(
livox — Unified Livox LiDAR command-line tool

Usage:
  livox <verb> [options]

Verbs:
  search           Discover all Livox devices on the network
  setup            Assign a static IP address to a lidar
  reboot           Soft-reboot one or more lidars (firmware restart)
  reset            Reset one or more lidars (config/state reset)
  firmware_upgrade Upgrade the firmware on one or more lidars

Run  livox <verb> --help  for per-verb usage.
)";

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << kTopLevelUsage;
    return 1;
  }

  std::string verb(argv[1]);
  std::vector<std::string> args;
  for (int i = 2; i < argc; ++i) args.emplace_back(argv[i]);

  if (verb == "search") return verb_search(args);
  if (verb == "setup")  return verb_setup(args);
  if (verb == "reboot") return verb_reboot_or_reset(args, false);
  if (verb == "reset")  return verb_reboot_or_reset(args, true);
  if (verb == "firmware_upgrade") return verb_firmware_upgrade(args);

  std::cerr << "[err] Unknown verb: " << verb << "\n" << kTopLevelUsage;
  return 1;
}
