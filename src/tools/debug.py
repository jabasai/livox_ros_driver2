#!/usr/bin/env python3
"""
UDP Diagnostic Script
Analyses UDP reception issues by testing at multiple levels simultaneously.
Usage: sudo python3 udp_diag.py --ip 192.168.1.10 --port <port> [--iface eno1] [--duration 15]
"""

import argparse
import socket
import subprocess
import threading
import time
import sys
import os
import struct
from datetime import datetime


# ──────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────

def header(title):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")

def run(cmd):
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        return result.stdout.strip(), result.stderr.strip()
    except subprocess.TimeoutExpired:
        return "", "timeout"

def ts():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


# ──────────────────────────────────────────────
# 1. Static network diagnostics
# ──────────────────────────────────────────────

def check_network(ip, iface):
    header("1. NETWORK CONFIGURATION")

    print("\n--- Interface addresses ---")
    out, _ = run(f"ip addr show {iface}")
    print(out or f"  [interface {iface} not found]")

    print("\n--- Routing table (relevant entries) ---")
    out, _ = run("ip route show")
    for line in out.splitlines():
        if "192.168.1" in line or "default" in line:
            print(f"  {line}")

    print(f"\n--- Route used to reach device 192.168.1.167 ---")
    out, _ = run("ip route get 192.168.1.167")
    print(f"  {out}")

    print(f"\n--- Route used to reach device 192.168.1.184 ---")
    out, _ = run("ip route get 192.168.1.184")
    print(f"  {out}")

    print(f"\n--- rp_filter settings ---")
    for key in ["all", iface, "wlp1s0"]:
        out, _ = run(f"sysctl net.ipv4.conf.{key}.rp_filter")
        print(f"  {out}")

    print(f"\n--- ARP cache for devices ---")
    out, _ = run("arp -n")
    for line in out.splitlines():
        if "192.168.1.16" in line or "192.168.1.18" in line:
            print(f"  {line}")
    if not any(x in out for x in ["192.168.1.16", "192.168.1.18"]):
        print("  [no ARP entries found for devices — try pinging them first]")


# ──────────────────────────────────────────────
# 2. Socket state check
# ──────────────────────────────────────────────

def check_sockets(ip, port):
    header("2. SOCKET STATE")

    print(f"\n--- All UDP sockets bound to {ip} or 0.0.0.0 ---")
    out, _ = run("ss -ulnp")
    found = False
    print(f"  {'Local Address':30s}  {'Process'}")
    for line in out.splitlines():
        if ip in line or "0.0.0.0" in line or "*" in line:
            if "State" in line or "Recv" in line:  # header
                continue
            print(f"  {line}")
            found = True
    if not found:
        print("  [no relevant UDP sockets found]")

    if port:
        print(f"\n--- Checking specifically for port {port} ---")
        out, _ = run(f"ss -ulnp sport = :{port}")
        print(out or f"  [nothing listening on UDP port {port}]")


# ──────────────────────────────────────────────
# 3. tcpdump capture (background thread)
# ──────────────────────────────────────────────

tcpdump_results = []

def run_tcpdump(iface, port, duration):
    port_filter = f"and udp port {port}" if port else "and udp"
    cmd = (
        f"tcpdump -i {iface} -n -c 50 -tt "
        f"'(src 192.168.1.167 or src 192.168.1.184) {port_filter}' "
        f"2>&1"
    )
    try:
        proc = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True)
        time.sleep(duration)
        proc.terminate()
        out = proc.stdout.read()
        tcpdump_results.extend(out.splitlines())
    except Exception as e:
        tcpdump_results.append(f"tcpdump error: {e}")


# ──────────────────────────────────────────────
# 4. Raw socket capture (no UDP stack involvement)
# ──────────────────────────────────────────────

raw_results = []

def run_raw_socket(iface, duration):
    """
    Captures at raw Ethernet level — bypasses routing, iptables INPUT,
    and socket binding entirely. If we see packets here but not on the
    UDP socket, the problem is above the NIC driver.
    """
    try:
        s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0800))
        s.bind((iface, 0))
        s.settimeout(1.0)
        deadline = time.time() + duration
        count = 0
        while time.time() < deadline:
            try:
                pkt = s.recv(65535)
                # Parse IP header (starts at byte 14 after Ethernet header)
                if len(pkt) < 34:
                    continue
                proto = pkt[23]
                if proto != 17:  # UDP only
                    continue
                src_ip = socket.inet_ntoa(pkt[26:30])
                dst_ip = socket.inet_ntoa(pkt[30:34])
                if src_ip in ("192.168.1.167", "192.168.1.184"):
                    dst_port = struct.unpack("!H", pkt[36:38])[0]
                    src_port = struct.unpack("!H", pkt[34:36])[0]
                    length   = struct.unpack("!H", pkt[38:40])[0]
                    raw_results.append(
                        f"  [{ts()}] UDP {src_ip}:{src_port} -> {dst_ip}:{dst_port}  "
                        f"payload={length-8}B"
                    )
                    count += 1
                    if count >= 20:
                        break
            except socket.timeout:
                continue
        s.close()
    except PermissionError:
        raw_results.append("  [raw socket requires root — run with sudo]")
    except Exception as e:
        raw_results.append(f"  [raw socket error: {e}]")


# ──────────────────────────────────────────────
# 5. UDP socket test listener
# ──────────────────────────────────────────────

udp_results = []

def run_udp_listener(ip, port, duration):
    if not port:
        udp_results.append("  [no port specified — skipping UDP listener test]")
        return
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        s.bind((ip, port))
        s.settimeout(1.0)
        udp_results.append(f"  Socket bound to {ip}:{port} — listening...")
        deadline = time.time() + duration
        count = 0
        while time.time() < deadline:
            try:
                data, addr = s.recvfrom(65535)
                udp_results.append(
                    f"  [{ts()}] Received {len(data)}B from {addr[0]}:{addr[1]}"
                )
                count += 1
                if count >= 20:
                    break
            except socket.timeout:
                continue
        if count == 0:
            udp_results.append(f"  [no UDP packets received on {ip}:{port} in {duration}s]")
        s.close()
    except OSError as e:
        udp_results.append(f"  [socket error: {e}]")


# ──────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="UDP reception diagnostics")
    parser.add_argument("--ip",       default="192.168.1.10",  help="Local IP to bind to")
    parser.add_argument("--port",     type=int, default=None,   help="UDP port to test")
    parser.add_argument("--iface",    default="eno1",           help="Ethernet interface name")
    parser.add_argument("--duration", type=int, default=15,     help="Capture duration in seconds")
    args = parser.parse_args()

    if os.geteuid() != 0:
        print("WARNING: Not running as root. Raw socket and tcpdump tests will fail.")
        print("         Re-run with: sudo python3 udp_diag.py ...\n")

    print(f"\nUDP Diagnostic Tool")
    print(f"  Target IP  : {args.ip}")
    print(f"  Port       : {args.port or 'not specified'}")
    print(f"  Interface  : {args.iface}")
    print(f"  Duration   : {args.duration}s")

    # Static checks first (instant)
    check_network(args.ip, args.iface)
    check_sockets(args.ip, args.port)

    # Launch parallel capture threads
    header(f"3. LIVE CAPTURE ({args.duration}s)")
    print(f"\n  Starting parallel captures — please ensure devices are streaming...\n")

    t_tcp  = threading.Thread(target=run_tcpdump,    args=(args.iface, args.port, args.duration))
    t_raw  = threading.Thread(target=run_raw_socket, args=(args.iface, args.duration))
    t_udp  = threading.Thread(target=run_udp_listener, args=(args.ip, args.port, args.duration))

    t_tcp.start(); t_raw.start(); t_udp.start()
    t_tcp.join();  t_raw.join();  t_udp.join()

    print("\n--- tcpdump (NIC via libpcap) ---")
    if tcpdump_results:
        for line in tcpdump_results:
            print(f"  {line}")
    else:
        print("  [no output]")

    print("\n--- Raw IP socket (kernel receive path, pre-iptables) ---")
    for line in raw_results:
        print(line)

    print(f"\n--- UDP socket bound to {args.ip}:{args.port or '?'} ---")
    for line in udp_results:
        print(line)

    # ── Interpretation ──
    header("4. INTERPRETATION")

    saw_tcpdump = any("192.168.1" in l for l in tcpdump_results)
    saw_raw     = len([l for l in raw_results if "UDP" in l]) > 0
    saw_udp     = any("Received" in l for l in udp_results)

    print()
    if not saw_tcpdump and not saw_raw:
        print("  ✗ No packets seen at NIC level (tcpdump + raw socket both empty)")
        print("    → Devices are not sending, or sending to wrong IP/port")
        print("    → Check device configuration and run: sudo tcpdump -i eno1 -n host 192.168.1.167")
    elif saw_tcpdump and saw_raw and not saw_udp:
        print("  ✓ Packets arriving at NIC")
        print("  ✗ Not reaching UDP socket")
        print("    → Check iptables INPUT rules, or port mismatch between device and listener")
    elif saw_tcpdump and not saw_raw:
        print("  ✓ tcpdump sees packets")
        print("  ✗ Raw socket does not — unusual, may indicate interface name mismatch")
    elif saw_udp:
        print("  ✓ Packets received successfully on UDP socket!")
        print("    → No reception problem detected during this run")
    else:
        print("  ✗ No packets seen at any level")
        print("    → Verify devices are actively streaming during the test window")

    print()


if __name__ == "__main__":
    main()
