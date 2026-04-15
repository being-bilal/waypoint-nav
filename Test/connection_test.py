# =============================================================================
# CONNECTION TEST  –  Verify Jetson ↔ Base Station UDP Link
# =============================================================================
# Run this script on EITHER machine to verify the network link is healthy.
#
#   On the BASE STATION (your laptop):
#       python connection_test.py --role base
#
#   On the VEHICLE (Jetson):
#       python connection_test.py --role vehicle
#
# The script runs five tests:
#   1. PING          – Can we reach the remote machine at all?
#   2. UDP SEND      – Can we push a packet to the remote machine?
#   3. UDP RECEIVE   – Does the remote machine's packet arrive here?
#   4. ECHO (round-trip) – Send a packet and get an echo reply (needs
#                          the other side running in listen-echo mode)
#   5. THROUGHPUT    – Burst-send packets and measure delivery rate
#
# You can also run it in LISTEN mode so the remote side can test against you:
#       python connection_test.py --role base --listen
# =============================================================================

import socket
import subprocess
import platform
import time
import json
import argparse
import threading
import sys

# ─── Network Config (mirrored from project constants.py) ────────────────────
# These MUST match Vehicle/constants.py and Basestation/constants.py
VEHICLE_IP    = "192.168.0.104"
BASE_IP       = "192.168.0.109"

# Port assignments (from constants):
#   Base → Vehicle waypoints:   port 5005
#   Vehicle → Base telemetry:   port 5006
WP_PORT       = 5005       # waypoints:  base sends, vehicle receives
TELEM_PORT    = 5006       # telemetry:  vehicle sends, base receives

# Test-specific port for the echo / throughput tests
ECHO_PORT     = 5099

# ─── ANSI colours for terminal output ────────────────────────────────────────
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

def banner(text):
    width = 60
    print(f"\n{CYAN}{'═' * width}")
    print(f"  {BOLD}{text}{RESET}{CYAN}")
    print(f"{'═' * width}{RESET}")

def result(label, passed, detail=""):
    icon = f"{GREEN}✅ PASS{RESET}" if passed else f"{RED}❌ FAIL{RESET}"
    det  = f"  ({detail})" if detail else ""
    print(f"  {icon}  {label}{det}")
    return passed


# =============================================================================
# TEST 1 – PING
# =============================================================================
def test_ping(remote_ip):
    banner("TEST 1 / 5 — PING")
    print(f"  Pinging {remote_ip} ...")

    # Platform-specific ping flag
    flag = "-n" if platform.system().lower() == "windows" else "-c"
    try:
        out = subprocess.run(
            ["ping", flag, "4", remote_ip],
            capture_output=True, text=True, timeout=15,
        )
        success = out.returncode == 0
        # Extract a summary line
        lines = out.stdout.strip().splitlines()
        summary = next((l for l in reversed(lines) if "%" in l or "loss" in l.lower()), "")
        return result("PING reachability", success, summary.strip())
    except Exception as e:
        return result("PING reachability", False, str(e))


# =============================================================================
# TEST 2 – UDP SEND  (fire a packet toward the remote machine)
# =============================================================================
def test_udp_send(remote_ip, port):
    banner("TEST 2 / 5 — UDP SEND")
    print(f"  Sending test packet to {remote_ip}:{port} ...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pkt  = json.dumps({"type": "connection_test", "ts": time.time()}).encode()
    try:
        sock.sendto(pkt, (remote_ip, port))
        return result("UDP send (no error)", True, f"{len(pkt)} bytes → {remote_ip}:{port}")
    except Exception as e:
        return result("UDP send", False, str(e))
    finally:
        sock.close()


# =============================================================================
# TEST 3 – UDP RECEIVE  (wait for ANY packet on the expected port)
# =============================================================================
def test_udp_receive(listen_port, timeout=5):
    banner("TEST 3 / 5 — UDP RECEIVE")
    print(f"  Listening on 0.0.0.0:{listen_port} for {timeout}s ...")
    print(f"  {YELLOW}(Make sure the other side sends a packet!){RESET}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", listen_port))
    sock.settimeout(timeout)
    try:
        data, addr = sock.recvfrom(4096)
        return result("UDP receive", True, f"{len(data)} bytes from {addr[0]}:{addr[1]}")
    except socket.timeout:
        return result("UDP receive", False, f"No packet received within {timeout}s")
    except Exception as e:
        return result("UDP receive", False, str(e))
    finally:
        sock.close()


# =============================================================================
# TEST 4 – ECHO ROUND-TRIP  (needs the remote side in --listen mode)
# =============================================================================
def test_echo(remote_ip, timeout=5):
    banner("TEST 4 / 5 — ECHO ROUND-TRIP")
    print(f"  Sending echo request to {remote_ip}:{ECHO_PORT} ...")
    print(f"  {YELLOW}(Remote must be running with --listen flag){RESET}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)

    send_ts = time.time()
    pkt = json.dumps({"type": "echo_request", "ts": send_ts}).encode()
    sock.sendto(pkt, (remote_ip, ECHO_PORT))

    try:
        data, addr = sock.recvfrom(4096)
        recv_ts = time.time()
        rtt_ms  = (recv_ts - send_ts) * 1000
        return result("Echo round-trip", True, f"RTT = {rtt_ms:.1f} ms from {addr[0]}")
    except socket.timeout:
        return result("Echo round-trip", False, "No echo reply (is remote in --listen mode?)")
    except Exception as e:
        return result("Echo round-trip", False, str(e))
    finally:
        sock.close()


# =============================================================================
# TEST 5 – THROUGHPUT  (burst-send N packets, count how many arrive)
# =============================================================================
def test_throughput(remote_ip, timeout=5, n_packets=100):
    banner("TEST 5 / 5 — THROUGHPUT")
    print(f"  Sending {n_packets} packets to {remote_ip}:{ECHO_PORT} ...")
    print(f"  {YELLOW}(Remote must be running with --listen flag){RESET}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)

    # Send burst
    t0 = time.time()
    for i in range(n_packets):
        pkt = json.dumps({"type": "throughput", "seq": i}).encode()
        sock.sendto(pkt, (remote_ip, ECHO_PORT))
    send_elapsed = time.time() - t0

    # Now ask for the report
    time.sleep(0.5)
    pkt = json.dumps({"type": "throughput_report_request"}).encode()
    sock.sendto(pkt, (remote_ip, ECHO_PORT))

    try:
        data, _ = sock.recvfrom(4096)
        report  = json.loads(data.decode())
        received = report.get("received", "?")
        detail = (f"Sent {n_packets} in {send_elapsed*1000:.0f}ms, "
                  f"remote received {received}/{n_packets}")
        return result("Throughput", int(received) == n_packets, detail)
    except socket.timeout:
        return result("Throughput", False, "No report received (is remote in --listen mode?)")
    except Exception as e:
        return result("Throughput", False, str(e))
    finally:
        sock.close()


# =============================================================================
# LISTEN / ECHO SERVER  (run on the "other" machine)
# =============================================================================
def run_listen_server():
    banner("ECHO / LISTEN SERVER")
    print(f"  Listening on 0.0.0.0:{ECHO_PORT}")
    print(f"  Waiting for echo and throughput requests ...")
    print(f"  {YELLOW}Press Ctrl+C to stop.{RESET}\n")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", ECHO_PORT))
    sock.settimeout(1.0)

    throughput_count = 0
    throughput_reset_time = time.time()

    try:
        while True:
            try:
                data, addr = sock.recvfrom(4096)
                pkt = json.loads(data.decode())
                ptype = pkt.get("type", "")

                if ptype == "echo_request":
                    # Reply immediately with echo_reply
                    reply = json.dumps({
                        "type": "echo_reply",
                        "original_ts": pkt.get("ts"),
                        "reply_ts": time.time(),
                    }).encode()
                    sock.sendto(reply, addr)
                    print(f"  {GREEN}↩ Echo reply{RESET} → {addr[0]}:{addr[1]}")

                elif ptype == "throughput":
                    throughput_count += 1

                elif ptype == "throughput_report_request":
                    reply = json.dumps({
                        "type": "throughput_report",
                        "received": throughput_count,
                    }).encode()
                    sock.sendto(reply, addr)
                    print(f"  {GREEN}📊 Throughput report{RESET}: {throughput_count} packets received → {addr[0]}")
                    throughput_count = 0  # reset for next burst

                elif ptype == "connection_test":
                    print(f"  {GREEN}📥 Connection test packet{RESET} from {addr[0]}:{addr[1]}")

                else:
                    print(f"  {YELLOW}📦 Unknown packet{RESET} from {addr[0]}: {ptype}")

            except socket.timeout:
                continue
    except KeyboardInterrupt:
        print(f"\n  {CYAN}Server stopped.{RESET}")
    finally:
        sock.close()


# =============================================================================
# MAIN
# =============================================================================
def main():
    parser = argparse.ArgumentParser(
        description="Test UDP connection between Base Station and Jetson (Vehicle).",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "--role", required=True, choices=["base", "vehicle"],
        help="Which machine you're running on:\n"
             "  base    = the laptop (Base Station)\n"
             "  vehicle = the Jetson (Vehicle / ASV)",
    )
    parser.add_argument(
        "--listen", action="store_true",
        help="Run as an echo/listen server so the OTHER side can test against you.",
    )
    parser.add_argument(
        "--vehicle-ip", default=VEHICLE_IP,
        help=f"Override vehicle IP (default: {VEHICLE_IP})",
    )
    parser.add_argument(
        "--base-ip", default=BASE_IP,
        help=f"Override base station IP (default: {BASE_IP})",
    )
    args = parser.parse_args()

    # Determine local/remote IPs based on role
    if args.role == "base":
        local_label  = "BASE STATION"
        remote_ip    = args.vehicle_ip
        remote_label = "VEHICLE (Jetson)"
        send_port    = WP_PORT        # base sends waypoints on 5005
        recv_port    = TELEM_PORT     # base receives telemetry on 5006
    else:
        local_label  = "VEHICLE (Jetson)"
        remote_ip    = args.base_ip
        remote_label = "BASE STATION"
        send_port    = TELEM_PORT     # vehicle sends telemetry on 5006
        recv_port    = WP_PORT        # vehicle receives waypoints on 5005

    print(f"\n{BOLD}{CYAN}╔══════════════════════════════════════════════════════════╗")
    print(f"║      JETSON ↔ BASE STATION  CONNECTION TESTER          ║")
    print(f"╚══════════════════════════════════════════════════════════╝{RESET}")
    print(f"  Role   : {BOLD}{local_label}{RESET}")
    print(f"  Remote : {remote_ip}  ({remote_label})")
    print(f"  Send →   port {send_port}")
    print(f"  Recv ←   port {recv_port}")

    # ── Listen-only mode ─────────────────────────────────────────────────
    if args.listen:
        run_listen_server()
        return

    # ── Run the test suite ───────────────────────────────────────────────
    results = []
    results.append(test_ping(remote_ip))
    results.append(test_udp_send(remote_ip, send_port))
    results.append(test_udp_receive(recv_port, timeout=10))
    results.append(test_echo(remote_ip, timeout=5))
    results.append(test_throughput(remote_ip, timeout=5))

    # ── Summary ──────────────────────────────────────────────────────────
    passed = sum(results)
    total  = len(results)
    banner("SUMMARY")
    if passed == total:
        print(f"  {GREEN}{BOLD}ALL {total} TESTS PASSED ✅{RESET}")
        print(f"  Connection between {local_label} and {remote_label} is solid.")
    else:
        print(f"  {YELLOW}{BOLD}{passed}/{total} tests passed{RESET}")
        if not results[0]:
            print(f"\n  {RED}💡 Ping failed — check:{RESET}")
            print(f"     • Are both devices on the same Wi-Fi network?")
            print(f"     • Is the IP correct? (Vehicle={args.vehicle_ip}, Base={args.base_ip})")
            print(f"     • Is the firewall blocking ICMP?")
        if not results[1]:
            print(f"\n  {RED}💡 UDP send failed — check:{RESET}")
            print(f"     • Firewall rules on the remote machine")
        if not results[2]:
            print(f"\n  {RED}💡 UDP receive failed — check:{RESET}")
            print(f"     • Is the other side actually sending packets?")
            print(f"     • Run this script on the other machine first with --role {'vehicle' if args.role == 'base' else 'base'}")
            print(f"     • Check firewall on THIS machine for port {recv_port}")
        if not results[3] or not results[4]:
            print(f"\n  {RED}💡 Echo/Throughput failed — check:{RESET}")
            print(f"     • Run the remote side with: python connection_test.py --role {'vehicle' if args.role == 'base' else 'base'} --listen")

    print()


if __name__ == "__main__":
    main()
