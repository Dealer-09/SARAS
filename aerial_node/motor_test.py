#!/usr/bin/env python3
"""
=============================================================================
  Sentinel — FC Connection Verifier + Motor Spin Test
  MicoAir H743 V2 (ArduPilot) via USB-C → /dev/ttyACM0

  What this does:
    1. Connects to FC via MAVLink
    2. Prints full FC health: firmware, IMU, baro, GPS, battery, EKF
    3. Requests all telemetry streams
    4. Optionally runs MAVLink motor test (DO_MOTOR_TEST) on all 4 motors
       at a safe 10% throttle for 2 seconds each

  SAFETY:
    - Remove ALL propellers before running motor test
    - Motor test only runs if you pass --motors flag explicitly
    - Default run is read-only health check (safe with props on)

  Usage:
    # Health check only (safe, props can be on):
    python3 motor_test.py

    # Full motor spin test (REMOVE PROPS FIRST):
    python3 motor_test.py --motors

    # Custom throttle % and duration:
    python3 motor_test.py --motors --throttle 15 --duration 1.5

    # If using GPIO UART instead of USB:
    python3 motor_test.py --port /dev/serial0 --baud 921600
=============================================================================
"""

import sys
import time
import argparse
import threading

try:
    from pymavlink import mavutil
except ImportError:
    print("[ERROR] pymavlink not installed.")
    print("        Run: ~/drone_venv/bin/pip install pymavlink")
    sys.exit(1)


# ── Defaults matching MicoAir H743 V2 USB-C (SERIAL0) ──────────────────────
DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUD = 115200

# Motor test parameters
MOTOR_COUNT    = 4       # quadrotor
MOTOR_TYPE     = 0       # 0 = throttle %, 1 = RPM
THROTTLE_PCT   = 10      # % — safe for a bench-clamped drone without props


# ── ANSI colour helpers ──────────────────────────────────────────────────────
GREEN  = '\033[92m'
YELLOW = '\033[93m'
RED    = '\033[91m'
CYAN   = '\033[96m'
RESET  = '\033[0m'

def ok(msg):    print(f"  {GREEN}✅ {msg}{RESET}")
def warn(msg):  print(f"  {YELLOW}⚠️  {msg}{RESET}")
def err(msg):   print(f"  {RED}❌ {msg}{RESET}")
def info(msg):  print(f"  {CYAN}ℹ  {msg}{RESET}")


# ── Telemetry collector (runs in background thread) ──────────────────────────
class Telemetry:
    def __init__(self):
        self.armed          = False
        self.base_mode      = 0
        self.custom_mode    = 0
        self.autopilot_type = None
        self.firmware_str   = None
        self.alt_rel        = 0.0
        self.battery_v      = 0.0
        self.battery_pct    = -1
        self.gps_fix        = 0
        self.gps_sats       = 0
        self.imu_ok         = False
        self.baro_ok        = False
        self.ekf_ok         = False
        self.ekf_flags      = 0
        self.ekf_velocity_ok   = False
        self.ekf_pos_horiz_ok  = False
        self.ekf_pos_vert_ok   = False
        self.attitude_roll  = 0.0
        self.attitude_pitch = 0.0
        self.attitude_yaw   = 0.0
        self.sys_errors     = 0
        self.vibration_x    = 0.0
        self.vibration_y    = 0.0
        self.vibration_z    = 0.0
        self.heartbeat_count = 0
        self.last_hb_time   = 0.0
        self._lock          = threading.Lock()

    def process(self, msg):
        t = msg.get_type()
        with self._lock:
            if t == 'HEARTBEAT':
                self.armed          = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.base_mode      = msg.base_mode
                self.custom_mode    = msg.custom_mode
                self.autopilot_type = msg.autopilot
                self.heartbeat_count += 1
                self.last_hb_time   = time.time()

            elif t == 'AUTOPILOT_VERSION':
                maj = (msg.flight_sw_version >> 24) & 0xFF
                min_ = (msg.flight_sw_version >> 16) & 0xFF
                pat = (msg.flight_sw_version >> 8) & 0xFF
                self.firmware_str = f"ArduPilot {maj}.{min_}.{pat}"

            elif t == 'GLOBAL_POSITION_INT':
                self.alt_rel = msg.relative_alt / 1000.0

            elif t == 'GPS_RAW_INT':
                self.gps_fix  = msg.fix_type
                self.gps_sats = msg.satellites_visible

            elif t == 'BATTERY_STATUS':
                if msg.battery_remaining >= 0:
                    self.battery_pct = msg.battery_remaining
                if msg.voltages and msg.voltages[0] != 65535:
                    self.battery_v = msg.voltages[0] / 1000.0

            elif t == 'SYS_STATUS':
                # Sensor health bitmask
                sensors_ok  = msg.onboard_control_sensors_health
                sensors_pres = msg.onboard_control_sensors_present
                IMU_BIT  = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL
                BARO_BIT = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
                self.imu_ok  = bool(sensors_ok & IMU_BIT)  if bool(sensors_pres & IMU_BIT)  else None
                self.baro_ok = bool(sensors_ok & BARO_BIT) if bool(sensors_pres & BARO_BIT) else None
                self.sys_errors = msg.errors_count1
                # Battery fallback
                if self.battery_pct < 0 and msg.battery_remaining >= 0:
                    self.battery_pct = msg.battery_remaining
                if self.battery_v == 0.0 and msg.voltage_battery != 65535:
                    self.battery_v = msg.voltage_battery / 1000.0

            elif t == 'ATTITUDE':
                import math
                self.attitude_roll  = math.degrees(msg.roll)
                self.attitude_pitch = math.degrees(msg.pitch)
                self.attitude_yaw   = math.degrees(msg.yaw)

            elif t == 'EKF_STATUS_REPORT':
                # Flags from ArduPilot EKF_STATUS_REPORT
                EKF_VELOCITY_HORIZ = 0x01
                EKF_VELOCITY_VERT  = 0x02
                EKF_POS_HORIZ_REL  = 0x04
                EKF_POS_HORIZ_ABS  = 0x08
                EKF_POS_VERT_ABS   = 0x10
                EKF_POS_VERT_AGL   = 0x20
                EKF_CONST_POS_MODE = 0x40
                EKF_PRED_POS_HORIZ = 0x80
                flags = msg.flags
                self.ekf_flags = flags
                self.ekf_velocity_ok  = bool(flags & (EKF_VELOCITY_HORIZ | EKF_VELOCITY_VERT))
                self.ekf_pos_horiz_ok = bool(flags & (EKF_POS_HORIZ_REL | EKF_POS_HORIZ_ABS))
                self.ekf_pos_vert_ok  = bool(flags & (EKF_POS_VERT_ABS  | EKF_POS_VERT_AGL))
                self.ekf_ok = self.ekf_velocity_ok and self.ekf_pos_vert_ok

            elif t == 'VIBRATION':
                self.vibration_x = msg.vibration_x
                self.vibration_y = msg.vibration_y
                self.vibration_z = msg.vibration_z


def request_streams(mav):
    """Request all telemetry stream groups at useful rates."""
    streams = [
        (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 5),   # GPS_RAW_INT, SYS_STATUS
        (mavutil.mavlink.MAV_DATA_STREAM_POSITION,        5),   # GLOBAL_POSITION_INT
        (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,          10),  # ATTITUDE
        (mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,          2),   # BATTERY_STATUS, EKF_STATUS
        (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,     2),   # RAW_IMU
    ]
    for sid, rate in streams:
        mav.mav.request_data_stream_send(
            mav.target_system, mav.target_component,
            sid, rate, 1
        )
    # Also request AUTOPILOT_VERSION and VIBRATION individually
    for msg_id in [148, 241]:  # AUTOPILOT_VERSION=148, VIBRATION=241
        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0, msg_id, 0, 0, 0, 0, 0, 0
        )


def print_health_report(tel: Telemetry):
    """Print a formatted health report."""
    print()
    print(f"{CYAN}{'─'*56}")
    print(f"  MicoAir H743 V2 — FC Health Report")
    print(f"{'─'*56}{RESET}")

    # Connection
    hb_age = time.time() - tel.last_hb_time if tel.last_hb_time > 0 else 999
    if tel.heartbeat_count > 0 and hb_age < 3.0:
        ok(f"FC Link       — {tel.heartbeat_count} heartbeats, last {hb_age:.1f}s ago")
    else:
        err(f"FC Link       — no heartbeat (age={hb_age:.0f}s)")

    # Firmware
    if tel.firmware_str:
        ok(f"Firmware      — {tel.firmware_str}")
    else:
        warn("Firmware      — version not received (AUTOPILOT_VERSION not streamed)")

    # Arm state
    if tel.armed:
        warn("Arm State     — ARMED ⚠️  (motors may spin)")
    else:
        ok("Arm State     — DISARMED (safe)")

    # IMU
    if tel.imu_ok is True:
        ok(f"IMU (BMI088+BMI270) — healthy | "
           f"Roll={tel.attitude_roll:+.1f}° Pitch={tel.attitude_pitch:+.1f}° Yaw={tel.attitude_yaw:+.1f}°")
    elif tel.imu_ok is False:
        err("IMU           — UNHEALTHY")
    else:
        warn("IMU           — status not received")

    # Vibration
    vib = max(tel.vibration_x, tel.vibration_y, tel.vibration_z)
    if vib > 0:
        if vib < 30:
            ok(f"Vibration     — X={tel.vibration_x:.1f} Y={tel.vibration_y:.1f} Z={tel.vibration_z:.1f} (OK)")
        elif vib < 60:
            warn(f"Vibration     — X={tel.vibration_x:.1f} Y={tel.vibration_y:.1f} Z={tel.vibration_z:.1f} (elevated)")
        else:
            err(f"Vibration     — X={tel.vibration_x:.1f} Y={tel.vibration_y:.1f} Z={tel.vibration_z:.1f} (TOO HIGH)")

    # Barometer
    if tel.baro_ok is True:
        ok(f"Baro (SPL06)  — healthy | Alt_rel={tel.alt_rel:.2f}m")
    elif tel.baro_ok is False:
        err("Baro          — UNHEALTHY")
    else:
        warn("Baro          — status not received")

    # GPS (NEO-6M on UART3/SERIAL3)
    fix_names = {0:'NO FIX', 1:'NO FIX', 2:'2D FIX', 3:'3D FIX', 4:'DGPS', 5:'RTK FLOAT', 6:'RTK FIXED'}
    fix_str = fix_names.get(tel.gps_fix, f'fix={tel.gps_fix}')
    sats_str = f"{tel.gps_sats} sats" if tel.gps_sats < 255 else "sats=unknown"
    if tel.gps_fix >= 3:
        ok(f"GPS (NEO-6M)  — {fix_str} | {sats_str}")
    elif tel.gps_fix >= 1:
        warn(f"GPS (NEO-6M)  — {fix_str} | {sats_str} (waiting for lock)")
    else:
        warn(f"GPS (NEO-6M)  — {fix_str} | {sats_str} (no fix or GPS_TYPE=0)")

    # Battery
    if tel.battery_pct >= 0:
        batt_str = f"{tel.battery_pct}% ({tel.battery_v:.2f}V)"
        if tel.battery_pct > 50:
            ok(f"Battery       — {batt_str}")
        elif tel.battery_pct > 20:
            warn(f"Battery       — {batt_str} (low)")
        else:
            err(f"Battery       — {batt_str} (CRITICAL)")
    else:
        warn("Battery       — not received (check SR0_EXTRA3 stream rate)")

    # EKF
    if tel.ekf_flags > 0:
        ekf_parts = []
        if tel.ekf_velocity_ok:  ekf_parts.append("vel✓")
        else:                    ekf_parts.append("vel✗")
        if tel.ekf_pos_horiz_ok: ekf_parts.append("pos_h✓")
        else:                    ekf_parts.append("pos_h✗")
        if tel.ekf_pos_vert_ok:  ekf_parts.append("pos_v✓")
        else:                    ekf_parts.append("pos_v✗")
        ekf_summary = " ".join(ekf_parts)
        if tel.ekf_ok:
            ok(f"EKF           — healthy | {ekf_summary} | flags=0x{tel.ekf_flags:02X}")
        else:
            warn(f"EKF           — degraded | {ekf_summary} | flags=0x{tel.ekf_flags:02X}")
    else:
        warn("EKF           — status not received")

    # System errors
    if tel.sys_errors > 0:
        err(f"System Errors — count={tel.sys_errors} (check GCS for pre-arm failures)")
    else:
        ok("System Errors — none")

    print(f"{CYAN}{'─'*56}{RESET}")
    print()


def run_motor_test(mav, throttle_pct: float, duration_s: float):
    """
    Run MAVLink DO_MOTOR_TEST on all 4 motors sequentially.

    MAV_CMD_DO_MOTOR_TEST (209):
      param1 = motor number (1-based)
      param2 = throttle type (0 = % of max)
      param3 = throttle value (0–100 for type 0)
      param4 = timeout seconds
      param5 = motor count (0 = just param1 motor)
      param6 = test order (0 = default)

    Ref: https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOTOR_TEST
    """
    print()
    print(f"{YELLOW}{'─'*56}")
    print(f"  ⚠️  MOTOR TEST — {MOTOR_COUNT} motors at {throttle_pct:.0f}% for {duration_s:.1f}s each")
    print(f"  CONFIRM: propellers are REMOVED")
    print(f"{'─'*56}{RESET}")
    response = input("  Type YES to proceed: ").strip()
    if response != "YES":
        print("  Aborted.")
        return

    for motor_num in range(1, MOTOR_COUNT + 1):
        print(f"\n  Spinning Motor {motor_num}...")
        mav.mav.command_long_send(
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
            0,                # confirmation
            float(motor_num), # param1: motor number (1-based)
            float(MOTOR_TYPE),# param2: throttle type (0=%)
            float(throttle_pct), # param3: throttle %
            float(duration_s),   # param4: duration seconds
            0.0,              # param5: motor count (0 = just this motor)
            0.0,              # param6: test order
            0.0               # param7: unused
        )

        # Wait for COMMAND_ACK
        ack = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=3.0)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST:
            result_names = {
                0: 'ACCEPTED',
                1: 'TEMPORARILY_REJECTED',
                2: 'DENIED',
                3: 'UNSUPPORTED',
                4: 'FAILED',
                5: 'IN_PROGRESS',
            }
            result_str = result_names.get(ack.result, f'result={ack.result}')
            if ack.result == 0:
                ok(f"Motor {motor_num} — {result_str}")
            else:
                err(f"Motor {motor_num} — {result_str}")
                warn("  ArduPilot may require motors to be DISARMED for motor test")
                warn("  Or try: set MOT_TEST_THROTTLE_TYPE=0 in Mission Planner")
        else:
            warn(f"Motor {motor_num} — no ACK received (timeout)")

        # Wait for motor to finish its run + small gap
        time.sleep(duration_s + 0.5)

    print()
    ok("Motor test complete")


def main():
    parser = argparse.ArgumentParser(
        description='MicoAir H743 V2 FC connection verifier + motor test'
    )
    parser.add_argument('--port',     default=DEFAULT_PORT,
                        help=f'Serial port (default: {DEFAULT_PORT})')
    parser.add_argument('--baud',     type=int, default=DEFAULT_BAUD,
                        help=f'Baud rate (default: {DEFAULT_BAUD}). Use 921600 for GPIO UART')
    parser.add_argument('--motors',   action='store_true',
                        help='Run motor spin test after health check (REMOVE PROPS FIRST)')
    parser.add_argument('--throttle', type=float, default=THROTTLE_PCT,
                        help=f'Motor test throttle %% (default: {THROTTLE_PCT})')
    parser.add_argument('--duration', type=float, default=2.0,
                        help='Motor test duration per motor in seconds (default: 2.0)')
    parser.add_argument('--wait',     type=float, default=8.0,
                        help='Seconds to collect telemetry before printing report (default: 8)')
    args = parser.parse_args()

    print()
    print(f"{CYAN}{'═'*56}")
    print(f"  Sentinel — FC Connection Verifier")
    print(f"  Port: {args.port} @ {args.baud} baud")
    print(f"{'═'*56}{RESET}")

    # ── Connect ──────────────────────────────────────────────────────────────
    print(f"\n  Connecting to FC...")
    try:
        mav = mavutil.mavlink_connection(
            args.port, baud=args.baud,
            source_system=255,   # GCS system ID
            source_component=0,
            autoreconnect=True
        )
    except Exception as e:
        err(f"Could not open serial port {args.port}: {e}")
        err("Check: FC powered on, USB cable plugged in, correct port")
        err("Try:   ls /dev/ttyACM* /dev/ttyUSB* /dev/serial*")
        sys.exit(1)

    # ── Wait for heartbeat ───────────────────────────────────────────────────
    print(f"  Waiting for heartbeat (timeout 10s)...")
    hb = mav.wait_heartbeat(timeout=10)
    if hb is None:
        err("No heartbeat received in 10s")
        err("Check: ArduPilot running on FC, correct baud rate")
        err(f"  USB baud  = 115200  (SERIAL0 on MicoAir H743 V2)")
        err(f"  GPIO baud = 921600  (SERIAL1 / UART1)")
        sys.exit(1)

    ok(f"Heartbeat received! system={mav.target_system} component={mav.target_component}")

    # ── Request streams ───────────────────────────────────────────────────────
    print(f"  Requesting telemetry streams...")
    request_streams(mav)

    # ── Collect telemetry ─────────────────────────────────────────────────────
    tel = Telemetry()
    print(f"  Collecting telemetry for {args.wait:.0f}s...")

    deadline = time.time() + args.wait
    while time.time() < deadline:
        msg = mav.recv_match(blocking=True, timeout=0.5)
        if msg:
            tel.process(msg)
        remaining = deadline - time.time()
        print(f"\r  [{args.wait - remaining:.1f}s / {args.wait:.0f}s] "
              f"HB={tel.heartbeat_count} "
              f"GPS={tel.gps_fix} "
              f"Batt={tel.battery_pct if tel.battery_pct >= 0 else '?'}% "
              f"Alt={tel.alt_rel:.1f}m    ",
              end='', flush=True)

    print()  # newline after progress

    # ── Print report ──────────────────────────────────────────────────────────
    print_health_report(tel)

    # ── Motor test ────────────────────────────────────────────────────────────
    if args.motors:
        run_motor_test(mav, args.throttle, args.duration)
    else:
        info("To spin motors: run with --motors flag (REMOVE PROPS FIRST)")
        info(f"  python3 motor_test.py --motors --throttle {args.throttle} --duration {args.duration}")

    mav.close()
    print()


if __name__ == '__main__':
    main()
