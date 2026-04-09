#!/usr/bin/env python3
"""
rover_teleop_key.py  –  Keyboard teleoperation for the rover
=====================================================================
Keys:
  w  – move forward
  s  – move backward
  a  – turn left  (counter-clockwise)
  d  – turn right (clockwise)
  x  – full stop

  q  – increase linear speed   (+0.1)
  z  – decrease linear speed   (-0.1)
  e  – increase angular speed  (+0.1)
  c  – decrease angular speed  (-0.1)

  Ctrl-C  – quit
=====================================================================
Publishes geometry_msgs/Twist on /cmd_vel
"""

import sys
import tty
import termios
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ── ANSI colours ──────────────────────────────────────────────────
RESET  = "\033[0m"
BOLD   = "\033[1m"
CYAN   = "\033[96m"
YELLOW = "\033[93m"
GREEN  = "\033[92m"
RED    = "\033[91m"

BANNER = f"""
{BOLD}{CYAN}╔══════════════════════════════════════════╗
║        ROVER KEYBOARD TELEOP             ║
╠══════════════════════════════════════════╣
║  {YELLOW}w{CYAN}          – forward                       ║
║  {YELLOW}s{CYAN}          – backward                      ║
║  {YELLOW}a{CYAN}          – turn left                     ║
║  {YELLOW}d{CYAN}          – turn right                    ║
║  {YELLOW}x{CYAN}          – STOP                          ║
║                                          ║
║  {YELLOW}q / z{CYAN}      – increase / decrease linear    ║
║  {YELLOW}e / c{CYAN}      – increase / decrease angular   ║
║                                          ║
║  Ctrl-C    – quit                        ║
╚══════════════════════════════════════════╝{RESET}
"""

# Key → (linear_x, angular_z) as fractions of current speed
KEY_BINDINGS = {
    'w': ( 1.0,  0.0),
    's': (-1.0,  0.0),
    'a': ( 0.0,  1.0),
    'd': ( 0.0, -1.0),
    'x': ( 0.0,  0.0),
}

SPEED_BINDINGS = {
    'q': (0.1, 0.0),
    'z': (-0.1, 0.0),
    'e': (0.0, 0.1),
    'c': (0.0, -0.1),
}

MIN_SPEED = 0.05
MAX_LINEAR  = 2.0
MAX_ANGULAR = 3.14


def get_char():
    """Read a single character from stdin without waiting for Enter."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class TeleopNode(Node):

    def __init__(self):
        super().__init__('rover_teleop_key')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linear_speed  = 0.5   # m/s
        self.angular_speed = 1.0   # rad/s

        self._stop_event = threading.Event()

    # ── helpers ───────────────────────────────────────────────────

    def _clamp(self, value, lo, hi):
        return max(lo, min(hi, value))

    def _publish(self, lin, ang):
        msg = Twist()
        msg.linear.x  = lin
        msg.angular.z = ang
        self.pub.publish(msg)

    def _print_status(self, label: str):
        print(
            f"\r{GREEN}[{label}]{RESET}  "
            f"linear={YELLOW}{self.linear_speed:.2f} m/s{RESET}  "
            f"angular={YELLOW}{self.angular_speed:.2f} rad/s{RESET}   ",
            end='',
            flush=True,
        )

    # ── main loop ─────────────────────────────────────────────────

    def run(self):
        print(BANNER)
        self._print_status("ready")

        try:
            while not self._stop_event.is_set():
                key = get_char()

                # Ctrl-C
                if key == '\x03':
                    break

                if key in KEY_BINDINGS:
                    lin_frac, ang_frac = KEY_BINDINGS[key]
                    lin = lin_frac * self.linear_speed
                    ang = ang_frac * self.angular_speed
                    self._publish(lin, ang)

                    label = {
                        'w': f"↑ FORWARD  {lin:+.2f} m/s",
                        's': f"↓ BACKWARD {lin:+.2f} m/s",
                        'a': f"← TURN LEFT  {ang:+.2f} r/s",
                        'd': f"→ TURN RIGHT {ang:+.2f} r/s",
                        'x': "■ STOP",
                    }[key]
                    self._print_status(label)

                elif key in SPEED_BINDINGS:
                    dl, da = SPEED_BINDINGS[key]
                    self.linear_speed  = self._clamp(
                        self.linear_speed  + dl, MIN_SPEED, MAX_LINEAR)
                    self.angular_speed = self._clamp(
                        self.angular_speed + da, MIN_SPEED, MAX_ANGULAR)
                    self._print_status("speed updated")

        except Exception as exc:
            print(f"\n{RED}Error: {exc}{RESET}")
        finally:
            # Always send a stop command on exit
            self._publish(0.0, 0.0)
            print(f"\n{RED}Stopped. Bye!{RESET}\n")


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    # Spin ROS in background thread so publishers stay alive
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
