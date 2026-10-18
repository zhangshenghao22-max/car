#!/usr/bin/env python3
import argparse
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist

HELP = """
RK3588 RT keyboard teleop -> /cmd_vel

w/s: forward/backward
a/d: left/right strafe
q/e: turn left/right
x or space: stop
r: reset speed to zero
+: increase speed steps
-: decrease speed steps
Ctrl-C: quit, publishes stop
"""


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def make_msg(vx, vy, wz):
    msg = Twist()
    msg.linear.x = float(vx)
    msg.linear.y = float(vy)
    msg.angular.z = float(wz)
    return msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/cmd_vel')
    parser.add_argument('--linear-step', type=float, default=0.03)
    parser.add_argument('--angular-step', type=float, default=0.05)
    parser.add_argument('--max-linear', type=float, default=0.20)
    parser.add_argument('--max-angular', type=float, default=0.30)
    parser.add_argument('--rate', type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('keyboard_teleop')
    pub = node.create_publisher(Twist, args.topic, 10)

    vx = vy = wz = 0.0
    old = termios.tcgetattr(sys.stdin)
    print(HELP)
    print(f'Publishing {args.topic}. Waiting for subscriber...')
    try:
        tty.setcbreak(sys.stdin.fileno())
        period = 1.0 / max(args.rate, 1.0)
        while rclpy.ok():
            if select.select([sys.stdin], [], [], period)[0]:
                ch = sys.stdin.read(1)
                if ch == '\x03':
                    break
                elif ch == 'w':
                    vx = clamp(vx + args.linear_step, -args.max_linear, args.max_linear)
                elif ch == 's':
                    vx = clamp(vx - args.linear_step, -args.max_linear, args.max_linear)
                elif ch == 'a':
                    vy = clamp(vy + args.linear_step, -args.max_linear, args.max_linear)
                elif ch == 'd':
                    vy = clamp(vy - args.linear_step, -args.max_linear, args.max_linear)
                elif ch == 'q':
                    wz = clamp(wz + args.angular_step, -args.max_angular, args.max_angular)
                elif ch == 'e':
                    wz = clamp(wz - args.angular_step, -args.max_angular, args.max_angular)
                elif ch in ('x', ' ', 'r'):
                    vx = vy = wz = 0.0
                elif ch == '+':
                    vx = clamp(vx + args.linear_step, -args.max_linear, args.max_linear)
                elif ch == '-':
                    vx = clamp(vx - args.linear_step, -args.max_linear, args.max_linear)
                print(f'cmd vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}')
            pub.publish(make_msg(vx, vy, wz))
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        pub.publish(make_msg(0.0, 0.0, 0.0))
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()
        print('\nStopped and published zero /cmd_vel.')


if __name__ == '__main__':
    main()
