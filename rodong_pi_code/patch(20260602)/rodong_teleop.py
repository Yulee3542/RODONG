#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
rodong_teleop.py  —  RODONG13 keyboard control node
Topics published:
  /rodong/cmd         std_msgs/String   "INIT" | "STOP"
  /rodong/manual_goal geometry_msgs/Point  x, y (cm), z=0
"""

import sys
import tty
import termios
import threading
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point

BANNER = """
============================================================
  RODONG Teleop  (RODONG13)
------------------------------------------------------------
  i        : INIT  — start autonomous driving
  s        : STOP  — full stop
  m        : MANUAL — enter coordinate-input mode
  u        : UTURN  — U-turn test
  q / Ctrl+C : quit
============================================================
"""

MANUAL_HELP = """
[MANUAL MODE]  Enter a goal coordinate (relative to current position, in cm)
  e.g.)  100 0     →  100cm ahead
         0 50      →  50cm to the left
        -80 0      →  80cm behind
  'b' + Enter   →  cancel MANUAL mode (return to autonomous)
"""


def get_key():
    """Read a single key from the terminal (non-blocking)"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def get_line():
    """MANUAL mode: restore the terminal to normal mode, then read one line"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    termios.tcsetattr(fd, termios.TCSADRAIN, old)  # already normal, but just in case
    return sys.stdin.readline().strip()


class TeleopNode:
    def __init__(self):
        rospy.init_node('rodong_teleop', anonymous=False)
        self.pub_cmd  = rospy.Publisher('/rodong/cmd',         String, queue_size=1)
        self.pub_goal = rospy.Publisher('/rodong/manual_goal', Point,  queue_size=1)
        self.manual_mode = False

    def send_cmd(self, cmd):
        msg = String()
        msg.data = cmd
        self.pub_cmd.publish(msg)
        rospy.loginfo('[Teleop] CMD → %s', cmd)

    def send_goal(self, x_cm, y_cm):
        msg = Point()
        msg.x = float(x_cm)
        msg.y = float(y_cm)
        msg.z = 0.0
        self.pub_goal.publish(msg)
        rospy.loginfo('[Teleop] GOAL → x=%.1f cm  y=%.1f cm', x_cm, y_cm)

    def enter_manual(self):
        self.manual_mode = True
        self.send_cmd('MANUAL')
        print(MANUAL_HELP)
        while not rospy.is_shutdown() and self.manual_mode:
            try:
                line = get_line()
            except (EOFError, KeyboardInterrupt):
                break

            if line.lower() == 'b':
                print('[MANUAL] cancel → return to autonomous')
                self.manual_mode = False
                self.send_cmd('INIT')
                break

            parts = line.split()
            if len(parts) == 2:
                try:
                    x = float(parts[0])
                    y = float(parts[1])
                    self.send_goal(x, y)
                    print('[MANUAL] goal sent: x={:.1f}  y={:.1f} cm'.format(x, y))
                    print("  enter next coordinate (or 'b' to exit):")
                except ValueError:
                    print('[MANUAL] enter numbers. e.g.) 100 0')
            else:
                print("[MANUAL] format error. e.g.) 100 0  /  'b' to exit")

    def run(self):
        print(BANNER)
        print('Node ready. Press a key...')
        while not rospy.is_shutdown():
            key = get_key()

            if key in ('\x03', 'q'):          # Ctrl+C or q
                print('\n[Teleop] quitting.')
                self.send_cmd('STOP')
                break

            elif key == 'i':
                self.send_cmd('INIT')
                print('[Teleop] autonomous driving started')

            elif key == 's':
                self.send_cmd('STOP')
                print('[Teleop] stop')

            elif key == 'm':
                print('[Teleop] entering MANUAL mode')
                self.enter_manual()

            elif key == 'u':
                self.send_cmd('UTURN')
                print('[Teleop] UTURN started')

            else:
                pass  # ignore other keys


if __name__ == '__main__':
    try:
        TeleopNode().run()
    except rospy.ROSInterruptException:
        pass
