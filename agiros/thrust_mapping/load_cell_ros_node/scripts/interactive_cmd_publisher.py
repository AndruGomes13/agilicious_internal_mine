#!/usr/bin/python3
import math, sys, tty, termios, select
import rospy
from agiros_msgs.msg import Command
from geometry_msgs.msg import Vector3

RATE_HZ   = 100                   # loop rate
TIMEOUT   = 1 / RATE_HZ * 0.9     # key poll timeout

def make_msg(thrust, roll, pitch, yaw):
    msg = Command()
    msg.is_single_rotor_thrust = False
    msg.collective_thrust      = thrust
    msg.bodyrates              = Vector3(roll, pitch, yaw)
    return msg

class Keyboard:
    """non-blocking key reader (handles arrow sequences)"""
    def __init__(self):
        self._settings = termios.tcgetattr(sys.stdin)

    def read(self, timeout=TIMEOUT):
        tty.setraw(sys.stdin.fileno())
        r, *_ = select.select([sys.stdin], [], [], timeout)
        key = ''
        if r:
            first = sys.stdin.read(1)
            if first == '\x1b':     # escape – possible arrow key
                key = first + sys.stdin.read(2)  # e.g. '\x1b[A'
            else:
                key = first
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._settings)
        return key

if __name__ == "__main__":
    rospy.init_node("interactive_cmd_publisher")

    # params
    min_cmd   = rospy.get_param("~min_cmd",   192)
    max_cmd   = rospy.get_param("~max_cmd",  1792)
    inc_thrust= rospy.get_param("~inc_step",   25.0)
    rate_step = rospy.get_param("~rate_step",   0.1)   # rad/s per key-press
    max_rate  = rospy.get_param("~max_rate",    4.0)   # rad/s limit

    pub  = rospy.Publisher("/command", Command, queue_size=1)
    rate = rospy.Rate(RATE_HZ)
    kb   = Keyboard()

    thrust = min_cmd
    roll = pitch = yaw = 0.0
    typed = ''

    print("↑/↓ thrust | Q/A pitch | W/S roll | E/D yaw | number+Enter sets thrust | z/ESC quits")

    while not rospy.is_shutdown():
        k = kb.read()

        # thrust (arrow up / down)
        if k == '\x1b[A':                          # ↑
            thrust = min(thrust + inc_thrust, max_cmd)
        elif k == '\x1b[B':                        # ↓
            thrust = max(thrust - inc_thrust, min_cmd)

        # body-rates -------------------------------------------
        elif k in ('q', 'Q'):
            pitch = min(pitch + rate_step,  max_rate)
        elif k in ('a', 'A'):
            pitch = max(pitch - rate_step, -max_rate)
        elif k in ('s', 'S'):
            roll  = max(roll  - rate_step, -max_rate)
        elif k in ('w', 'W'):
            roll  = min(roll  + rate_step,  max_rate)
        elif k in ('e', 'E'):                        # →
            yaw   = min(yaw   + rate_step,  max_rate)
        elif k in ('d', 'D'):                        # ←
            yaw   = max(yaw   - rate_step, -max_rate)

        # numeric thrust entry -------------------------------
        elif k in ('\r', '\n'):                    # Enter
            if typed:
                try:
                    thrust = max(min(float(typed), max_cmd), min_cmd)
                except ValueError:
                    rospy.logwarn("Invalid number")
                typed = ''
        elif k and k.isprintable():
            if k.lower() == 'z' or k == '\x1b':    # quit
                break
            typed += k

        pub.publish(make_msg(thrust, roll, pitch, yaw))
        rate.sleep()

    print("\nExiting…")
    rospy.loginfo("Interactive command publisher terminated.")