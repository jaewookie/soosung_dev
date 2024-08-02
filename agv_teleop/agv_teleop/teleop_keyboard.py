import os
import select
import sys
import rclpy
import math

from geometry_msgs.msg import Twist
from agv_msgs.msg import ForkControl
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

MAX_LIN = 1.1
MAX_ANG = 0.7854

LIN_VEL = 0.05
ANG_VEL = 0.0524
FORK_VEL = 0.25

msg = """
-------------------------------------
            w : 전진

a : 좌회전   s : 정지   d : 우회전

            x : 후진


u : 포크 상승 i : 정지  o : 포크 하강

CTRL-C : 종료
-------------------------------------
"""

e = """
Communications Failed
"""


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently: linear velocity {0}[m/s]\t steering angle {1}[degree] '.format(
        round(target_linear_velocity, 2),
        round(target_angular_velocity)))

def print_mast(mast_vel):
    print('currently:\tmast linear velocity {0} '.format(
        mast_vel))

def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel

def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    drive_pub = node.create_publisher(Twist, 'cmd_vel', qos)
    mast_pub = node.create_publisher(ForkControl, 'mast_vel', qos)

    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    mast_vel = 0.0

    try:
        print(msg)
        while(1):
            key = get_key(settings)
            if key == 'w':
                target_linear_velocity = constrain(target_linear_velocity+LIN_VEL, -MAX_LIN, MAX_LIN)
                target_angular_velocity = 0.0
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity*180/math.pi)
            elif key == 'x':
                target_linear_velocity = constrain(target_linear_velocity-LIN_VEL, -MAX_LIN, MAX_LIN)
                target_angular_velocity = 0.0
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity*180/math.pi)
            elif key == 'a':
                target_angular_velocity = constrain(target_angular_velocity+ANG_VEL, -MAX_ANG, MAX_ANG)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity*180/math.pi)
            elif key == 'd':
                target_angular_velocity = constrain(target_angular_velocity-ANG_VEL, -MAX_ANG, MAX_ANG)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity*180/math.pi)
            elif key == ' ' or key == 's':
                target_linear_velocity = 0.0
                target_angular_velocity = 0.0
                print_vels(target_linear_velocity, target_angular_velocity*180/math.pi)
            elif key == 'u':
                mast_vel = FORK_VEL
                print_mast(mast_vel)
            elif key == 'i':
                mast_vel = 0.0
                print_mast(mast_vel)
            elif key == 'o':
                mast_vel = -FORK_VEL
                print_mast(mast_vel)
            else:
                if (key == '\x03'):
                    break

            if status == 20:
                print(msg)
                status = 0

            twist = Twist()
            fork = ForkControl()

            twist.linear.x = target_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = target_angular_velocity

            drive_pub.publish(twist)

            fork.fork_vel = mast_vel
            mast_pub.publish(fork)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        drive_pub.publish(twist)

        fork.fork_vel = 0.0
        mast_pub.publish(fork)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
