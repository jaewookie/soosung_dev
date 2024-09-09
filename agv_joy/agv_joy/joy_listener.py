import os
import select
import sys
import rclpy
import math

from geometry_msgs.msg import Twist
from agv_msgs.msg import ForkControl
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    # import termios
    import tty

MAX_LIN = 1.1
MAX_ANG = 0.7854

LIN_VEL = 0.05
ANG_VEL = 0.0524
FORK_VEL = 0.25

msg = """
-------------------------------------
            w : 전진  /  위 방향키

a : 좌회전   s : 정지   d : 우회전
   왼쪽 방향키       동그라미     오른쪽 방향키

            x : 후진  /  아래 방향키


u : 포크 상승  /  세모   i : 정지 / 엑스  o : 포크 하강 / 네모

CTRL-C : 종료
-------------------------------------
"""

e = """
Communications Failed
"""

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
    # if os.name != 'nt':
    #     settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    drive_pub = node.create_publisher(Twist, 'cmd_vel', qos)
    mast_pub = node.create_publisher(ForkControl, 'mast_vel', qos)

    # Joy subscriber to listen to the DualShock controller
    def joy_callback(joy_msg):
        nonlocal target_linear_velocity, target_angular_velocity, mast_vel, status

        target_linear_velocity = constrain(joy_msg.axes[1]*0.6, -MAX_LIN, MAX_LIN)
        target_angular_velocity = constrain(joy_msg.axes[0]*0.7854, -MAX_ANG, MAX_ANG)
        print_vels(target_linear_velocity, target_angular_velocity*180/math.pi)

        # Mapping buttons to actions
        # if joy_msg.axes[7]==1:  # Up on D-pad
        #     target_linear_velocity = constrain(target_linear_velocity+LIN_VEL, -MAX_LIN, MAX_LIN)
        #     print_vels(target_linear_velocity, target_angular_velocity*180/math.pi)
        # elif joy_msg.axes[7]==-1:  # Down on D-pad
        #     target_linear_velocity = constrain(target_linear_velocity-LIN_VEL, -MAX_LIN, MAX_LIN)
        #     print_vels(target_linear_velocity, target_angular_velocity*180/math.pi)
        # elif joy_msg.axes[7]==0 and joy_msg.axes[6]==0:
        #     target_linear_velocity = 0.0
        #     target_angular_velocity = 0.0
        #     print_vels(target_linear_velocity, target_angular_velocity*180/math.pi)
        # elif joy_msg.axes[6]==1:  # Left on D-pad
        #     target_angular_velocity = constrain(target_angular_velocity+ANG_VEL, -MAX_ANG, MAX_ANG)
        #     print_vels(target_linear_velocity, target_angular_velocity*180/math.pi)
        # elif joy_msg.axes[6]==-1:  # Right on D-pad
        #     target_angular_velocity = constrain(target_angular_velocity-ANG_VEL, -MAX_ANG, MAX_ANG)
        #     print_vels(target_linear_velocity, target_angular_velocity*180/math.pi)
        # elif joy_msg.buttons[1]==1:  # Circle
        #     target_linear_velocity = 0.0
        #     target_angular_velocity = 0.0
        #     print_vels(target_linear_velocity, target_angular_velocity*180/math.pi)
        if joy_msg.buttons[1]==1:
            mast_vel = FORK_VEL
            print_mast(mast_vel)
        elif joy_msg.buttons[1]==0 and joy_msg.buttons[3]==0:
            mast_vel = 0.0
            print_mast(mast_vel)
        elif joy_msg.buttons[3]==1:
            mast_vel = -FORK_VEL
            print_mast(mast_vel)

        publish_twist_and_fork()

    def publish_twist_and_fork():
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

    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    mast_vel = 0.0

    status = 0

    joy_sub = node.create_subscription(Joy, 'joy', joy_callback, qos)

    try:
        print(msg)
        rclpy.spin(node)

    except Exception as e:
        print(e)

    finally:
        target_linear_velocity = 0.0
        target_angular_velocity = 0.0
        mast_vel = 0.0
        publish_twist_and_fork()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
