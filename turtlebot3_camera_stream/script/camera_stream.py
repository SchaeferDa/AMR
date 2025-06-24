#!/usr/bin/env python
#
# 
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Darby Lim

import os
import select
import sys

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.clock import Clock
from rclpy.qos import QoSProfile
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from turtlebot3_camera_stream.inference import Inference
import numpy as np
import cv2
from threading import Thread, Event


if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.03
ANG_VEL_STEP_SIZE = 0.1

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a         d
        s

w/s : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

class CompressedImageSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.buffer = None

    def listener_callback(self, msg: CompressedImage):
        self.buffer = msg.data
        

class TurtleBotController():
    def __init__(self):
        print('INIT TB\n')
        self.status = 0
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.ROS_DISTRO = os.environ.get('ROS_DISTRO')
        self.qos = QoSProfile(depth=10)
        self.node = rclpy.create_node('teleop_keyboard')
        if self.ROS_DISTRO == 'humble':
            self.pub = self.node.create_publisher(Twist, 'cmd_vel', self.qos)
        else:
            self.pub = self.node.create_publisher(TwistStamped, 'cmd_vel', self.qos)
   
        
    def turn_left(self):
        print('TURNING LEFT')
        self.target_linear_velocity = get_linear_limit_velocity()
        self.target_angular_velocity = get_angular_limit_velocity()
        self.publish()
            
    def turn_right(self):
        print('TURNING RIGHT')
        self.target_linear_velocity = get_linear_limit_velocity()
        self.target_angular_velocity = -get_angular_limit_velocity()
        self.publish()

    
    def drive_straight(self):
        print('DRIVING STRAIGHT')
        self.target_linear_velocity = get_linear_limit_velocity()
        self.target_angular_velocity = 0.0
        self.publish()

    def stop(self):
        print('STOPPING')
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.publish()

    def publish(self):
        if self.ROS_DISTRO == 'humble':
            twist = Twist()
            twist.linear.x = self.target_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = self.target_angular_velocity
            self.pub.publish(twist)
        else:
            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = Clock().now().to_msg()
            twist_stamped.header.frame_id = ''
            twist_stamped.twist.linear.x = self.target_linear_velocity

            twist_stamped.twist.angular.x = 0.0
            twist_stamped.twist.angular.y = 0.0
            twist_stamped.twist.linear.y = 0.0
            twist_stamped.twist.linear.z = 0.0
            twist_stamped.twist.angular.z = self.target_angular_velocity

            self.pub.publish(twist_stamped)
    def __del__(self):
        self.stop()

def get_linear_limit_velocity():
    if TURTLEBOT3_MODEL == 'burger':
        return  BURGER_MAX_LIN_VEL
    else:
        return  WAFFLE_MAX_LIN_VEL


def get_angular_limit_velocity():
    if TURTLEBOT3_MODEL == 'burger':
        return BURGER_MAX_ANG_VEL
    else:
        return WAFFLE_MAX_ANG_VEL


def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear velocity {0}\t angular velocity {1} '.format(
        target_linear_velocity,
        target_angular_velocity))


def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    inference = Inference()
    inference.init("./model-v4.pt")
    image_subscriber = CompressedImageSubscriber()
    event = Event()
    def show_img():
        turtle_bot_controller = TurtleBotController()
        turtle_bot_controller.drive_straight()
        while(not event.is_set()):
            if(image_subscriber.buffer):
                np_arr = np.frombuffer(image_subscriber.buffer, np.uint8)
                image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                image = np.flip(image, 0)
                result = inference.inference(image)
                image = result.plot()
                if(result.boxes):
                    best_cls = result.names[max(result.boxes.numpy(), key=lambda x: x.conf).cls[0]]
                    if(best_cls == 'Right'):
                        turtle_bot_controller.turn_right()
                    elif(best_cls == 'Left'):
                        turtle_bot_controller.turn_left()
                else:
                    turtle_bot_controller.drive_straight()
            
                if image is not None:
                    cv2.imshow("Compressed Image", image)
                    cv2.waitKey(1)
                else:
                    print("Image decode failed")        

    thread = Thread(target= show_img)
    thread.start()
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt: pass

    event.set()
    thread.join()


if __name__ == '__main__':
    main()