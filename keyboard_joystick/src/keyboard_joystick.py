#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

import time
import sys
from sfml import sf
import numpy as np
import signal

class KeyboardJoystick(object):

    # AUTHOR: Sebastian Lopez-Cot
    # PURPOSE: This module simply opens a window listening for key events to drive the racecar.
    # It overrides the /teleop topic to do so. Commands are left, right, front, and back arrow.
    # This allows for offline testing in gazebo without a physical joystick (for occasions where
    # the joystick was left in the lab or when someone else is in posession of the joystick).
    #
    # To use this module, simply rosrun it and it will start posting messages to the car.
    # It will broadcast the last state of the steering_angle and speed given to it.
    # Hitting forward or back will transition the speed from -CMD_VEL to 0 to +CMD_VEL.
    # Hitting left or right will transition the steering angle from -CMD_ANGLE to 0 to +CMD_ANGLE.
    # Hitting the right-ctrl key will toggle "high-speed" mode
    # Hitting the right-shift key will make the car come to a complete stop and reset its steering heading.
    #
    # Note, window must be focused key actions to work

    def __init__(self):

        self.pub = rospy.Publisher(
            '/vesc/low_level/ackermann_cmd_mux/input/teleop',
            AckermannDriveStamped,
            queue_size=1)

        # Setup a render window for the map.
        self.CLOSE_WINDOW_EVENT = 0
        self.RESIZE_WINDOW_EVENT = 1
        self.KEYDOWN_WINDOW_EVENT = 5

        self.WINDOW_WIDTH = 500
        self.WINDOW_HEIGHT = 500

        self.KEY_LEFT = 71
        self.KEY_RIGHT = 72
        self.KEY_UP = 73
        self.KEY_DOWN = 74
        self.KEY_ESCAPE = 36
        self.KEY_RIGHT_CTRL = 41
        self.KEY_RIGHT_SHIFT = 42

        # drawables (point count of 3 is a triangle)
        self.left_arrow_shape = sf.CircleShape(radius=80, point_count=3)
        self.left_arrow_shape.fill_color = sf.Color.GREEN
        self.left_arrow_shape.outline_color = sf.Color.BLACK
        self.left_arrow_shape.outline_thickness = 3
        self.right_arrow_shape = sf.CircleShape(radius=80, point_count=3)
        self.right_arrow_shape.fill_color = sf.Color.GREEN
        self.right_arrow_shape.outline_color = sf.Color.BLACK
        self.right_arrow_shape.outline_thickness = 3
        self.up_arrow_shape = sf.CircleShape(radius=80, point_count=3)
        self.up_arrow_shape.fill_color = sf.Color.RED
        self.up_arrow_shape.outline_color = sf.Color.BLACK
        self.up_arrow_shape.outline_thickness = 3
        self.down_arrow_shape = sf.CircleShape(radius=80, point_count=3)
        self.down_arrow_shape.fill_color = sf.Color.RED
        self.down_arrow_shape.outline_color = sf.Color.BLACK
        self.down_arrow_shape.outline_thickness = 3
        self.stop_circle = sf.CircleShape(radius=80)
        self.stop_circle.fill_color = sf.Color.BLUE
        self.stop_circle.outline_color = sf.Color.BLACK
        self.stop_circle.outline_thickness = 3
        self.fast_circle = sf.CircleShape(radius=50)
        self.fast_circle.fill_color = sf.Color.RED
        self.fast_circle.outline_color = sf.Color.BLACK
        self.fast_circle.outline_thickness = 3


        self.turning_left = False
        self.turning_right = False
        self.forward = False
        self.backward = False
        self.stopped = True
        self.fast = False

        centerX = self.WINDOW_WIDTH / 2
        centerY = self.WINDOW_HEIGHT / 2

        ORIGIN_OFFSET_X = -80
        ORIGIN_OFFSET_Y = -70
        KEY_OFFSET = 150
        self.left_arrow_shape.position = sf.Vector2(ORIGIN_OFFSET_X + centerX - KEY_OFFSET, centerY + ORIGIN_OFFSET_Y)
        self.right_arrow_shape.position = sf.Vector2(ORIGIN_OFFSET_X + centerX + KEY_OFFSET, centerY + ORIGIN_OFFSET_Y)
        self.up_arrow_shape.position = sf.Vector2(ORIGIN_OFFSET_X + centerX, centerY - KEY_OFFSET + ORIGIN_OFFSET_Y)
        self.down_arrow_shape.position = sf.Vector2(ORIGIN_OFFSET_X + centerX, centerY + KEY_OFFSET + ORIGIN_OFFSET_Y)
        self.stop_circle.position = sf.Vector2(ORIGIN_OFFSET_X + centerX, centerY + ORIGIN_OFFSET_Y)
        self.fast_circle.position = sf.Vector2(50,50)

        self.window = sf.RenderWindow(
            sf.VideoMode(self.WINDOW_WIDTH, self.WINDOW_HEIGHT), "Keyboard Joystick")

        self.velocity = 0.0
        self.steering_angle = 0.0
        self.CMD_VEL = 1.0
        self.CMD_ANGLE = np.pi / 12.0

        self.last_vel_cmd = time.time()
        self.last_steering_cmd = time.time()

        self.time_before_idle = 0.25  # seconds

    def update(self, event=None):
        for event in self.window.events:
            # window closed or escape key pressed: exit
            if event.type == self.CLOSE_WINDOW_EVENT:
                self.window.close()

            if event.type == self.RESIZE_WINDOW_EVENT:
                # self.resize()
                pass

            if event.type == self.KEYDOWN_WINDOW_EVENT:
                if event.code == self.KEY_LEFT:
                    self.steering_angle = min(
                        self.steering_angle + self.CMD_ANGLE, self.CMD_ANGLE)
                    self.last_steering_cmd = time.time()
                elif event.code == self.KEY_RIGHT:
                    self.steering_angle = max(
                        self.steering_angle - self.CMD_ANGLE, -1.0 * self.CMD_ANGLE)
                    self.last_steering_cmd = time.time()
                elif event.code == self.KEY_UP:
                    self.velocity = min(
                        self.velocity + self.CMD_VEL, self.CMD_VEL)
                    self.last_vel_cmd = time.time()
                elif event.code == self.KEY_DOWN:
                    self.velocity = max(
                        self.velocity - self.CMD_VEL, -1.0 * self.CMD_VEL)
                    self.last_vel_cmd = time.time()
                elif event.code == self.KEY_ESCAPE:
                    self.window.close()
                elif event.code == self.KEY_RIGHT_CTRL:
                    self.fast = not self.fast
                elif event.code == self.KEY_RIGHT_SHIFT:
                    self.velocity = 0
                    self.steering_angle = 0

                self.forward = (self.velocity > 0)
                self.backward = (self.velocity < 0)
                self.turning_left = (self.steering_angle > 0)
                self.turning_right = (self.steering_angle < 0)
                self.stopped = (not self.forward and not self.backward)

        self.window.clear(sf.Color.WHITE)

        if self.turning_left:
            self.window.draw(self.left_arrow_shape)
        if self.turning_right:
            self.window.draw(self.right_arrow_shape)
        if self.forward:
            self.window.draw(self.up_arrow_shape)
        if self.backward:
            self.window.draw(self.down_arrow_shape)
        if self.stopped:
            self.window.draw(self.stop_circle)
        if self.fast:
            if int(time.time()) % 2 == 0:
                self.fast_circle.fill_color = sf.Color.RED
            else:
                self.fast_circle.fill_color = sf.Color.YELLOW
            self.window.draw(self.fast_circle)

        self.window.display()

        msg = AckermannDriveStamped()
        fast_factor = 2.5 if self.fast else 1.0
        msg.drive.steering_angle = self.steering_angle
        msg.drive.speed = self.velocity * fast_factor

        self.pub.publish(msg)

        print "ANGLE: ", msg.drive.steering_angle, ", VEL: ", msg.drive.speed, "\tTIME: ", time.time()


if __name__ == "__main__":
    rospy.init_node('Keyboard_Joystick')

    joy = KeyboardJoystick()

    # for a hacked ctrl+c exit
    def signal_handler(signal, frame):
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    while joy.window.is_open:
        joy.update()

        time.sleep(1.0 / 10.0)
