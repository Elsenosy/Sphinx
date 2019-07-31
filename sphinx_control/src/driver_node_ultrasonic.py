#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import MDD10A as HBridge

# Set GPIO methods
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

_FREQUENCY = 20


def _clip(value, minimum, maximum):
    """Ensure value is between minimum and maximum."""

    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    return value

class Driver:
    def __init__(self):
        rospy.init_node('driver')

        self._last_received = rospy.get_time()
        self._timeout = rospy.get_param('~timeout', 2)
        self._rate = rospy.get_param('~rate', 10)
        self._max_speed = rospy.get_param('~max_speed', 100.0)
        self._wheel_base = rospy.get_param('~wheel_base', 0.91)

        self._left_speed_percent = 0
        self._right_speed_percent = 0
        self.range = 100

        # Setup subscriber for velocity twist message
        rospy.Subscriber(
            '/sphinx/sphinx_skid_steer_controller/cmd_vel', Twist, self.velocity_received_callback)

        # Subscribe for sensor msgs
        rospy.Subscriber('/sphinx/ultrasonic_range', Range, self.ultrasonicCallback)

    def velocity_received_callback(self, message):
        """Handle new velocity command message."""

        self._last_received = rospy.get_time()

        # Extract linear and angular velocities from the message
        linear = message.linear.x
        angular = message.angular.z

        # Calculate wheel speeds in m/s
        left_speed = linear - angular*self._wheel_base/2
        right_speed = linear + angular*self._wheel_base/2

        # Ideally we'd now use the desired wheel speeds along
        # with data from wheel speed sensors to come up with the
        # power we need to apply to the wheels, but we don't have
        # wheel speed sensors. Instead, we'll simply convert m/s
        # into percent of maximum wheel speed, which gives us a
        # duty cycle that we can apply to each motor.
        self._left_speed_percent = (100 * left_speed/self._max_speed)
        self._right_speed_percent = (100 * right_speed/self._max_speed)

    def ultrasonicCallback(self, message):
        """Handle ultrasonic range messages."""
        self.range = message.range

    def run(self):
        """The control loop of the driver."""

        rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():
            # If we haven't received new commands for a while, we
            # may have lost contact with the commander-- stop
            # moving
            delay = rospy.get_time() - self._last_received
            if delay < self._timeout:
                if self.range < 20.0:
                    HBridge.setMotorLeft(0)
                    HBridge.setMotorRight(0)
                else:
                    HBridge.setMotorLeft(self._left_speed_percent)
                    HBridge.setMotorRight(self._right_speed_percent)
                    rospy.loginfo("Motor left: %s" % self._left_speed_percent)
                    rospy.loginfo("Motor right: %s" % self._right_speed_percent)
            else:
                HBridge.setMotorLeft(0)
                HBridge.setMotorRight(0)
                
            rate.sleep()

def main():
        driver = Driver()

        # Run driver. This will block
        driver.run()

if __name__ == '__main__':
    main()        
