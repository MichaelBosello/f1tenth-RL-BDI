import rospy
from nav_msgs.msg import Odometry

import argparse


class Position():
    def __init__(self, is_simulator=False):
        self.is_simulator = is_simulator
        self.odometry = None
        if not is_simulator:
            odom_topic = "/vesc/odom"
        else:
            odom_topic = "/odom"
        self.odom_subscriber = rospy.Subscriber(odom_topic, Odometry, self.odometry_callback)

    def odometry_callback(self, odometry):
        self.odometry = odometry

    def car_position_coordinates(self):
        return self.odometry.pose.pose.position

    def car_position(self):
        c = self.car_position_coordinates()
        if c.x > 8.25 and c.x < 9 and c.y > -0.7 and c.y < 0.7:
            return "A"
        if c.x > 9.55 and c.x < 10.8 and c.y > 1.38 and c.y < 2.13:
            return "B"
        if c.x > 11.38 and c.x < 12.13 and c.y > -0.7 and c.y < 0.7:
            return "C"
        if c.x > 6.68 and c.x < 8 and c.y > 9.45 and c.y < 11:
            return "END1"
        if c.x > 17.2 and c.x < 18.5 and c.y > 13.6 and c.y < 14.9:
            return "END2"
        return None


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulator", action='store_true', help="to set the use of the simulator")
    args = parser.parse_args()

    rospy.init_node('position_test')
    position = Position(args.simulator)
    print("use '2D pose estimate' button to move the car")
    while True:
        print("enter key to show position - enter q to quit")
        cmd = input()
        if cmd == "q":
            exit()

        print("###### position coordinates, position map point ######")
        print(position.car_position_coordinates())
        print(position.car_position())
        print("##################")

