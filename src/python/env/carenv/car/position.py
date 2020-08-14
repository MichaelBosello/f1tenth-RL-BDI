import rospy
from nav_msgs.msg import Odometry

import argparse

try:
    from geometry_msgs.msg import PoseStamped
except ImportError:
    pass

try:
    from carenv.car.sensors import Sensors
except ImportError:
    from sensors import Sensors

try:
    from carenv.car.car_control import Drive
except ImportError:
    from car_control import Drive


class Position():
    def __init__(self, control, is_simulator=False):
        self.is_simulator = is_simulator
        self.control = control
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
        if c.x > 7.5 and c.x < 8.25 and c.y > -0.7 and c.y < 0.7:
            return "PRE_A"
        if c.x < 0.3 and c.y > -0.7 and c.y < 0.7:
            return "START"
        return ""

    def position_index(self, position):
        if position == "A":
            return 0
        if position == "B":
            return 1
        if position == "C":
            return 2
        if position == "END1":
            return 3
        if position == "END2":
            return 4
        if position == "PRE_A":
            return 5
        if position == "START":
            return 6
        return -1

    def reset_to_pos(self, position):
        pose = PoseStamped()
        if position == "A":
            pose.pose.position.x = 8.625
            pose.pose.position.y = 0
            pose.pose.position.z = 0
        if position == "B":
            pose.pose.position.x = 10.175
            pose.pose.position.y = 1.755
            pose.pose.position.z = 0
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0.70710808
            pose.pose.orientation.w = 0.70710548
        if position == "C":
            pose.pose.position.x = 11.755
            pose.pose.position.y = 0
            pose.pose.position.z = 0
        self.control.reset_simulator(pose)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulator", action='store_true', help="to set the use of the simulator")
    args = parser.parse_args()

    rospy.init_node('position_test')
    sensors = Sensors(args.simulator)
    drive = Drive(sensors, args.simulator)
    position = Position(drive, args.simulator)
    print("use '2D pose estimate' button to move the car")
    while True:
        print("enter key to show position - enter q to quit")
        cmd = input()
        if cmd == "r":
            position.reset_to_last_pos()
        if cmd == "q":
            exit()

        print("###### position coordinates, position map point ######")
        print(position.car_position_coordinates())
        print(position.car_position())
        print("##################")

