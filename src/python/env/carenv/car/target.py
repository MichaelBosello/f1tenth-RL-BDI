import rospy

try:
    from visualization_msgs.msg import Marker
    from geometry_msgs.msg import PointStamped
except Exception:
    pass

import argparse
import random

TARGETS = ["end1", "end2"]

class Target():
    def __init__(self, is_simulator=False):
        self.is_simulator = is_simulator
        self.target = None
        if self.is_simulator:
            self.marker_publisher = rospy.Publisher('/dynamic_viz', Marker, queue_size=0)
            rospy.sleep(1)
            self.marker = {}
            for i, target in enumerate(TARGETS):
                self.marker[target] = Marker()
                self.marker[target].header.frame_id = "/map"
                self.marker[target].header.stamp = rospy.Time.now()
                self.marker[target].id = i
                self.marker[target].type = Marker.SPHERE
                self.marker[target].action = Marker.ADD
                self.marker[target].pose.orientation.x = 0
                self.marker[target].pose.orientation.y = 0
                self.marker[target].pose.orientation.z = 0
                self.marker[target].pose.orientation.w = 1
                self.marker[target].scale.x = 0.5
                self.marker[target].scale.y = 0.5
                self.marker[target].scale.z = 0.5
                self.marker[target].lifetime = rospy.Duration()
                self.marker[target].color.r = 0.46
                self.marker[target].color.g = 0.53
                self.marker[target].color.b = 0.6
                self.marker[target].color.a = 1.0
            point = PointStamped()
            point.point.x = 7.5
            point.point.y = 8.4
            point.point.z = 0
            self.marker[TARGETS[0]].pose.position = point.point
            self.marker_publisher.publish(self.marker[TARGETS[0]])
            point = PointStamped()
            point.point.x = 18
            point.point.y = 15.9
            point.point.z = 0
            self.marker[TARGETS[1]].pose.position = point.point
            self.marker_publisher.publish(self.marker[TARGETS[1]])

    def new_target(self):
        self.target = TARGETS[random.randrange(len(TARGETS))]
        if self.is_simulator:
            for target in TARGETS:
                self.marker[target].color.r = 0.46
                self.marker[target].color.g = 0.53
                self.marker[target].color.b = 0.6
                self.marker[target].header.stamp = rospy.get_rostime()
                self.marker_publisher.publish(self.marker[target])
            self.marker[self.target].color.r = 0
            self.marker[self.target].color.g = 1
            self.marker[self.target].color.b = 0
            self.marker[self.target].header.stamp = rospy.get_rostime()
            self.marker_publisher.publish(self.marker[self.target])



    def current_target(self):
        return self.target


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulator", action='store_true', help="to set the use of the simulator")
    args = parser.parse_args()

    rospy.init_node('target_test')
    target = Target(args.simulator)
    while True:
        print("Write command")
        cmd = input()
        if cmd == "n":
            target.new_target()
        if cmd == "c":
            print(target.current_target())
        if cmd == "q":
            exit()

