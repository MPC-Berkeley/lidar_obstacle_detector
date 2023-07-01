#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray
from std_msgs.msg import Bool
from geometry_msgs.msg import PolygonStamped, Polygon, Point32
from visualization_msgs.msg import Marker

from dynamic_reconfigure.server import Server
from lidar_obstacle_detector.cfg import safety_checker_Config

class RectangleSafetyChecker(object):
    """
    Check whether there are obstacles in a rectangle region in front of the vehicle
    """
    def __init__(self):
        """
        Init node
        """
        rospy.init_node('safety_checker', anonymous=False)

        rate = rospy.get_param('cmd_rate', 10)
        self.rate = rospy.Rate(rate) # 36hz

        server = Server(safety_checker_Config, self.dynamic_reconfig_cb)
    
        # X-Y are based on LiDAR coordinate system. Y is pointing towards the front of the vehicle, while X is pointing towards the cable
        self.x_max = 2
        self.x_min = -2
        self.y_max = 7
        self.y_min = 0.2
        self.size_thres = 0.1
        self.base_frame = rospy.get_param('~base_frame', 'os_lidar')

        self.boundingbox_array = []

        self.bbox_sub = rospy.Subscriber("/obstacle_detector/jsk_bboxes", BoundingBoxArray, self.bbox_cb, queue_size=10)

        self.marker_pub = rospy.Publisher("/obstacle_detector/marker", Marker, queue_size=10)
        self.region_pub = rospy.Publisher("/obstacle_detector/detect_region", PolygonStamped, queue_size=10)
        self.safety_pub = rospy.Publisher("/obstacle_detector/safety_flag", Bool, queue_size=10)

    def dynamic_reconfig_cb(self, config, level):
        """
        callback for dynamic reconfigure
        """
        self.x_max = config["safety_region_xmax"]
        self.x_min = config["safety_region_xmin"]

        self.y_max = config["safety_region_ymax"]
        self.y_min = config["safety_region_ymin"]

        self.size_thres = config["obstalce_size_thres"]

        return config

    def bbox_cb(self, data):
        """
        callback function for bounding box array
        """
        self.boundingbox_array = data.boxes
        # print("================")
        # print(data.boxes)

    def spin(self):
        """
        main loop of the node
        """
        while not rospy.is_shutdown():
            
            is_safe = True
            for bbox in self.boundingbox_array:
                if self.x_min <= bbox.pose.position.x <= self.x_max and self.y_min <= bbox.pose.position.y <= self.y_max:
                    if bbox.dimensions.x >= self.size_thres or bbox.dimensions.y >= self.size_thres or bbox.dimensions.z >= self.size_thres:
                        is_safe = False
                        break

            # rospy.loginfo("Safety = %s" % is_safe)

            self.region = [
                Point32(self.x_min, self.y_min, 0),
                Point32(self.x_min, self.y_max, 0),
                Point32(self.x_max, self.y_max, 0),
                Point32(self.x_max, self.y_min, 0)
            ]

            marker_msg = Marker()
            marker_msg.header.frame_id = self.base_frame
            marker_msg.type = marker_msg.ARROW
            marker_msg.action = marker_msg.ADD
            marker_msg.scale.x = 0.8
            marker_msg.scale.y = 0.2
            marker_msg.scale.z = 0.2
            marker_msg.pose.orientation.z = 0.7071068
            marker_msg.pose.orientation.w = 0.7071068
            marker_msg.color.a = 1.0
            if is_safe:
                marker_msg.color.r = 0.0
                marker_msg.color.g = 1.0
                marker_msg.color.b = 0.0
            else:
                marker_msg.color.r = 1.0
                marker_msg.color.g = 0.0
                marker_msg.color.b = 0.0

            polygon_msg = PolygonStamped()
            polygon_msg.header.stamp = rospy.Time.now()
            polygon_msg.header.frame_id = self.base_frame
            polygon_msg.polygon = Polygon(self.region)

            safety_msg = Bool()
            safety_msg.data = is_safe
            # self.safety_pub.publish(safety_msg)
            self.region_pub.publish(polygon_msg)
            self.marker_pub.publish(marker_msg)

            self.rate.sleep()

def main():
    """
    main function
    """
    node = RectangleSafetyChecker()
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
