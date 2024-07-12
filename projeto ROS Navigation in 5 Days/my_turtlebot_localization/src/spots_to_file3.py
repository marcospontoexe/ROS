#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from my_turtlebot_localization.srv import MyServiceMessage, MyServiceMessageResponse

class SpotRecorder:
    def __init__(self):
        self.spots = []
        self.current_position = None
        self.current_orientation = None
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.service = rospy.Service('/save_spot', MyServiceMessage, self.handle_save_spot)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.current_position = f"x:{position.x}, y:{position.y}, z:{position.z}"
        self.current_orientation = f"x:{orientation.x}, y:{orientation.y}, z:{orientation.z}, w:{orientation.w}"

    def handle_save_spot(self, req):
        if req.label == "end":
            return self.save_to_file()
        elif self.current_position and self.current_orientation:
            return self.record_spot(req.label)
        else:
            return MyServiceMessageResponse(navigation_successfull=False, message="No odometry data available")

    def record_spot(self, label):
        self.spots.append({'label': label, 'position': self.current_position, 'orientation': self.current_orientation})
        return MyServiceMessageResponse(navigation_successfull=True, message="Spot recorded successfully")

    def save_to_file(self):
        try:
            with open('spots.txt', 'wt+') as file: 
                for spot in self.spots:
                    file.write(f"Label: {spot['label']}, Position: {spot['position']}, Orientation: {spot['orientation']}\n")
            return MyServiceMessageResponse(navigation_successfull=True, message="File saved successfully")
        except Exception as e:
            return MyServiceMessageResponse(navigation_successfull=False, message=f"Failed to save file: {str(e)}")

if __name__ == "__main__":
    rospy.init_node('spot_recorder')
    spot_recorder = SpotRecorder()
    rospy.spin()
