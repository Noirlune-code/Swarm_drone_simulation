

import rospy
from geometry_msgs.msg import PoseStamped
import tf
import math


FORMATION_OFFSETS = {
    "drone1": [0.0, 0.0, 0.0],      
    "drone2": [-1.5, -1.5, 0.0],    
    "drone3": [1.5, -1.5, 0.0],    }

class FormationController:
    def __init__(self):
        rospy.init_node('formation_controller')

        self.leader_pose = None
        rospy.Subscriber("/drone1/pose", PoseStamped, self.leader_pose_callback)

        self.followers = {}
        for drone_id in FORMATION_OFFSETS:
            if drone_id != "drone1":
                pub = rospy.Publisher(f"/{drone_id}/goal_pose", PoseStamped, queue_size=10)
                self.followers[drone_id] = pub

        rospy.Timer(rospy.Duration(0.1), self.update_formation)
        rospy.spin()

    def leader_pose_callback(self, msg):
        self.leader_pose = msg

    def update_formation(self, event):
        if not self.leader_pose:
            return

        leader_x = self.leader_pose.pose.position.x
        leader_y = self.leader_pose.pose.position.y
        leader_z = self.leader_pose.pose.position.z

        for drone_id, offset in FORMATION_OFFSETS.items():
            if drone_id == "drone1":
                continue  # Skip leader

            dx, dy, dz = offset
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "world"

            goal.pose.position.x = leader_x + dx
            goal.pose.position.y = leader_y + dy
            goal.pose.position.z = leader_z + dz

            goal.pose.orientation = self.leader_pose.pose.orientation
            self.followers[drone_id].publish(goal)

if __name__ == '__main__':
    try:
        FormationController()
    except rospy.ROSInterruptException:
        pass
