#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty
import time

class PickAndPlace:

    def __init__(self):
        rospy.init_node('pick_and_place_demo')
        
        # Connect to the trajectory controller action server
        self.client = actionlib.SimpleActionClient(
            '/ur/ur_arm_scaled_pos_joint_traj_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        rospy.loginfo("Waiting for joint trajectory action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to joint trajectory action server")
        
        # Joint names (order is important)
        self.joint_names = [
            'ur_arm_elbow_joint',
            'ur_arm_shoulder_lift_joint',
            'ur_arm_shoulder_pan_joint',
            'ur_arm_wrist_1_joint',
            'ur_arm_wrist_2_joint',
            'ur_arm_wrist_3_joint'
        ]
        
        # Joint positions from your data
        self.left_pickup = [-1.4435839653015137, -1.90 , -0.5025284926043909, -1.2548909944346924, 1.5955007076263428, -0.07734996477235967]
        self.left_up = [-1.4435958862304688, -1.2566581529429932, -0.5024922529803675, -1.2548909944346924, 1.5954649448394775, -0.07731372514833623]
        self.right_up = [-1.4435839653015137, -1.2566341918757935, 0.5026633739471436, -1.2548790735057374, 1.5954649448394775, -0.07733804384340459]
        self.right_down = [-1.4435715675354004, -1.90, 0.5026873350143433, -1.2549031537822266, 1.5954411029815674, -0.07731372514833623]
        
    
    def calculate_duration(self, speed_percentage):
        """Calculate the duration based on speed percentage"""
        min_duration = 1.0  # seconds
        max_duration = 5.0  # seconds

        duration = min_duration + ((max_duration - min_duration) * ((100 - speed_percentage) / 100))

        return duration

    
    def move_to_joint_positions(self, positions, speed_percentage):
        """Move the arm to the specified joint positions"""
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions

        if (speed_percentage > 100):
            speed_percentage = 100

        elif (speed_percentage < 0):
            speed_percentage = 0

        else:
            int(speed_percentage)

        duration = self.calculate_duration(speed_percentage)

        point.time_from_start = rospy.Duration(duration)
        
        goal.trajectory.points.append(point)
        
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        
        if result:
            rospy.loginfo("Movement completed successfully")
        else:
            rospy.logerr("Movement failed")
    

    def run_pick_and_place_cycle(self, current_speed_percentage):
        """Run one complete pick and place cycle"""


def main():
    rospy.loginfo("Starting pick and place demo")

    # Get speed pecentage from parameter server
    speed_percentage = rospy.get_param('~speed_percentage', 50)
    
    # Create pick and place object
    pick_place = PickAndPlace()
    
    # Run pick and place in a loop
    try:
        cycle_count = 0
        while not rospy.is_shutdown():


            cycle_count += 1
            rospy.loginfo("Starting pick and place cycle #{}".format(cycle_count))

            # Get the current speed percentage from parameter server
            current_speed_percentage = rospy.get_param('~speed_percentage', speed_percentage)

            # Move to left up (safe position)
            rospy.loginfo("Moving to left up position")
            pick_place.move_to_joint_positions(pick_place.left_up, current_speed_percentage)

            

            # Get the current speed percentage from parameter server
            current_speed_percentage = rospy.get_param('~speed_percentage', speed_percentage)
            # Move to left pickup
            rospy.loginfo("Moving to left pickup position")
            pick_place.move_to_joint_positions(pick_place.left_pickup, current_speed_percentage)
            
            

            # Get the current speed percentage from parameter server
            current_speed_percentage = rospy.get_param('~speed_percentage', speed_percentage)
            # Move back to left up (safe position with object)
            rospy.loginfo("Moving up with object")
            pick_place.move_to_joint_positions(pick_place.left_up, current_speed_percentage)
            

            # Get the current speed percentage from parameter server
            current_speed_percentage = rospy.get_param('~speed_percentage', speed_percentage)
            # Move to right up (above drop location)
            rospy.loginfo("Moving to right up position")
            pick_place.move_to_joint_positions(pick_place.right_up, current_speed_percentage)
            

            # Get the current speed percentage from parameter server
            current_speed_percentage = rospy.get_param('~speed_percentage', speed_percentage)
            # Move to right down (drop location)
            rospy.loginfo("Moving to right drop position")
            pick_place.move_to_joint_positions(pick_place.right_down, current_speed_percentage)
            
            

            # Get the current speed percentage from parameter server
            current_speed_percentage = rospy.get_param('~speed_percentage', speed_percentage)
            # Move back to right up (safe position)
            rospy.loginfo("Moving back up after dropping")
            pick_place.move_to_joint_positions(pick_place.right_up, current_speed_percentage)
            
            rospy.loginfo("Pick and place cycle completed")
            rospy.loginfo("Completed cycle #{}".format(cycle_count))
            
            # Add a small delay between cycles
            rospy.sleep(1.0)
            
    except KeyboardInterrupt:
        rospy.loginfo("Demo interrupted by user")
    except Exception as e:
        rospy.logerr("Error in pick and place demo: {}".format(e))
    
    rospy.loginfo("Pick and place demo finished")

if __name__ == '__main__':
    main()