#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty
from std_msgs.msg import Bool, Int32
import time

from enum import Enum

class MotionState(Enum):
    STARTED = 0
    SLOWED = 1
    STOPPED = 2

class PickAndPlace:

    def __init__(self):
        rospy.init_node('pick_and_place_demo')

        self.current_state = MotionState.STARTED
        self.current_speed = 100
        self.is_executing = False  # Flag to track if motion is in progress

        # Subscribe to state command updates
        self.start_cmd_sub = rospy.Subscriber('/pp_start', Int32, self.start_motions)
        self.slow_cmd_sub = rospy.Subscriber('/pp_slow', Int32, self.slow_motions)
        self.stop_cmd_sub = rospy.Subscriber('/pp_stop', Int32, self.stop_motions)
        
        # Set up service clients for robot dashboard
        self.pause_service = None
        try:
            rospy.loginfo("Waiting for dashboard pause service...")
            rospy.wait_for_service('/ur/ur_hardware_interface/dashboard/pause', timeout=5.0)
            self.pause_service = rospy.ServiceProxy('/ur/ur_hardware_interface/dashboard/pause', Empty)
            rospy.loginfo("Connected to dashboard pause service")
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logwarn(f"Failed to connect to pause service: {e}")

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
        self.left_pickup = [-1.4435839653015137, -1.90, -0.5025284926043909, -1.2548909944346924, 1.5955007076263428, -0.07734996477235967]
        self.left_up = [-1.4435958862304688, -1.2566581529429932, -0.5024922529803675, -1.2548909944346924, 1.5954649448394775, -0.07731372514833623]
        self.right_up = [-1.4435839653015137, -1.2566341918757935, 0.5026633739471436, -1.2548790735057374, 1.5954649448394775, -0.07733804384340459]
        self.right_down = [-1.4435715675354004, -1.90, 0.5026873350143433, -1.2549031537822266, 1.5954411029815674, -0.07731372514833623]
    
    def start_motions(self, msg):
        """
        Callback for start motion commands.
        Sets speed to the received value (or 100% if the message doesn't contain a valid speed).
        Changes the state to STARTED.
        """
        if msg is not None and hasattr(msg, 'data'):
            speed = msg.data
            if 0 <= speed <= 100:
                self.current_speed = speed
            else:
                self.current_speed = 100
        else:
            self.current_speed = 100
        
        was_stopped = (self.current_state == MotionState.STOPPED)
        self.current_state = MotionState.STARTED
        rospy.loginfo(f"Motion STARTED with speed {self.current_speed}%")
        
        # Update parameter on the parameter server
        rospy.set_param('~speed_percentage', self.current_speed)
        
        try:
            # If we were stopped, we need to resume the program
            if was_stopped:
                # First try to play (resume) the program
                rospy.wait_for_service('/ur/ur_hardware_interface/dashboard/play', timeout=2.0)
                play = rospy.ServiceProxy('/ur/ur_hardware_interface/dashboard/play', Empty)
                play()
                rospy.loginfo("Resumed program after stop")
            else:
                # If not previously stopped, load and start as normal
                # Load the remote control program if it's not already running
                rospy.wait_for_service('/ur/ur_hardware_interface/dashboard/load_program', timeout=2.0)
                load_program = rospy.ServiceProxy('/ur/ur_hardware_interface/dashboard/load_program', rospy.ServiceRequest)
                load_program("filename: 'remote_control.urp'")
                rospy.loginfo("Loaded remote_control.urp program")
                
                # Play the program
                rospy.wait_for_service('/ur/ur_hardware_interface/dashboard/play', timeout=2.0)
                play = rospy.ServiceProxy('/ur/ur_hardware_interface/dashboard/play', Empty)
                play()
                rospy.loginfo("Started remote control program")
        except Exception as e:
            rospy.logwarn(f"Error starting program: {e}")
    
    def slow_motions(self, msg):
        """
        Callback for slow motion commands.
        Sets speed to 50% and changes the state to SLOWED.
        """
        # Always set to 50% for SLOW_DOWN zone as requested
        self.current_speed = 50
        self.current_state = MotionState.SLOWED
        rospy.loginfo("Motion SLOWED to 50%")
        
        # Update parameter on the parameter server
        rospy.set_param('~speed_percentage', self.current_speed)
    
    def stop_motions(self, msg):
        """
        Callback for stop motion commands.
        Sets speed to 0% and changes the state to STOPPED.
        Uses dashboard/pause service to safely pause the robot.
        """
        self.current_speed = 0
        self.current_state = MotionState.STOPPED
        rospy.loginfo("Motion STOPPED")
        
        # Update parameter on the parameter server
        rospy.set_param('~speed_percentage', self.current_speed)
        
        # Use dashboard pause service to stop the robot
        if self.pause_service is not None:
            try:
                self.pause_service()
                rospy.loginfo("Robot paused via dashboard service")
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to pause robot: {e}")
                # Fallback to canceling the current goal
                if self.is_executing:
                    self.client.cancel_goal()
                    rospy.loginfo("Cancelled current motion goal as fallback")
        else:
            # If pause service is not available, cancel the current goal
            if self.is_executing:
                self.client.cancel_goal()
                rospy.loginfo("Cancelled current motion goal (pause service not available)")
    
    def move_to_joint_positions(self, positions, speed_percentage):
        """Move the arm to the specified joint positions"""
        # If we're in STOPPED state, don't move
        if self.current_state == MotionState.STOPPED:
            rospy.loginfo("Movement skipped - robot is in STOPPED state")
            return False
        
        # Apply the current speed from the state
        if self.current_state == MotionState.SLOWED:
            speed_percentage = 50  # Always use 50% for SLOW_DOWN
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions

        # Sanitize speed percentage
        if speed_percentage > 100:
            speed_percentage = 100
        elif speed_percentage < 0:
            speed_percentage = 0
        else:
            speed_percentage = int(speed_percentage)

        duration = self._calculate_duration(speed_percentage)
        point.time_from_start = rospy.Duration(duration)
        
        goal.trajectory.points.append(point)
        
        self.is_executing = True
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        self.is_executing = False
        
        if result:
            rospy.loginfo(f"Movement completed successfully at {speed_percentage}% speed")
            return True
        else:
            rospy.logerr("Movement failed")
            return False
    
    def run_motion_cycle(self):
        """Run one complete pick and place cycle"""
        if self.current_state == MotionState.STOPPED:
            rospy.loginfo("Cycle skipped - robot is in STOPPED state")
            return False

        # Get the current speed percentage from parameter server
        current_speed_percentage = rospy.get_param('~speed_percentage', self.current_speed)

        # Move to left up (safe position)
        rospy.loginfo("Moving to left up position")
        if not self.move_to_joint_positions(self.left_up, current_speed_percentage):
            return False

        # Get the current speed percentage again (it might have changed due to safety system)
        current_speed_percentage = rospy.get_param('~speed_percentage', self.current_speed)
        # Move to left pickup
        rospy.loginfo("Moving to left pickup position")
        if not self.move_to_joint_positions(self.left_pickup, current_speed_percentage):
            return False
        
        # Get the current speed percentage again
        current_speed_percentage = rospy.get_param('~speed_percentage', self.current_speed)
        # Move back to left up (safe position with object)
        rospy.loginfo("Moving up with object")
        if not self.move_to_joint_positions(self.left_up, current_speed_percentage):
            return False
        
        # Get the current speed percentage again
        current_speed_percentage = rospy.get_param('~speed_percentage', self.current_speed)
        # Move to right up (above drop location)
        rospy.loginfo("Moving to right up position")
        if not self.move_to_joint_positions(self.right_up, current_speed_percentage):
            return False
        
        # Get the current speed percentage again
        current_speed_percentage = rospy.get_param('~speed_percentage', self.current_speed)
        # Move to right down (drop location)
        rospy.loginfo("Moving to right drop position")
        if not self.move_to_joint_positions(self.right_down, current_speed_percentage):
            return False
        
        # Get the current speed percentage again
        current_speed_percentage = rospy.get_param('~speed_percentage', self.current_speed)
        # Move back to right up (safe position)
        rospy.loginfo("Moving back up after dropping")
        if not self.move_to_joint_positions(self.right_up, current_speed_percentage):
            return False
        
        return True

    def _calculate_duration(self, speed_percentage):
        """Calculate the duration based on speed percentage"""
        min_duration = 1.0  # seconds
        max_duration = 5.0  # seconds

        # If speed is 0 (STOPPED), use maximum duration
        if speed_percentage == 0:
            return max_duration

        duration = min_duration + ((max_duration - min_duration) * ((100 - speed_percentage) / 100))
        return duration

def main():
    rospy.loginfo("Starting pick and place demo")

    # Get speed percentage from parameter server with default 50%
    speed_percentage = rospy.get_param('~speed_percentage', 50)
    
    # Create pick and place object
    pick_place = PickAndPlace()
    
    # Run pick and place in a loop
    try:
        while not rospy.is_shutdown():
            if pick_place.current_state != MotionState.STOPPED:
                pick_place.run_motion_cycle()
            else:
                rospy.loginfo("Robot is STOPPED. Waiting for start command...")
            
            # Add a small delay between cycles
            rospy.sleep(1.0)
            
    except KeyboardInterrupt:
        rospy.loginfo("Demo interrupted by user")
    except Exception as e:
        rospy.logerr(f"Error in pick and place demo: {e}")
    
    rospy.loginfo("Pick and place demo finished")

if __name__ == '__main__':
    main()