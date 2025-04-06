import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool, Int32, Int64
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

        # Add publisher for command execution timestamp
        self.cmd_executed_pub = rospy.Publisher('/cmd_executed_timestamp', Int64, queue_size=10)

        # Subscribe to state command updates
        self.start_cmd_sub = rospy.Subscriber('/pp_start', Int32, self.start_motions)
        self.slow_cmd_sub = rospy.Subscriber('/pp_slow', Int32, self.slow_motions)
        self.stop_cmd_sub = rospy.Subscriber('/pp_stop', Int32, self.stop_motions)
        
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
        Sets speed to 100% and changes the state to STARTED.
        """
        # Force speed to 100% for EXECUTE state, regardless of message content
        self.current_speed = 100
        self.current_state = MotionState.STARTED
        rospy.loginfo(f"Motion STARTED with speed {self.current_speed}%")
        
        # Update parameter on the parameter server
        rospy.set_param('~speed_percentage', self.current_speed)

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
        Cancels the current goal if executing.
        """
        self.current_speed = 0
        self.current_state = MotionState.STOPPED
        rospy.loginfo("Motion STOPPED")
        
        # Update parameter on the parameter server
        rospy.set_param('~speed_percentage', self.current_speed)
        
        # Cancel any ongoing motion
        if self.is_executing:
            self.client.cancel_goal()
            rospy.loginfo("Cancelled current motion goal")
        
        # Always publish execution timestamp
        timestamp = rospy.Time.now().to_nsec()
        self.cmd_executed_pub.publish(timestamp)
        rospy.loginfo(f"Published command executed timestamp: {timestamp}")
    
    def move_to_joint_positions(self, positions, speed_percentage=None):
        """Move the arm to the specified joint positions"""
        # If we're in STOPPED state, don't move
        if self.current_state == MotionState.STOPPED:
            rospy.loginfo("Movement skipped - robot is in STOPPED state")
            return False
        
        # Set speed based on current state, ignoring the passed-in value
        if self.current_state == MotionState.STARTED:
            speed_percentage = 100  # Always use 100% for EXECUTE/STARTED state
        elif self.current_state == MotionState.SLOWED:
            speed_percentage = 50   # Always use 50% for SLOW_DOWN state
        else:
            speed_percentage = 0    # Stop for any other state
        
        rospy.loginfo(f"Setting motion speed to {speed_percentage}% based on state {self.current_state.name}")
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions

        duration = self._calculate_duration(speed_percentage)
        point.time_from_start = rospy.Duration(duration)
        
        goal.trajectory.points.append(point)
        
        try:
            self.is_executing = True
            self.client.send_goal(goal)
            self.client.wait_for_result()
            result = self.client.get_result()
            self.is_executing = False
            
            if result:
                rospy.loginfo(f"Movement completed successfully at {speed_percentage}% speed")
                return True
            else:
                rospy.logwarn("Movement did not complete successfully")
                return False
        except Exception as e:
            rospy.logerr(f"Error during movement: {e}")
            self.is_executing = False
            return False
    
    def run_motion_cycle(self):
        """Run one complete pick and place cycle"""
        if self.current_state == MotionState.STOPPED:
            rospy.loginfo("Cycle skipped - robot is in STOPPED state")
            return False

        try:
            # Move to left up (safe position)
            rospy.loginfo("Moving to left up position")
            if not self.move_to_joint_positions(self.left_up):
                return False

            # Check if we've been stopped during motion
            if self.current_state == MotionState.STOPPED:
                return False

            # Move to left pickup
            rospy.loginfo("Moving to left pickup position")
            if not self.move_to_joint_positions(self.left_pickup):
                return False
            
            # Check if we've been stopped during motion
            if self.current_state == MotionState.STOPPED:
                return False
            
            # Move back to left up (safe position with object)
            rospy.loginfo("Moving up with object")
            if not self.move_to_joint_positions(self.left_up):
                return False
            
            # Check if we've been stopped during motion
            if self.current_state == MotionState.STOPPED:
                return False
            
            # Move to right up (above drop location)
            rospy.loginfo("Moving to right up position")
            if not self.move_to_joint_positions(self.right_up):
                return False
            
            # Check if we've been stopped during motion
            if self.current_state == MotionState.STOPPED:
                return False
            
            # Move to right down (drop location)
            rospy.loginfo("Moving to right drop position")
            if not self.move_to_joint_positions(self.right_down):
                return False
            
            # Check if we've been stopped during motion
            if self.current_state == MotionState.STOPPED:
                return False
            
            # Move back to right up (safe position)
            rospy.loginfo("Moving back up after dropping")
            if not self.move_to_joint_positions(self.right_up):
                return False
            
            return True
        except Exception as e:
            rospy.logerr(f"Error in motion cycle: {e}")
            return False

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