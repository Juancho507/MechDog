#!/usr/bin/env python
"""
Navigation Manager Node for MechDog
Coordinates all navigation components and manages state machine
100% portable - no simulation dependencies
"""

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from nav_msgs.msg import Path
import threading


class NavigationManager:
    """
    Navigation state machine states:
    - IDLE: Waiting for goal
    - PLANNING: Computing global path
    - MOVING: Executing path
    - RECOVERY: Stuck, trying recovery behaviors
    - PAUSED: Paused by user
    - ERROR: Error state
    - GOAL_REACHED: Successfully reached goal
    """
    
    # States
    STATE_IDLE = "idle"
    STATE_PLANNING = "planning"
    STATE_MOVING = "moving"
    STATE_RECOVERY = "recovery"
    STATE_PAUSED = "paused"
    STATE_ERROR = "error"
    STATE_GOAL_REACHED = "goal_reached"
    
    def __init__(self):
        rospy.init_node('navigation_manager', anonymous=False)
        
        # Load parameters
        self.load_parameters()
        
        # State
        self.current_state = self.STATE_IDLE
        self.current_goal = None
        self.global_path = None
        self.emergency_stop = False
        self.state_lock = threading.Lock()
        
        # Metrics
        self.start_time = None
        self.distance_traveled = 0.0
        self.recovery_count = 0
        
        # Publishers
        self.status_pub = rospy.Publisher(
            self.param_status_topic, String, queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher(
            self.param_goal_topic, PoseStamped, queue_size=1)
        
        # Subscribers
        self.goal_sub = rospy.Subscriber(
            self.param_goal_topic, PoseStamped, self.goal_callback)
        self.path_sub = rospy.Subscriber(
            self.param_global_plan_topic, Path, self.path_callback)
        self.emergency_sub = rospy.Subscriber(
            self.param_emergency_topic, Bool, self.emergency_callback)
        self.safety_status_sub = rospy.Subscriber(
            self.param_safety_status_topic, String, self.safety_status_callback)
        
        # Services (for future expansion)
        # rospy.Service('~pause', Trigger, self.pause_callback)
        # rospy.Service('~resume', Trigger, self.resume_callback)
        # rospy.Service('~cancel_goal', Trigger, self.cancel_goal_callback)
        
        # State machine timer
        self.state_timer = rospy.Timer(
            rospy.Duration(0.1),  # 10 Hz
            self.state_machine_callback)
        
        # Diagnostics timer
        self.diag_timer = rospy.Timer(
            rospy.Duration(1.0 / self.param_diagnostics_rate),
            self.diagnostics_callback)
        
        rospy.loginfo("Navigation Manager initialized in state: %s", self.current_state)
        self.publish_status()
        
    def load_parameters(self):
        """Load parameters from parameter server"""
        # Components
        self.param_enable_global_planner = rospy.get_param(
            'navigation/components/global_planner', True)
        self.param_enable_local_planner = rospy.get_param(
            'navigation/components/local_planner', True)
        self.param_enable_safe_learning = rospy.get_param(
            'navigation/components/safe_learning', True)
        
        # Goal handling
        self.param_goal_tolerance = rospy.get_param('navigation/goal_handling/goal_tolerance', 0.1)
        self.param_goal_timeout = rospy.get_param('navigation/goal_handling/timeout', 60.0)
        
        # Recovery
        self.param_recovery_enabled = rospy.get_param('navigation/recovery/enabled', True)
        self.param_max_recovery_attempts = rospy.get_param(
            'navigation/recovery/max_recovery_attempts', 5)
        
        # Monitoring
        self.param_publish_diagnostics = rospy.get_param(
            'navigation/monitoring/publish_diagnostics', True)
        self.param_diagnostics_rate = rospy.get_param('navigation/monitoring/diagnostics_rate', 1.0)
        
        # Topics
        self.param_goal_topic = rospy.get_param('navigation/topics/goal', '/mechdog/goal')
        self.param_global_plan_topic = rospy.get_param(
            'navigation/topics/global_plan', '/mechdog/global_plan')
        self.param_status_topic = rospy.get_param('navigation/topics/status', '/mechdog/navigation_status')
        self.param_emergency_topic = rospy.get_param(
            'safe_learning/topics/emergency_stop_output', '/mechdog/emergency_stop')
        self.param_safety_status_topic = rospy.get_param(
            'safe_learning/topics/safety_status_output', '/mechdog/safety_status')
        
    def goal_callback(self, msg):
        """Receive new navigation goal"""
        with self.state_lock:
            rospy.loginfo("Received new goal: (%.2f, %.2f)", 
                         msg.pose.position.x, msg.pose.position.y)
            
            self.current_goal = msg
            self.start_time = rospy.Time.now()
            self.distance_traveled = 0.0
            self.recovery_count = 0
            
            # Transition to planning state
            self.set_state(self.STATE_PLANNING)
            
    def path_callback(self, msg):
        """Receive global path from planner"""
        with self.state_lock:
            if len(msg.poses) > 0:
                self.global_path = msg
                rospy.loginfo("Received global path with %d waypoints", len(msg.poses))
                
                # Reset timeout clock whenever we get a fresh path
                self.start_time = rospy.Time.now()
                
                # Transition to moving state if we were planning
                if self.current_state == self.STATE_PLANNING:
                    self.set_state(self.STATE_MOVING)
            else:
                rospy.logwarn("Received empty path")
                if self.current_state == self.STATE_PLANNING:
                    self.set_state(self.STATE_ERROR)
                    
    def emergency_callback(self, msg):
        """Receive emergency stop signal"""
        self.emergency_stop = msg.data
        
        if self.emergency_stop:
            rospy.logwarn("Emergency stop activated")
            with self.state_lock:
                if self.current_state == self.STATE_MOVING:
                    self.set_state(self.STATE_RECOVERY)
                    self.recovery_count += 1
                    
    def safety_status_callback(self, msg):
        """Receive safety status from safe learning node"""
        status = msg.data
        
        # React to safety status
        if status == "EMERGENCY_STOP":
            with self.state_lock:
                if self.current_state == self.STATE_MOVING:
                    rospy.logwarn("Safety system triggered emergency stop")
                    self.set_state(self.STATE_RECOVERY)
                    self.recovery_count += 1
                    
    def state_machine_callback(self, event):
        """Main state machine"""
        with self.state_lock:
            if self.current_state == self.STATE_IDLE:
                self.handle_idle_state()
            elif self.current_state == self.STATE_PLANNING:
                self.handle_planning_state()
            elif self.current_state == self.STATE_MOVING:
                self.handle_moving_state()
            elif self.current_state == self.STATE_RECOVERY:
                self.handle_recovery_state()
            elif self.current_state == self.STATE_PAUSED:
                self.handle_paused_state()
            elif self.current_state == self.STATE_ERROR:
                self.handle_error_state()
            elif self.current_state == self.STATE_GOAL_REACHED:
                self.handle_goal_reached_state()
                
    def handle_idle_state(self):
        """Handle IDLE state - waiting for goal"""
        # Nothing to do, wait for goal
        pass
        
    def handle_planning_state(self):
        """Handle PLANNING state - waiting for path"""
        if self.current_goal is None:
            self.set_state(self.STATE_IDLE)
            return
            
        # Check for timeout
        if self.start_time is not None:
            elapsed = (rospy.Time.now() - self.start_time).to_sec()
            if elapsed > 10.0:  # 10 seconds planning timeout
                rospy.logerr("Planning timeout")
                self.set_state(self.STATE_ERROR)
                
    def handle_moving_state(self):
        """Handle MOVING state - executing path"""
        if self.current_goal is None:
            self.set_state(self.STATE_IDLE)
            return
            
        # Check for goal timeout
        if self.start_time is not None:
            elapsed = (rospy.Time.now() - self.start_time).to_sec()
            if elapsed > self.param_goal_timeout:
                rospy.logerr("Goal timeout after %.1fs", elapsed)
                self.set_state(self.STATE_ERROR)
                return
                
        # Goal reached check is done by local planner
        # Here we just monitor overall progress
        
    def handle_recovery_state(self):
        """Handle RECOVERY state - trying to recover"""
        if self.recovery_count >= self.param_max_recovery_attempts:
            rospy.logerr("Maximum recovery attempts reached")
            self.set_state(self.STATE_ERROR)
            return
            
        rospy.loginfo("Recovery attempt %d/%d", 
                     self.recovery_count, self.param_max_recovery_attempts)
        
        # After a brief pause, try moving again
        rospy.sleep(1.0)
        
        if not self.emergency_stop:
            rospy.loginfo("Recovery successful, resuming")
            self.set_state(self.STATE_MOVING)
        
    def handle_paused_state(self):
        """Handle PAUSED state - paused by user"""
        # Wait for resume command
        pass
        
    def handle_error_state(self):
        """Handle ERROR state"""
        # Log error and wait for new goal
        rospy.logerr_throttle(10.0, "Navigation in ERROR state")
        
    def handle_goal_reached_state(self):
        """Handle GOAL_REACHED state"""
        if self.start_time is not None:
            elapsed = (rospy.Time.now() - self.start_time).to_sec()
            rospy.loginfo("Goal reached in %.1f seconds", elapsed)
            
        # Reset and return to idle
        self.current_goal = None
        self.global_path = None
        self.set_state(self.STATE_IDLE)
        
    def set_state(self, new_state):
        """Transition to new state"""
        if new_state != self.current_state:
            rospy.loginfo("State transition: %s -> %s", self.current_state, new_state)
            self.current_state = new_state
            self.publish_status()
            
    def publish_status(self):
        """Publish current navigation status"""
        msg = String()
        msg.data = self.current_state
        self.status_pub.publish(msg)
        
    def diagnostics_callback(self, event):
        """Publish diagnostics"""
        if not self.param_publish_diagnostics:
            return
            
        # Log current status
        if self.current_state != self.STATE_IDLE:
            info = "Navigation Status: %s" % self.current_state
            
            if self.start_time is not None:
                elapsed = (rospy.Time.now() - self.start_time).to_sec()
                info += " | Time: %.1fs" % elapsed
                
            if self.recovery_count > 0:
                info += " | Recoveries: %d" % self.recovery_count
                
            rospy.loginfo_throttle(5.0, info)
            
    def run(self):
        """Main loop"""
        rospy.spin()


if __name__ == '__main__':
    try:
        manager = NavigationManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass
