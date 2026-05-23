#!/usr/bin/env python
"""
Global Planner Node for MechDog Navigation
Implements discrete path planning algorithms (BFS, A*)
100% portable - no simulation dependencies
"""

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from collections import deque
import heapq


class GlobalPlanner:
    def __init__(self):
        rospy.init_node('global_planner', anonymous=False)
        
        # Load parameters
        self.load_parameters()
        
        # State
        self.current_map = None
        self.current_goal = None
        self.robot_pose = None
        
        # Publishers
        self.path_pub = rospy.Publisher(
            self.param_plan_output, Path, queue_size=1)
        
        # Subscribers
        self.map_sub = rospy.Subscriber(
            self.param_map_input, OccupancyGrid, self.map_callback)
        self.goal_sub = rospy.Subscriber(
            self.param_goal_input, PoseStamped, self.goal_callback)
        
        # Timer for periodic replanning
        self.replan_timer = rospy.Timer(
            rospy.Duration(1.0 / self.param_planning_frequency),
            self.replan_callback)
        
        rospy.loginfo("Global Planner initialized - algorithm: %s", self.param_algorithm)
        
    def load_parameters(self):
        """Load parameters from parameter server"""
        # Algorithm
        self.param_algorithm = rospy.get_param('~global_planner/algorithm', 'astar')
        
        # Planning
        self.param_cell_size = rospy.get_param('~global_planner/planning/cell_size', 0.1)
        self.param_max_planning_time = rospy.get_param(
            '~global_planner/planning/max_planning_time', 5.0)
        self.param_planning_frequency = rospy.get_param(
            'navigation/planning/global_planning_frequency', 1.0)
        
        # A* parameters
        self.param_heuristic = rospy.get_param(
            '~global_planner/astar/heuristic', 'manhattan')
        self.param_heuristic_weight = rospy.get_param(
            '~global_planner/astar/heuristic_weight', 1.0)
        
        # Goal tolerance
        self.param_goal_tolerance = rospy.get_param(
            '~global_planner/goal/xy_tolerance', 0.2)
        
        # Inflation
        self.param_inflation_radius = rospy.get_param(
            '~global_planner/inflation/radius', 0.3)
        
        # Topics
        self.param_map_input = rospy.get_param(
            '~global_planner/topics/map_input', '/mechdog/map')
        self.param_goal_input = rospy.get_param(
            '~global_planner/topics/goal_input', '/mechdog/goal')
        self.param_plan_output = rospy.get_param(
            '~global_planner/topics/plan_output', '/mechdog/global_plan')
        
        # Frames
        self.param_global_frame = rospy.get_param(
            '~global_planner/frames/global_frame', 'map')
        
    def map_callback(self, msg):
        """Receive updated occupancy grid"""
        self.current_map = msg
        rospy.logdebug("Received occupancy grid: %dx%d", msg.info.width, msg.info.height)
        
    def goal_callback(self, msg):
        """Receive new goal"""
        self.current_goal = msg
        rospy.loginfo("Received new goal: (%.2f, %.2f)",
                     msg.pose.position.x, msg.pose.position.y)
        # Trigger immediate replanning
        self.plan_path()
        
    def replan_callback(self, event):
        """Periodic replanning"""
        if self.current_goal is not None and self.current_map is not None:
            self.plan_path()
            
    def plan_path(self):
        """Main planning function"""
        if self.current_map is None or self.current_goal is None:
            return
            
        start_time = rospy.Time.now()
        
        # Convert goal to grid coordinates
        goal_grid = self.world_to_grid(
            self.current_goal.pose.position.x,
            self.current_goal.pose.position.y)
        
        # Assume robot at origin for now (should get from TF)
        start_grid = self.world_to_grid(0.0, 0.0)
        
        # Plan based on selected algorithm
        if self.param_algorithm == 'astar':
            path_grid = self.astar(start_grid, goal_grid)
        elif self.param_algorithm == 'bfs':
            path_grid = self.bfs(start_grid, goal_grid)
        else:
            rospy.logwarn("Unknown algorithm: %s", self.param_algorithm)
            return
            
        # Check planning time
        planning_time = (rospy.Time.now() - start_time).to_sec()
        if planning_time > self.param_max_planning_time:
            rospy.logwarn("Planning exceeded time limit: %.2fs", planning_time)
            
        if path_grid is None:
            rospy.logwarn("No path found to goal")
            return
            
        # Convert path to world coordinates and publish
        path_msg = self.grid_path_to_msg(path_grid)
        self.path_pub.publish(path_msg)
        
        rospy.loginfo("Path planned in %.3fs, length: %d waypoints",
                     planning_time, len(path_grid))
        
    def astar(self, start, goal):
        """A* pathfinding algorithm"""
        if not self.is_valid_cell(start) or not self.is_valid_cell(goal):
            return None
            
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if self.distance(current, goal) < self.param_goal_tolerance / self.param_cell_size:
                return self.reconstruct_path(came_from, current)
                
            for neighbor in self.get_neighbors(current):
                if not self.is_valid_cell(neighbor) or self.is_occupied(neighbor):
                    continue
                    
                tentative_g = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.param_heuristic_weight * self.heuristic(neighbor, goal)
                    f_score[neighbor] = f
                    heapq.heappush(open_set, (f, neighbor))
                    
        return None
        
    def bfs(self, start, goal):
        """BFS pathfinding algorithm"""
        if not self.is_valid_cell(start) or not self.is_valid_cell(goal):
            return None
            
        queue = deque([[start]])
        visited = {start}
        
        while queue:
            path = queue.popleft()
            current = path[-1]
            
            if self.distance(current, goal) < self.param_goal_tolerance / self.param_cell_size:
                return path
                
            for neighbor in self.get_neighbors(current):
                if neighbor in visited:
                    continue
                if not self.is_valid_cell(neighbor) or self.is_occupied(neighbor):
                    continue
                    
                visited.add(neighbor)
                queue.append(path + [neighbor])
                
        return None
        
    def heuristic(self, a, b):
        """Heuristic function for A*"""
        if self.param_heuristic == 'manhattan':
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
        elif self.param_heuristic == 'euclidean':
            return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        else:
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
            
    def distance(self, a, b):
        """Euclidean distance between two grid cells"""
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
    def get_neighbors(self, cell):
        """Get valid neighbors (4-connected)"""
        x, y = cell
        neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
        return neighbors
        
    def is_valid_cell(self, cell):
        """Check if cell is within map bounds"""
        if self.current_map is None:
            return False
        x, y = cell
        return 0 <= x < self.current_map.info.width and 0 <= y < self.current_map.info.height
        
    def is_occupied(self, cell):
        """Check if cell is occupied"""
        if self.current_map is None:
            return True
        x, y = cell
        index = y * self.current_map.info.width + x
        return self.current_map.data[index] > 50  # Occupied threshold
        
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        if self.current_map is None:
            return (0, 0)
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        return (grid_x, grid_y)
        
    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates"""
        if self.current_map is None:
            return (0.0, 0.0)
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        x = grid_x * resolution + origin_x
        y = grid_y * resolution + origin_y
        return (x, y)
        
    def reconstruct_path(self, came_from, current):
        """Reconstruct path from A* came_from map"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
        
    def grid_path_to_msg(self, path_grid):
        """Convert grid path to Path message"""
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.param_global_frame
        
        for grid_cell in path_grid:
            world_x, world_y = self.grid_to_world(grid_cell[0], grid_cell[1])
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = self.param_global_frame
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            
        return path_msg
        
    def run(self):
        """Main loop"""
        rospy.spin()


if __name__ == '__main__':
    try:
        planner = GlobalPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
