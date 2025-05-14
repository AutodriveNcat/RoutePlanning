#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String  # For publishing the final path as a string (optional)
import json
import heapq
from shapely.geometry import LineString, Point
from geopy.distance import geodesic

# LaneNode and A* Algorithm as in your implementation
class LaneNode:
    def __init__(self, path_id, start, end, center):
        self.path_id = path_id
        self.start = tuple(start)
        self.end = tuple(end)
        self.center = center
        self.g = float('inf')  # Initialize with infinity for A*
        self.h = 0
        self.f = 0
        self.parent = None

    def __eq__(self, other):
        return self.path_id == other.path_id

    def __lt__(self, other):
        return self.f < other.f

def heuristic(a, b):
    return geodesic(a.end, b.start).meters

# def load_lane_graph(lanes_file):
#     with open(lanes_file, 'r') as f:
#         lanes_data = json.load(f)["lanes"]

#     lanes = {}
#     graph = {}

#     for lane in lanes_data:
#         path_id = lane["path_id"]
#         start = lane["start"]
#         end = lane["end"]
#         center = lane["center"]
#         connected_to = lane["connected_to"]

#         lane_node = LaneNode(path_id, start, end, center)
#         lanes[path_id] = lane_node

#         # Filter out invalid connections (e.g., -1)
#         valid_connections = [conn for conn in connected_to if conn != -1]
#         for connection in valid_connections:
#             graph.setdefault(path_id, []).append(connection)

#     return lanes, graph

def load_lane_graph(lanes_file):
    with open(lanes_file, 'r') as f:
        lanes_data = json.load(f)["lanes"]  # Load the JSON file here

    lanes = {}
    graph = {}
    for lane in lanes_data:
        path_id = lane["path_id"]
        start = lane["start"]
        end = lane["end"]
        center = lane["center"]
        connected_to = lane["connected_to"]

        lane_node = LaneNode(path_id, start, end, center)
        lanes[path_id] = lane_node

        valid_connections = [conn for conn in connected_to if conn != -1]
        for connection in valid_connections:
            graph.setdefault(path_id, []).append(connection)

    return lanes, graph


def find_closest_lane_node(lanes, gps_point):
    closest_lane = None
    min_distance = float('inf')

    for lane in lanes.values():
        start_distance = geodesic(gps_point, lane.start).meters
        end_distance = geodesic(gps_point, lane.end).meters
        if start_distance < min_distance:
            min_distance = start_distance
            closest_lane = lane
        if end_distance < min_distance:
            min_distance = end_distance
            closest_lane = lane

    return closest_lane

def a_star_lane_level(graph, lanes, start_lane, goal_lane):
    open_list = []
    closed_list = set()

    # Initialize the start node
    start_lane.g = 0
    start_lane.f = heuristic(start_lane, goal_lane)

    heapq.heappush(open_list, start_lane)

    while open_list:
        current_node = heapq.heappop(open_list)
        closed_list.add(current_node.path_id)

        # Check if we've reached the goal
        if current_node == goal_lane:
            path = []
            while current_node:
                path.append(current_node)
                current_node = current_node.parent
            return path[::-1]

        # Explore neighbors
        for neighbor_id in graph.get(current_node.path_id, []):
            if neighbor_id in closed_list:
                continue
            if neighbor_id not in lanes:
                rospy.logwarn(f"Lane ID {neighbor_id} not found. Skipping.")
                continue

            neighbor_node = lanes[neighbor_id]
            tentative_g = current_node.g + heuristic(current_node, neighbor_node)

            if tentative_g < neighbor_node.g or neighbor_node not in open_list:
                neighbor_node.g = tentative_g
                neighbor_node.h = heuristic(neighbor_node, goal_lane)
                neighbor_node.f = neighbor_node.g + neighbor_node.h
                neighbor_node.parent = current_node

                if neighbor_node not in open_list:
                    heapq.heappush(open_list, neighbor_node)

    return None

def extract_centerline_path(lane_path):
    complete_centerline = []
    for lane in lane_path:
        complete_centerline.extend(lane.center)
    return complete_centerline

# ----- New helper for cutting a LineString -----
def cut(line, distance):
    """
    Splits a LineString at the provided distance (measured along the line).
    Returns a tuple: (first_part, second_part) as two LineStrings.
    """
    if distance <= 0:  # Return empty first part and the whole line as second part.
        return LineString([]), line
    if distance >= line.length:
        return line, LineString([])
    
    coords = list(line.coords)
    accumulated = 0.0
    for i in range(1, len(coords)):
        p0 = coords[i - 1]
        p1 = coords[i]
        seg = LineString([p0, p1])
        seg_length = seg.length
        if accumulated + seg_length >= distance:
            ratio = (distance - accumulated) / seg_length
            cut_point = (p0[0] + ratio * (p1[0] - p0[0]), p0[1] + ratio * (p1[1] - p0[1]))
            first_part_coords = coords[:i] + [cut_point]
            second_part_coords = [cut_point] + coords[i:]
            return LineString(first_part_coords), LineString(second_part_coords)
        accumulated += seg_length

    return line, LineString([])

# Main class for subscribing to GPS and running A*
class AStarPlannerNode:
    def __init__(self):
        rospy.init_node('a_star_planner', anonymous=True)

        # Topics to subscribe
        rospy.Subscriber('/localization_gps', NavSatFix, self.localization_callback)
        rospy.Subscriber('/gps_coordinates', NavSatFix, self.hmi_callback)

        # Publisher for the computed path
        self.path_publisher = rospy.Publisher('/computed_path', String, queue_size=10)

        # # Load lane data from JSON
        # with open('/home/byron/catkin_ws/src/a3_astar/data/lanes.json', 'r') as f: ### must be updated to correct directory
        #     lanes_data = json.load(f)["lanes"]
        # self.lanes, self.graph = load_lane_graph(lanes_data)
        
        with open('/home/byron/catkin_ws/src/a3_astar/data/lanes.json', 'r') as f:
            lanes_data = json.load(f)
        self.lanes, self.graph = load_lane_graph('/home/byron/catkin_ws/src/a3_astar/data/lanes.json')  # Pass the file path here


        # Initialize variables
        self.start_gps = None
        self.goal_gps = None

    def localization_callback(self, msg):
        # Extract the current location of the vehicle
        self.start_gps = (msg.longitude, msg.latitude)
        rospy.loginfo(f"Received current location: {self.start_gps}")
        self.try_compute_path()

    def hmi_callback(self, msg):
        # Extract the goal location from HMI
        self.goal_gps = (msg.longitude, msg.latitude)
        rospy.loginfo(f"Received goal location: {self.goal_gps}")
        self.try_compute_path()

    def try_compute_path(self):
    # Only compute the path when both start and goal are available
        if self.start_gps and self.goal_gps:
            rospy.loginfo("Computing path...")
            start_lane = find_closest_lane_node(self.lanes, self.start_gps)
            goal_lane = find_closest_lane_node(self.lanes, self.goal_gps)

            if not start_lane or not goal_lane:
                rospy.logerr("Unable to find closest lanes for the provided GPS points.")
                return

            # Run A* algorithm
            lane_path = a_star_lane_level(self.graph, self.lanes, start_lane, goal_lane)
            if lane_path:
                centerline_path = extract_centerline_path(lane_path)
                rospy.loginfo("Path successfully computed.")

                # ---- TRUNCATE THE CENTERLINE PATH AT THE GOAL_GPS ----
                # Create a Shapely LineString from the centerline.
                centerline_line = LineString(centerline_path)
                # Create a Shapely point for the goal.
                goal_point = Point(self.goal_gps)  # (lon, lat)

                # Project the goal point onto the centerline (this gives distance along the line).
                proj_distance = centerline_line.project(goal_point)
                # Use the cut() function to split the line at the projection.
                first_part, _ = cut(centerline_line, proj_distance)
                # Get the truncated line coordinates as a list.
                truncated_centerline = list(first_part.coords)
                # Force the endpoint to be exactly the goal_gps.
                truncated_centerline[-1] = self.goal_gps

                rospy.loginfo("Truncated centerline computed. Publishing...")
                # Publish the truncated centerline path
                self.path_publisher.publish(json.dumps(truncated_centerline))
            else:
                rospy.logwarn("No path found between start and goal.")

if __name__ == '__main__':
    try:
        planner_node = AStarPlannerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
