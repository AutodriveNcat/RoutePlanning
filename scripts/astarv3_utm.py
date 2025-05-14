#!/usr/bin/env python3
import rospy
import json
import heapq
import utm
from geopy.distance import geodesic
from shapely.geometry import LineString, Point
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
#!/usr/bin/env python3
# This script implements an A* path planner for a vehicle using lane-level data.
# ------------------------------------------------------------------
# LaneNode and A* setup
# ------------------------------------------------------------------
class LaneNode:
    def __init__(self, path_id, start, end, center):
        self.path_id = path_id
        self.start = tuple(start)  # (lon, lat)
        self.end = tuple(end)
        self.center = center       # list of [lon, lat] pairs
        self.g = float('inf')
        self.h = 0
        self.f = 0
        self.parent = None

    def __eq__(self, other):
        return self.path_id == other.path_id

    def __lt__(self, other):
        return self.f < other.f

def heuristic(a, b):
    # geodesic distance expects (lat, lon)
    return geodesic(a.end, b.start).meters

def load_lane_graph(lanes_file):
    with open(lanes_file, 'r') as f:
        lanes_data = json.load(f)["lanes"]
    lanes = {}
    graph = {}
    for lane in lanes_data:
        path_id = lane["path_id"]
        start = lane["start"]
        end = lane["end"]
        center = lane["center"]
        connected_to = lane["connected_to"]
        node = LaneNode(path_id, start, end, center)
        lanes[path_id] = node
        valid_connections = [conn for conn in connected_to if conn != -1]
        for connection in valid_connections:
            graph.setdefault(path_id, []).append(connection)
    return lanes, graph

def find_closest_lane_node(lanes, gps_point):
    closest_lane = None
    min_distance = float('inf')
    # gps_point is (lon, lat)
    for lane in lanes.values():
        for pt in lane.center:
            d = geodesic((gps_point[1], gps_point[0]), (pt[1], pt[0])).meters
            if d < min_distance:
                min_distance = d
                closest_lane = lane
    return closest_lane

def a_star_lane_level(graph, lanes, start_lane, goal_lane):
    open_list = []
    closed_set = set()
    start_lane.g = 0
    start_lane.f = heuristic(start_lane, goal_lane)
    heapq.heappush(open_list, start_lane)

    while open_list:
        current = heapq.heappop(open_list)
        closed_set.add(current.path_id)

        # If goal reached, build path
        if current == goal_lane:
            path = []
            while current:
                path.append(current)
                current = current.parent
            return path[::-1]

        for neighbor_id in graph.get(current.path_id, []):
            if neighbor_id in closed_set:
                continue
            if neighbor_id not in lanes:
                rospy.logwarn(f"Lane ID {neighbor_id} not found.")
                continue
            neighbor = lanes[neighbor_id]
            tentative_g = current.g + heuristic(current, neighbor)
            if tentative_g < neighbor.g:
                neighbor.g = tentative_g
                neighbor.h = heuristic(neighbor, goal_lane)
                neighbor.f = neighbor.g + neighbor.h
                neighbor.parent = current
                heapq.heappush(open_list, neighbor)
    return None

def extract_centerline_path(lane_path):
    composite = []
    for lane in lane_path:
        composite.extend(lane.center)
    return composite


def truncate_goal_lane(goal_lane_center, goal_gps, min_fraction, threshold):

    n = len(goal_lane_center)
    if n == 0:
        rospy.logwarn("Empty goal lane centerline; using goal GPS only.")
        return [goal_gps]
    
    # Compute cumulative distances along the lane
    cumulative = [0.0]
    for i in range(1, n):
        d = geodesic(
            (goal_lane_center[i-1][1], goal_lane_center[i-1][0]),
            (goal_lane_center[i][1], goal_lane_center[i][0])
        ).meters
        cumulative.append(cumulative[-1] + d)
        
    total_length = cumulative[-1]
    if total_length == 0:
        rospy.logwarn("Total length of goal lane is zero; using goal GPS only.")
        return [goal_gps]
    
    candidate_idx = None
    # Iterate through each point and log its fraction and distance from goal.
    for i, pt in enumerate(goal_lane_center):
        fraction = cumulative[i] / total_length
        dist_to_goal = geodesic((goal_gps[1], goal_gps[0]), (pt[1], pt[0])).meters
        rospy.loginfo(f"Index {i}: fraction {fraction:.2f}, distance to goal {dist_to_goal:.2f} m")
        if fraction >= min_fraction and dist_to_goal < threshold:
            candidate_idx = i
            rospy.loginfo(f"Using candidate index {i} as it meets min_fraction and threshold conditions.")
            break
    
    # If no candidate found that matches both criteria, use the last point.
    if candidate_idx is None:
        candidate_idx = n - 1
        rospy.loginfo("No candidate found meeting both min_fraction and threshold; using last index.")
    
    truncated = goal_lane_center[:candidate_idx + 1]
    truncated[-1] = goal_gps  # Force the endpoint to be exactly goal_gps.
    rospy.loginfo(f"Truncated goal lane at index {candidate_idx}.")
    return truncated

# ------------------------------------------------------------------
# Optionally remove cycles from the composite path.
# ------------------------------------------------------------------
def remove_cycles_from_path(path):
    """
    Remove loops in the final list of [lon, lat] coordinates.
    One simple strategy: if a coordinate appears more than once, remove the intermediate loop.
    (This is a simple approach and may be refined.)
    """
    seen = {}
    new_path = []
    for pt in path:
        key = tuple(pt)  # use tuple of coordinates as key
        if key in seen:
            # remove all points after the first occurrence; keep the first one
            idx = seen[key]
            new_path = new_path[:idx+1]
        else:
            seen[key] = len(new_path)
            new_path.append(pt)
    return new_path

# ------------------------------------------------------------------
# AStarPlannerNode class using ROS
# ------------------------------------------------------------------
class AStarPlannerNode:
    def __init__(self):
        rospy.init_node('a_star_planner', anonymous=True)
        rospy.Subscriber('/localization_gps', NavSatFix, self.localization_callback)
        rospy.Subscriber('/gps_coordinates', NavSatFix, self.hmi_callback)
        self.path_publisher = rospy.Publisher('/computed_path', String, queue_size=10)
        self.path_utm_publisher = rospy.Publisher('/path_utm', String, queue_size=10)
        lanes_file = '/home/byron/catkin_ws/src/a3_astar/data/lanes.json'
        self.lanes, self.graph = load_lane_graph(lanes_file)
        self.start_gps = None  # (lon, lat)
        self.goal_gps = None   # (lon, lat)
    
    def convert_to_utm(self, gps_path):
        utm_path = []
        for lon, lat in gps_path:
            utm_coords = utm.from_latlon(lat, lon)  # returns (x, y, zone number, zone letter)
            utm_path.append([utm_coords[0], utm_coords[1]])  # only use x and y
        return utm_path

    def reset_lane_states(self):
        # Reset g, h, f, and parent for all lanes before each planning request
        for lane in self.lanes.values():
            lane.g = float('inf')
            lane.h = 0
            lane.f = 0
            lane.parent = None

    def localization_callback(self, msg):
        self.start_gps = (msg.longitude, msg.latitude)
        rospy.loginfo("Received current location: {}".format(self.start_gps))
        self.try_compute_path()

    def hmi_callback(self, msg):
        self.goal_gps = (msg.longitude, msg.latitude)
        rospy.loginfo("Received goal location: {}".format(self.goal_gps))
        self.try_compute_path()

    def try_compute_path(self):
        if self.start_gps and self.goal_gps:
            rospy.loginfo("Computing path...")
            self.reset_lane_states()
            start_lane = find_closest_lane_node(self.lanes, self.start_gps)
            goal_lane = find_closest_lane_node(self.lanes, self.goal_gps)
            if not start_lane or not goal_lane:
                rospy.logerr("Could not find start or goal lane.")
                return

            lane_path = a_star_lane_level(self.graph, self.lanes, start_lane, goal_lane)
            if lane_path:
                rospy.loginfo("Lane path computed: %s", [lane.path_id for lane in lane_path])
                # Build composite centerline from all lanes except the goal lane.
                composite_centerline = []
                for lane in lane_path[:-1]:
                    composite_centerline.extend(lane.center)

                # Truncate the goal lane.
                # For the goal lane, truncate its centerline.
                # Here, we use min_fraction=0.75 (or you can adjust) and threshold=10.0 (meters)
                truncated_goal_segment = truncate_goal_lane(goal_lane.center, self.goal_gps, min_fraction=0.1, threshold=1.0)
                # truncated_goal_segment = truncate_goal_lane(goal_lane.center, self.goal_gps, min_fraction=0.50)
                final_centerline = composite_centerline + truncated_goal_segment

                # Remove any cycles/loops from the final composite if needed.
                final_centerline = remove_cycles_from_path(final_centerline)

                rospy.loginfo("Final centerline computed ({} points)".format(len(final_centerline)))
                self.path_publisher.publish(json.dumps(final_centerline))
                
                # Convert the final centerline from GPS to UTM coordinates and publish it.
                final_centerline_utm = self.convert_to_utm(final_centerline)
                self.path_utm_publisher.publish(json.dumps(final_centerline_utm))
                rospy.loginfo("Final UTM path published ({} points)".format(len(final_centerline_utm)))
            else:
                rospy.logwarn("No lane path found between start and goal.")

if __name__ == '__main__':
    try:
        planner_node = AStarPlannerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass