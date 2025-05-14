#!/usr/bin/env python3

import rospy
import folium
import json
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse  # ROS service for triggering map creation
import shutil

# Global variables to store the latest data
# Expecting the computed JSON to contain two keys: "composite" (raw lane path selection) and "truncated" (after truncation)
latest_composite_path = None
latest_truncated_path = None
start_gps = None
goal_gps = None

# Callback function for the /computed_path topic.
# We assume the message data is a JSON object containing keys: "composite" and "truncated"
def path_callback(msg):
    global latest_composite_path, latest_truncated_path
    try:
        data = json.loads(msg.data)
        rospy.loginfo(f"Path data received: {data}")
        if isinstance(data, dict):
            if "composite" in data and "truncated" in data:
                latest_composite_path = data["composite"]
                latest_truncated_path = data["truncated"]
            else:
                rospy.logwarn("No composite key found; falling back to truncated only.")
                latest_truncated_path = data
        else:
            latest_truncated_path = data
            rospy.loginfo("Received data as list (assumed truncated path).")
    except Exception as e:
        rospy.logerr(f"Error parsing path data: {e}")

# Callback for /localization_gps (current location)
def localization_callback(msg):
    global start_gps
    # Folium expects [lat, lon]
    start_gps = [msg.latitude, msg.longitude]
    rospy.loginfo(f"Updated start GPS: {start_gps}")

# Callback for /gps_coordinates (goal location)
def hmi_callback(msg):
    global goal_gps
    # Folium expects [lat, lon]
    goal_gps = [msg.latitude, msg.longitude]
    rospy.loginfo(f"Updated goal GPS: {goal_gps}")

# Service callback function to generate the map.
def generate_map(req):
    global latest_composite_path, latest_truncated_path, start_gps, goal_gps

    if latest_truncated_path is None:
        return TriggerResponse(
            success=False,
            message="No path data available to visualize!"
        )
    if start_gps is None or goal_gps is None:
        return TriggerResponse(
            success=False,
            message="Start or goal GPS data is missing!"
        )
    
    try:
        # We need to convert the coordinates from [lon, lat] to [lat, lon] for Folium.
        # For the composite (raw) lane path:
        if latest_composite_path:
            composite_converted = [[pt[1], pt[0]] for pt in latest_composite_path]
        else:
            composite_converted = None

        # For the truncated path:
        truncated_converted = [[pt[1], pt[0]] for pt in latest_truncated_path]

        # Determine the central point for the map (take the first point of the composite if available,
        # otherwise of the truncated path)
        if composite_converted and len(composite_converted) > 0:
            map_center = composite_converted[0]
        else:
            map_center = truncated_converted[0]

        # Create a Folium map
        m = folium.Map(location=map_center, zoom_start=16)

        # Add the composite lane path (full, before truncation) if available
        if composite_converted:
            folium.PolyLine(
                composite_converted,
                color="blue",
                weight=4,
                opacity=0.7,
                popup="Lane Path Selection (Composite)"
            ).add_to(m)

        # Add the truncated centerline path as another overlay
        folium.PolyLine(
            truncated_converted,
            color="red",
            weight=8,
            opacity=0.3,
            popup="Truncated Centerline"
        ).add_to(m)

        # Add start and goal markers using the latest GPS values
        folium.Marker(start_gps, popup="Start", icon=folium.Icon(color="green")).add_to(m)
        folium.Marker(goal_gps, popup="Goal", icon=folium.Icon(color="red")).add_to(m)

        # Add layer control so you can toggle overlays
        folium.LayerControl().add_to(m)

        # Save the map to an HTML file
        m.save("a_star_path_map.html")
        rospy.loginfo("Map saved as a_star_path_map.html")

        # Optionally, move the file if necessary
        destination_folder = "/home/byron/catkin_ws/src/a3_astar/map_validation"
        shutil.move("a_star_path_map.html", f"{destination_folder}/a_star_path_map.html")
        rospy.loginfo(f"Map moved to {destination_folder}/a_star_path_map.html")

        return TriggerResponse(
            success=True,
            message=f"Map successfully generated and moved to {destination_folder}/a_star_path_map.html!"
        )
    except Exception as e:
        rospy.logerr(f"Error visualizing path: {e}")
        return TriggerResponse(
            success=False,
            message=f"Failed to generate map: {e}"
        )

def visualize_a_star_path():
    rospy.init_node('a_star_visualizer', anonymous=True)
    
    # Subscribe to /computed_path which now sends both composite and truncated paths
    rospy.Subscriber('/computed_path', String, path_callback)
    # Subscribe to /localization_gps (current location)
    rospy.Subscriber('/localization_gps', NavSatFix, localization_callback)
    # Subscribe to /gps_coordinates (goal location)
    rospy.Subscriber('/gps_coordinates', NavSatFix, hmi_callback)
    
    # Create a ROS service to trigger map generation
    rospy.Service('/generate_a_star_map', Trigger, generate_map)
    
    rospy.loginfo("A* Visualizer Node Running. Listening for path data and waiting for map generation requests...")
    rospy.spin()

if __name__ == '__main__':
    try:
        visualize_a_star_path()
    except rospy.ROSInterruptException:
        pass