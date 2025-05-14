#!/usr/bin/env python3

import rospy
import folium
import json
from std_msgs.msg import String

# Callback function for the /computed_path topic
def path_callback(msg):
    try:
        # Parse the JSON string from the message
        centerline_path = json.loads(msg.data)  # Expecting a list of [lon, lat] pairs

        # Convert [lon, lat] to [lat, lon] for Folium
        converted_path = [[pt[1], pt[0]] for pt in centerline_path]

        # Determine the central point for the map (e.g., first point in the path)
        map_center = converted_path[0]  # You can modify this to use a centroid if desired

        # Create a Folium map
        m = folium.Map(location=map_center, zoom_start=16)

        # Add the computed path as a PolyLine overlay
        folium.PolyLine(converted_path, color="blue", weight=4, opacity=0.8, popup="A* Path").add_to(m)

        # Add start and goal markers (assume they're the first and last points in the path)
        start_gps_folium = converted_path[0]
        goal_gps_folium = converted_path[-1]

        folium.Marker(start_gps_folium, popup="Start", icon=folium.Icon(color="green")).add_to(m)
        folium.Marker(goal_gps_folium, popup="Goal", icon=folium.Icon(color="red")).add_to(m)

        # Add layer control
        folium.LayerControl().add_to(m)

        # Save the map to an HTML file
        m.save("a_star_path_map.html")
        rospy.loginfo("Map saved as a_star_path_map.html")

    except Exception as e:
        rospy.logerr(f"Error visualizing path: {e}")

def visualize_a_star_path():
    # Initialize the ROS node
    rospy.init_node('a_star_visualizer', anonymous=True)

    # Subscribe to the /computed_path topic
    rospy.Subscriber('/computed_path', String, path_callback)

    rospy.loginfo("A* Visualizer Node Running. Listening to /computed_path...")
    rospy.spin()

if __name__ == '__main__':
    try:
        visualize_a_star_path()
    except rospy.ROSInterruptException:
        pass
