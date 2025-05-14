#!/usr/bin/env python3

import rospy
import folium
import json
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse  # ROS service for triggering map creation
import shutil
from novatel_oem7_msgs.msg import BESTPOS


# Global variables to store the latest data
latest_centerline_path = None
start_gps = None
goal_gps = None


# Callback function for the /computed_path topic
def path_callback(msg):
    global latest_centerline_path
    try:
        # Parse the JSON string from the message and store it in the global variable
        latest_centerline_path = json.loads(msg.data)  # Expecting a list of [lon, lat] pairs
        rospy.loginfo("Received new A* path data!")
    except Exception as e:
        rospy.logerr(f"Error parsing path data: {e}")
        
# Callback for /localization_gps
# def localization_callback(msg):
#     global start_gps
#     start_gps = [msg.latitude, msg.longitude]  # Folium expects [lat, lon]
#     rospy.loginfo(f"Updated start GPS: {start_gps}")

# def bestpos_callback(self, msg):
#     self.start_gps = (msg.lon, msg.lat)  # Extract lat and lon from BESTPOS - might have to flip
#     rospy.loginfo("Received BESTPOS location: Lat {}, Lon {}".format(self.start_gps[1], self.start_gps[0]))

def bestpos_callback(msg):
    global start_gps
    # Folium expects [lat, lon] order.
    start_gps = [msg.lat, msg.lon]
    rospy.loginfo("Received BESTPOS location: Lat {}, Lon {}".format(start_gps[0], start_gps[1]))


# Callback for /hmi_goal
def hmi_callback(msg):
    global goal_gps
    goal_gps = [msg.latitude, msg.longitude]  # Folium expects [lat, lon]
    rospy.loginfo(f"Updated goal GPS: {goal_gps}")


# Service callback function to generate the map
# Service callback function to generate the map
def generate_map(req):
    global latest_centerline_path, start_gps, goal_gps

    if latest_centerline_path is None:
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
        # Convert [lon, lat] to [lat, lon] for Folium
        converted_path = [[pt[1], pt[0]] for pt in latest_centerline_path]

        # Determine the central point for the map (e.g., first point in the path)
        map_center = converted_path[0]

        # Create a Folium map
        m = folium.Map(location=map_center, zoom_start=16)

        # Add the computed path as a PolyLine overlay
        folium.PolyLine(converted_path, color="blue", weight=4, opacity=0.8, popup="A* Path").add_to(m)

        # Add start and goal markers using the latest GPS values
        folium.Marker(start_gps, popup="Start", icon=folium.Icon(color="green")).add_to(m)
        folium.Marker(goal_gps, popup="Goal", icon=folium.Icon(color="red")).add_to(m)

        # Add layer control
        folium.LayerControl().add_to(m)

        # Save the map to an HTML file
        m.save("a_star_path_map.html")
        rospy.loginfo("Map saved as a_star_path_map.html")

        # Optionally move the file if necessary
        destination_folder = "/home/autodrive/GP_test/ADC2Y4/src/a3_astar/map_validation"
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
    global latest_centerline_path

    # Initialize the ROS node
    rospy.init_node('a_star_visualizer', anonymous=True)

    # Subscribe to the /computed_path topic
    rospy.Subscriber('/computed_path', String, path_callback)
    
    # # Subscribe to /localization_gps (current location)
    # rospy.Subscriber('/localization_gps', NavSatFix, localization_callback)

     # New subscription for BESTPOS
    #rospy.Subscriber('/novatel/oem7/bestpos', BESTPOS, self.bestpos_callback)  # Use actual BESTPOS message type

    rospy.Subscriber('/novatel/oem7/bestpos', BESTPOS, bestpos_callback)


    # Subscribe to /gps_coordinates (goal location)
    rospy.Subscriber('/gps_coordinates', NavSatFix, hmi_callback)


    # Create a ROS service to trigger map generation
    rospy.Service('/generate_a_star_map', Trigger, generate_map)

    rospy.loginfo("A* Visualizer Node Running. Listening to /computed_path and waiting for map generation requests...")
    rospy.spin()

if __name__ == '__main__':
    try:
        visualize_a_star_path()
    except rospy.ROSInterruptException:
        pass
