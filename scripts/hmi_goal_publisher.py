#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix

def publish_hmi_goal():
    # Initialize the ROS node
    rospy.init_node('hmi_goal_publisher', anonymous=True)
    
    # Create a publisher for the /hmi_goal topic
    pub = rospy.Publisher('/hmi_goal', NavSatFix, queue_size=10)
    
    # Wait for user input to simulate the goal selection
    rospy.loginfo("HMI Goal Publisher Node Initialized.")
    rospy.loginfo("Enter goal latitude and longitude to publish a goal location (e.g., 36.074208 -79.772608):")
    
    while not rospy.is_shutdown():
        try:
            # Get user input for latitude and longitude
            goal_lat = float(input("Enter Latitude: "))
            goal_lon = float(input("Enter Longitude: "))
            
            # Create and populate the NavSatFix message
            goal_location = NavSatFix()
            goal_location.header.stamp = rospy.Time.now()  # Timestamp
            goal_location.header.frame_id = "hmi_frame"    # Frame ID
            goal_location.latitude = goal_lat
            goal_location.longitude = goal_lon
            goal_location.altitude = 0.0  # Optional
            
            # Publish the message
            rospy.loginfo(f"Publishing Goal Location: Lat={goal_lat}, Lon={goal_lon}")
            pub.publish(goal_location)

        except ValueError:
            rospy.logerr("Invalid input. Please enter numerical values for latitude and longitude.")
        except rospy.ROSInterruptException:
            break

if __name__ == '__main__':
    try:
        publish_hmi_goal()
    except rospy.ROSInterruptException:
        pass
