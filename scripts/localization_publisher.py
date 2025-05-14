#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix

def localization_publisher():
    # Initialize the ROS node
    rospy.init_node('localization_publisher', anonymous=True)
    
    # Create a publisher for the /localization_gps topic
    pub = rospy.Publisher('/localization_gps', NavSatFix, queue_size=10)
    
    # Set the rate of publishing (e.g., 1 Hz)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # Create a NavSatFix message
        car_location = NavSatFix()
        car_location.header.stamp = rospy.Time.now()  # Timestamp
        car_location.header.frame_id = "localization_frame"  # Frame ID

        # Simulated GPS coordinates (replace with real data if available)
        car_location.latitude =  36.07943656406164  # Example latitude
        car_location.longitude = -79.77618803586068  # Example longitude
        car_location.altitude = 0.0  # Example altitude (can be left as 0.0 if unused)
        
        # Log the data and publish it
        rospy.loginfo(f"Publishing Car Location: Lat={car_location.latitude}, Lon={car_location.longitude}")
        pub.publish(car_location)
        
        # Sleep to maintain the publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        localization_publisher()
    except rospy.ROSInterruptException:
        pass
