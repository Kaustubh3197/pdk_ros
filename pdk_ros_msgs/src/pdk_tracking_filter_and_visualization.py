#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from pdk_ros_msg.msg import pdk_RadarObjectList
from tf.transformations import quaternion_from_euler
import math

pub_filtered = rospy.Publisher('filtered_data_topic', pdk_RadarObjectList, queue_size=10)

def radar_object_list_callback(data):
    # Create a new marker
    marker = Marker()
    marker.header = data.header
    marker.header.frame_id = "base_footprint"
    marker.ns = "radar_objects"
    marker.id = data.u_ObjId  # Use a unique ID for the marker
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.lifetime = rospy.Duration(0.7)


    # Set the marker position and orientation to represent the whole object list
    marker.pose.position.x = data.f_DistX  # Use the first object's position
    marker.pose.position.y = data.f_DistY
    marker.pose.position.z = 0.0

    
    yaw = math.atan2(data.f_VabsY, data.f_VabsX)
    q = quaternion_from_euler(0, 0, yaw)
    

    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]

    # Calculate the overall size of the marker based on the object list
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0

    # Set the marker color to represent the whole object list
    marker.color.r = 0.0 # Use the first object's color
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    # Publish the marker
    marker_array_msg.markers = [marker]
    #print("Unfiltered Object ID:", data.u_ObjId)
    #print("Unfiltered Object Score:", data.f_ObjectScore)
    #print("Unfiltered Object DistX:", data.f_DistX)
    #print("Unfiltered Object DistY:", data.f_DistY)
    marker_array_pub_unfiltered.publish(marker_array_msg)

def filtered_radar_object_list_callback(data):
    # Create a new marker
    marker = Marker()
    marker.header = data.header
    marker.header.frame_id = "base_footprint"
    marker.lifetime = rospy.Duration(0.7)
    if data.f_ObjectScore > 0.90 and -50 < data.f_DistX < 50 and -50 < data.f_DistY < 50:
        marker.ns = "radar_objects"
        marker.id = data.u_ObjId  # Use a unique ID for the marker
        marker.action = Marker.ADD
        # Set the marker position and orientation to represent the whole object list
    
        marker.type = Marker.ARROW
        marker.color.r = 1.0 # Use the first object's color
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.position.x = data.f_DistX  # Use the first object's position
        marker.pose.position.y = data.f_DistY
        marker.pose.position.z = 1
        marker.pose.orientation.w = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0

        # Calculate the overall size of the marker based on the object list
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0


        # Publish the marker
        marker_array_msg.markers = [marker]
        #print("Filtered Object ID:", data.u_ObjId)
        #print("Filtered Object Score:", data.f_ObjectScore)
        #print("Filtered Object DistX:", data.f_DistX)
        #print("Filtered Object DistY:", data.f_DistY)
        marker_array_pub_filtered.publish(marker_array_msg)

        
    
    # Determine the object size based on bounding box distances
    ''' 
    size_x = data.f_LDeltaX_left + data.f_LDeltaX_mid + data.f_LDeltaX_right
    size_y = data.f_LDeltaY_left + data.f_LDeltaY_mid + data.f_LDeltaY_right
    size_z = 2.0  # Set a default size in the z direction (vertical)

    # Determine the object type based on size
    
    if size_x < 0.5 and size_y < 2.0 :  # Assuming size.x represents the width of the object and y represents the height 
        marker.ns = "Humans"
        marker.type = Marker.ARROW
        marker.color.r = 1.0 # Use the first object's color
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
    elif size_x < 5.0 and size_y < 2.0:
        marker.ns = "cars"
        marker.type = Marker.CYLINDER
        marker.color.r = 0.0 # Use the first object's color
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
    else:
        marker.ns = "others"
        marker.type = Marker.SPHERE  # Default shape for other objects
        marker.color.r = 0.0 # Use the first object's color
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
    '''
    

def callback(data):
    if data.f_ObjectScore > 0.9 and -50 < data.f_DistX < 50 and -50 < data.f_DistY < 50:
 
	# Create a new message for the filtered data
        filtered_message = pdk_RadarObjectList ()
        filtered_message.header = data.header
        filtered_message.u_ObjId = data.u_ObjId
        filtered_message.f_DistX = data.f_DistX
        filtered_message.f_DistY = data.f_DistY
        filtered_message.f_VrelX = data.f_VrelX
        filtered_message.f_VrelY = data.f_VrelY
        filtered_message.f_ArelX = data.f_ArelX
        filtered_message.f_ArelY = data.f_ArelY
        filtered_message.f_DistXStd = data.f_DistXStd
        filtered_message.f_DistYStd = data.f_DistYStd
        filtered_message.f_VrelXStd = data.f_VrelXStd
        filtered_message.f_VrelYStd = data.f_VrelYStd
        filtered_message.f_ArelXStd = data.f_ArelXStd
        filtered_message.f_ArelYStd = data.f_ArelYStd
        filtered_message.f_LDeltaX_left = data.f_LDeltaX_left
        filtered_message.f_LDeltaX_mid = data.f_LDeltaX_mid
        filtered_message.f_LDeltaX_right = data.f_LDeltaX_right
        filtered_message.f_LDeltaY_left = data.f_LDeltaY_left
        filtered_message.f_LDeltaY_mid = data.f_LDeltaY_mid
        filtered_message.f_LDeltaY_right = data.f_LDeltaY_right
        filtered_message.f_RCS = data.f_RCS
        filtered_message.f_ObjectScore = data.f_ObjectScore
        filtered_message.u_LifeCycles = data.u_LifeCycles
        filtered_message.f_VabsX = data.f_VabsX
        filtered_message.f_VabsY = data.f_VabsY
        filtered_message.f_AabsX = data.f_AabsX
        filtered_message.f_AabsY = data.f_AabsY
        filtered_message.f_VabsXStd = data.f_VabsXStd
        filtered_message.f_VabsYStd = data.f_VabsYStd
        filtered_message.f_AabsXStd = data.f_AabsXStd
        filtered_message.f_AabsYStd = data.f_AabsYStd
 
 
        # Print the filtered data attributes
        #print("filtered_message.u_ObjId:", data.u_ObjId)
        #print("filtered_message.f_ObjectScore:", data.f_ObjectScore)
        #print("filtered_message.f_DistX:", data.f_DistX)
        #print("filtered_message.f_DistY:", data.f_DistY)
 
        # Publish the filtered data to the new topic
        pub_filtered.publish(filtered_message)
    

if __name__ == '__main__':
    rospy.init_node('pdk_tracking_filter_and_visualization')
    # Create a marker array publisher
    marker_array_pub_unfiltered = rospy.Publisher('/pdk/tracking/visualization_marker_array', MarkerArray, queue_size=10)
    marker_array_pub_filtered = rospy.Publisher('/pdk/tracking/filtered_marker_array', MarkerArray, queue_size=10)
    marker_array_msg = MarkerArray()

    # Subscribe to the pdk_RadarObjectList topic
    rospy.Subscriber('/pdk/tracking', pdk_RadarObjectList, radar_object_list_callback)
    rospy.Subscriber('/pdk/tracking', pdk_RadarObjectList, filtered_radar_object_list_callback)
    rospy.Subscriber('/pdk/tracking', pdk_RadarObjectList, callback)
    # Spin and wait for messages
    rospy.spin()
