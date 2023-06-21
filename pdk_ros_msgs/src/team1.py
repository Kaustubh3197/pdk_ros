#!/usr/bin/env python
import rospy
from pdk_ros_msg.msg import *



pub_toteam1 = rospy.Publisher('car2x_unit',PerceivedObjectContainer, queue_size=10)
 
def callback(data):
  
        perceivedobjectcontainer = PerceivedObjectContainer()
        perceivedobjectcontainer.numberOfPerceivedObjects = 1
        
        perceivedobject = PerceivedObject()        

        perceivedobject.objectID = data.u_ObjId
        perceivedobject.measurementDeltaTime = data.header.stamp.secs
        perceivedobject.objectAge = data.u_LifeCycles
        perceivedobject.objectPerceptionQuality = data.f_ObjectScore
        perceivedobject.position.x_cord.value = (data.f_DistX*100)
        perceivedobject.position.y_cord.value = (data.f_DistY*100)
        perceivedobject.velocity.cartesianVelocity.xVelocity.vel_comp_value = (data.f_VrelX*100)
        perceivedobject.velocity.cartesianVelocity.yVelocity.vel_comp_value = (data.f_VrelY*100)
        perceivedobject.acceleration.cartesianAcceleration.xAcceleration.value = (data.f_ArelX*10)
        perceivedobject.acceleration.cartesianAcceleration.yAcceleration.value = (data.f_ArelY*10)
        
        print("objectID:", data.u_ObjId)
        print("x_component:", int(data.f_DistX*100))
        print("y_component:", int(data.f_DistY*100))
        print("Xvel_component:", int(data.f_VrelX*100))
        print("Yvel_component:", int(data.f_VrelY*100))
        print("Xacc_component:", int(data.f_ArelX*10))
        print("Yacc_component:", int(data.f_ArelY*10))
      

        perceivedobjectcontainer.perceivedObjects.append(perceivedobject)
        
 
        # Publish the filtered data to the new topic
        pub_toteam1.publish(perceivedobjectcontainer)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/filtered_data_topic', pdk_RadarObjectList, callback)
    rospy.spin()
 
if __name__ == '__main__':
 listener()

