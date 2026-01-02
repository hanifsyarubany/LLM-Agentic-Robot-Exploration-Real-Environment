#!/usr/bin/python3
# coding=utf8
# Date:2022/06/30
import sys
import time
import rospy
from kinematics import ik_transform
from armpi_pro import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from std_msgs.msg import Int32

ik = ik_transform.ArmIK()

def stop():
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    target = ik.setPitchRanges((0.00, 0.12, 0.0), -145, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 20*0), (2, 500), (3, servo_data['servo3']+300),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))

class KinematicsControllerNode(object):
    def __init__(self):
        # -------------------------------------------------------------
        # Parameters
        # -------------------------------------------------------------
        self.main_gate_topic = "/activate_kinematics_controller"
        self.object_grasping = "/activate_object_grasping"
        self.object_storing  = "/activate_object_storing"
        self.sucess_grasping = "/grasping_success_mark"
        self.sucess_storing  = "/storing_success_mark"
        # -------------------------------------------------------------
        # Subscribers
        # -------------------------------------------------------------
        ##self.main_gate_topic_sub = rospy.Subscriber(
         #  self.main_gate_topic, Int32, self.main_gate_callback, queue_size=1
        #)
        self.object_grasping_sub = rospy.Subscriber(
            self.object_grasping, Int32, self.object_grasping_callback, queue_size=1
        )
        self.object_storing_sub = rospy.Subscriber(
            self.object_storing, Int32, self.object_storing_callback, queue_size=1
        )
        # -------------------------------------------------------------
        # Publishers
        # -------------------------------------------------------------
        self.sucess_grasping_pub = rospy.Publisher(
            self.sucess_grasping, Int32, queue_size=1, latch=True)
        self.sucess_storing_pub = rospy.Publisher(
            self.sucess_storing, Int32, queue_size=1, latch=True)

        rospy.loginfo("[KinematicsController] Initializing...")
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        rospy.sleep(0.5)

    def object_grasping_callback(self,msg):
        if msg.data==1:
            rospy.loginfo("[KinematicsController] Object Grasping...")
            target = ik.setPitchRanges((0.0, 0.32, 0.018), -145, -180, 0) 
            if target: 
                servo_data = target[1]
                bus_servo_control.set_servos(self.joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
                time.sleep(2)
                bus_servo_control.set_servos(self.joints_pub, 1500, ((1, 700),))
                time.sleep(2)
                bus_servo_control.set_servos(self.joints_pub, 1500, ((4, servo_data['servo4']-200),))
        success_data = Int32(data=1)
        self.sucess_grasping_pub.publish(success_data)

    def object_storing_callback(self,msg):
        if msg.data==1:
            rospy.loginfo("[KinematicsController] Object Storing...")
            bus_servo_control.set_servos(self.joints_pub, 1500, ((1, 700), (2, 494), (3, 639),
                        (4, 145),(5, 346),(6, 463)))
            time.sleep(2)
            bus_servo_control.set_servos(self.joints_pub, 1500, ((1, 200),))
            time.sleep(2)
        target = ik.setPitchRanges((0.00, 0.12, 0.0), -145, -180, 0)
        if target:
            servo_data = target[1]
            bus_servo_control.set_servos(self.joints_pub, 1500, ((1, 20*0), (2, 500), (3, servo_data['servo3']+300),(4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
        success_data = Int32(data=1)
        self.sucess_storing_pub.publish(success_data)
        
        

if __name__ == "__main__":
    rospy.init_node("kinematics_controller_node")
    rospy.on_shutdown(stop)
    try:
        KinematicsControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass  
  


