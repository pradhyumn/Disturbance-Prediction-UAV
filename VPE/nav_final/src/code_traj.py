#!/usr/bin/env python

# ROS python API
import rospy
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
# Trajectory
from nav_msgs.msg import Path
import tf
from tf.transformations import quaternion_from_euler
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Autoland Mode could not be set."%e

# Main class: Converts joystick commands to position setpoints
class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp         = PositionTarget()  #this is the thing published
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask    = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame= 1

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        #self.ALT_SP        = 3.0
        # update the setpoint message with the required altitude
        #self.sp.position.z    = self.ALT_SP
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0
        self.sp.position.z = 0.0



        self.path = Path()
        self.cur_pose = PoseStamped()


    ## Drone State callback     
    def stateCb(self, msg):
    	self.state = msg


    def callback(self, msg):
    	self.cur_pose.header = msg.header

    	self.cur_pose.pose.position.x = msg.position.x
    	self.cur_pose.pose.position.y = msg.position.y
    	self.cur_pose.pose.position.z = msg.position.z

    	print(self.cur_pose.pose.position.x)

    	#self.roll = 0
    	#self.pitch = 0
    	self.quaternion = tf.transformations.quaternion_from_euler(0,0,msg.yaw)

    	self.cur_pose.pose.orientation.x = self.quaternion[0]
    	self.cur_pose.pose.orientation.y = self.quaternion[1]
    	self.cur_pose.pose.orientation.z = self.quaternion[2]
    	self.cur_pose.pose.orientation.w = self.quaternion[3]

    	self.path.header = msg.header
    	self.path.poses.append(self.cur_pose)

    	print("-- Navigation Message Received --")
    	#rate.sleep()

    ###################
    ## End Callbacks ##
    ###################
    

# Main function
def main():

    # initiate node
    rospy.init_node('setpoints_node')#, anonymous=True)

    # flight mode object
    modes = fcuModes()
    # controller object
    cnt = Controller()

    # ROS loop rate, [Hz]
    rate = rospy.Rate(30.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    rospy.Subscriber('/planning/pos_cmd', PositionCommand, cnt.callback)

    path_pub = rospy.Publisher('/mavros/trajectory/path', Path, queue_size=100)

    pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=100)
    
    #sp_pub = rospy.Publisher('/setpoints', PositionTarget, queue_size=100)


    #     # Make sure the drone is armed
    while not cnt.state.armed:
    	modes.setArm()
        rate.sleep()

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect -- pass the local position only
    k=0
    while k<100:
        pose_pub.publish(cnt.cur_pose)
        rate.sleep()
        print(k)
        k = k+1

    # activate OFFBOARD mode
    modes.setOffboardMode()

    # ROS main loop
    while not rospy.is_shutdown():
        #cnt.updateSp()
        pose_pub.publish(cnt.cur_pose)
        path_pub.publish(cnt.path)         ### publish the trajectory here
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
