#!/usr/bin/env python3
import rospy
import tf
import math
from numpy import clip, average
from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandLong
from std_msgs.msg import Bool
from std_srvs.srv import Empty

class vehicle_state:
    arm = False
    mode = ""
    prearm_ready = False
    prearm_counter = 0
    postarm_counter = 0

class mission:
    phase = "ingress"

def mavrosStateCb(data: State):
    mav_state.mode = data

def takeoff_cb(msg):
    if msg.data is True:
        deployment.phase = "preflight"

def prearm_check_cb(msg):
    mav_state.prearm_ready = msg


def aruco_align():
    rospy.init_node('aruco_align')
    target_listener = tf.TransformListener()
    rate = rospy.Rate(15)

    forward_p = rospy.get_param("forward_p", 0.5)
    #forward_d = rospy.get_param("forward_d")
    horizontal_p = rospy.get_param("horizontal_p", 0.75)
    vertical_p = rospy.get_param("vertical_p", 0.5)
    yaw_p = rospy.get_param("yaw_p", 1)
    forward_max = rospy.get_param("forward_max", 0.125)
    horizontal_max = rospy.get_param("horizontal_max", 0.125)
    vertical_max = rospy.get_param("vertical_max", 0.25)
    yaw_max = rospy.get_param("yaw_max", 0.25)
    takeoff_height = rospy.get_param("yaw_max", 0.7)

    flight_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
    cmd_srv = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
    arm_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    reset_odom_srv = rospy.ServiceProxy('reset_odom', Empty)
    pause_odom_srv = rospy.ServiceProxy('pause_odom', Empty)
    reset_srv = rospy.ServiceProxy('reset', Empty)
    pause_srv = rospy.ServiceProxy('pause', Empty)

    takeoff_sub = rospy.Subscriber("/multijackal_03/takeoff", Bool, takeoff_cb)
    prearm_check_sub = rospy.Subscriber("prearm_check_ready", Bool, prearm_check_cb)

    forward_error_history = []
    horizontal_error_history = []
    vertical_error_history = []
    yaw_error_history = []

    landing_score_x = 0
    landing_score_y = 0

    drone_vel = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', Twist,queue_size=1)
    output_velocity = Twist()
    drone_pos = rospy.Publisher('mavros/setpoint_position/local', PoseStamped,queue_size=1)
    output_position = PoseStamped()
    while not rospy.is_shutdown():
        if deployment.phase == "alignment":
            try:
                if landing_score_y < 30 and landing_score_x < 30: 
                    ((yaw_error_forward,yaw_error_horizontal,vertical_error),_) = target_listener.lookupTransform('/multijackal_03/base_link','marker_5', rospy.Time(0))
                    ((forward_error,horizontal_error,_),_) = target_listener.lookupTransform('/multijackal_03/base_link','target', rospy.Time(0))
                    
                    forward_error_history.insert(0,forward_error)
                    horizontal_error_history.insert(0,horizontal_error)
                    vertical_error_history.insert(0,vertical_error)
                    yaw_error_history.insert(0, math.atan2(yaw_error_horizontal,yaw_error_forward))
                    
                    if len(forward_error_history) > 5:
                        forward_error_history.pop(5)
                        horizontal_error_history.pop(5)
                        vertical_error_history.pop(5)
                        yaw_error_history.pop(5)

                    forward_error = average(forward_error_history)
                    horizontal_error = average(horizontal_error_history)
                    vertical_error = average(vertical_error_history)
                    yaw_error = average(yaw_error_history)
                    forward_output = clip(-(forward_error * forward_p), -forward_max, forward_max)
                    horizontal_output = clip(-(horizontal_error * horizontal_p), -horizontal_max, horizontal_max)
                    vertical_output = clip((vertical_error * vertical_p), -vertical_max, vertical_max)
                    yaw_output = clip((yaw_error * yaw_p), -yaw_max, yaw_max)
                    
                    if abs(forward_error) < 0.05:
                        output_velocity.linear.x = 0
                        landing_score_x += 1
                    else:
                        output_velocity.linear.x = forward_output
                        landing_score_x = 0

                    if abs(horizontal_error) < 0.05:
                        output_velocity.linear.y = 0
                        landing_score_y += 1
                    else:
                        output_velocity.linear.y = horizontal_output
                        landing_score_y = 0
                    output_velocity.angular.z = yaw_output
                    output_velocity.linear.z = vertical_output

                    rospy.loginfo("Publishing {0:.2f} {1:.2f} {2:.2f} {3:.2f}".format(output_velocity.linear.x,output_velocity.linear.y,output_velocity.linear.z, output_velocity.angular.z))
                    drone_vel.publish(output_velocity)
                else:
                    rospy.loginfo("aligned, entering landing phase")
                    deployment.phase = "landing"

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Lost Aruco")
        
        elif deployment.phase == "landing":
            if(flight_mode_srv(custom_mode='AUTO.LAND')):
                rospy.loginfo("land success, waiting for disarm")
                deployment.phase = "landed"
        
        elif deployment.phase == "landed":
            if mav_state.arm is not False:
                continue
            else:
                rospy.loginfo("Resetting Odom and Map")
                if not reset_odom_srv():
                    rospy.logerr("Failed to reset odom!")
                if not pause_odom_srv():
                    rospy.logerr("Failed to pause odom!")
                if not reset_srv():
                    rospy.logerr("Failed to reset map!")
                if not pause_srv():
                    rospy.logerr("Failed to pause map!")
                rospy.signal_shutdown("Deployment Finished")
        
        elif deployment.phase == "preflight":
            rospy.loginfo("Resetting Odom and Map")
            if not reset_odom_srv() or not reset_srv() or not cmd_srv(command = 246, param1 = 1.0):
                rospy.logerr("Failed to reset")
                
            else:
                rospy.loginfo("Entering arming phase")
                deployment.phase = "arming"
            rospy.sleep(20)
        
        elif deployment.phase == "arming":
            rospy.loginfo(mav_state.prearm_counter)
            if mav_state.prearm_ready and mav_state.prearm_counter < 100: # approx 6 seconds
                mav_state.prearm_counter += 1
                if mav_state.prearm_counter >  50:
                    output_position.pose.position.z = takeoff_height
                    drone_pos.publish(output_position)
            elif mav_state.prearm_ready is False:
                rospy.loginfo("Prearm checks failed")
                mav_state.prearm_counter = 0
            elif mav_state.prearm_counter >= 100:
                if flight_mode_srv(custom_mode='OFFBOARD'):
                    rospy.loginfo("Entered Offboard Mode")
                    if arm_srv(True):
                        rospy.loginfo("Arming")
                        rospy.loginfo("Entering takeoff phase")
                        deployment.phase = "takeoff"
                else:
                    mav_state.prearm_counter = 0
        
        elif deployment.phase == "takeoff":
            rospy.loginfo(mav_state.postarm_counter)
            output_position.pose.position.z = takeoff_height
            drone_pos.publish(output_position)
            if mav_state.postarm_counter > 50:
                deployment.phase = "alignment"
            else:
                mav_state.postarm_counter += 1
        
        elif deployment.phase == "ingress":
            rospy.loginfo("Waiting to reach start point")
            rospy.sleep(10)

        rate.sleep()

if __name__ == '__main__':
    mav_state = vehicle_state()
    deployment = mission()
    try:
        aruco_align()
    except rospy.ROSInterruptException:
        pass