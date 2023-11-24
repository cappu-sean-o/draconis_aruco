#!/usr/bin/env python3
import rospy
import tf2_ros
import tf
import tf_conversions
from fiducial_msgs.msg import FiducialTransform,FiducialTransformArray
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
rospy.init_node('aruco_pcontrol', anonymous=True)
import numpy as np
import time
target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
flight_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
arm_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
reset_odom_srv = rospy.ServiceProxy('reset_odom', Empty)
pause_odom_srv = rospy.ServiceProxy('pause_odom', Empty)
reset_srv = rospy.ServiceProxy('reset', Empty)
pause_srv = rospy.ServiceProxy('pause', Empty)
x_p_gain = float(rospy.get_param('~x_p_gain'))
y_p_gain = float(rospy.get_param('~y_p_gain'))
yaw_vy_p_gain = float(rospy.get_param('~yaw_vy_p_gain'))
yaw_p_gain = float(rospy.get_param('~yaw_p_gain'))
target_frame=rospy.get_param('~drone_frame')
max_v=rospy.get_param('~max_v')
max_yaw_rate=rospy.get_param('~max_yaw_rate')
takeoff_height=rospy.get_param('~takeoff_height')
error_margin=0.05
# tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
tf_listener = tf.TransformListener()
transform = None
mavros_state=None
landed=False
local_pose=None
no_aruco=True
finished_takeoff=False
takeoff=False
close_enough=False

#Positive yaw  rate is clockwise
def constructTarget(vx, vy, vz,yaw_rate):
    target_raw_pose = PositionTarget()
    target_raw_pose.header.stamp = rospy.Time.now()

    target_raw_pose.coordinate_frame = PositionTarget.FRAME_BODY_NED  # 8
    target_raw_pose.velocity.x = np.clip(vx,-max_v,max_v)
    target_raw_pose.velocity.y = np.clip(vy,-max_v,max_v)
    target_raw_pose.velocity.z = np.clip(vz,-max_v,max_v)
    target_raw_pose.yaw_rate = np.clip(yaw_rate,-max_yaw_rate,max_yaw_rate)

    mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ \
        | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ \
        | PositionTarget.FORCE | PositionTarget.IGNORE_YAW

    target_raw_pose.type_mask = mask  

    return target_raw_pose
def constructTargetHeight(vx, vy, vz,pz,yaw_rate):
    target_raw_pose = PositionTarget()
    target_raw_pose.header.stamp = rospy.Time.now()

    target_raw_pose.coordinate_frame = PositionTarget.FRAME_BODY_NED  # 8
    target_raw_pose.velocity.x = np.clip(vx,-max_v,max_v)
    target_raw_pose.velocity.y = np.clip(vy,-max_v,max_v)
    target_raw_pose.velocity.z = vz
    target_raw_pose.position.x = 0
    target_raw_pose.position.y = 0
    target_raw_pose.position.z = pz
    target_raw_pose.yaw_rate = np.clip(yaw_rate,-max_yaw_rate,max_yaw_rate)

    mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY \
        | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ \
        | PositionTarget.FORCE | PositionTarget.IGNORE_YAW

    target_raw_pose.type_mask = mask  

    return target_raw_pose
def fiducialCb(data: FiducialTransformArray):
    global landed, finished_takeoff,no_aruco, close_enough, takeoff_height
    for fid in data.transforms:
        if fid.fiducial_id!=5:
            continue
        #rospy.loginfo("got an aruco transform")
        
        x_error=-(x_pos_goal-fid.transform.translation.z)
        y_error=(-fid.transform.translation.x)
        z_error=-(0.2+fid.transform.translation.y)
        quat=fid.transform.rotation
        # yaw_error = -tf_conversions.transformations.euler_from_quaternion(
        #             [quat.x, quat.y, quat.z, quat.w])[1]
        yaw_error = tf_conversions.transformations.euler_from_quaternion(
                    [quat.x, quat.y, quat.z, quat.w])[1]
        # rospy.loginfo(f"{x_error},{y_error},{yaw_error}")
        #rospy.loginfo(f"{tf_conversions.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[0]},{tf_conversions.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[1]},{tf_conversions.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]}")
        if (not landed)  :
            no_aruco=False
            vx=0
            vy=0
            vz=0
            vyaw=0
            if(abs(x_error)<error_margin and abs(y_error)<error_margin and abs(yaw_error)<error_margin):
                close_enough=True
                rospy.loginfo("Close Enough is True")
                vz=z_error*x_p_gain
                if abs(z_error)<error_margin:
                    rospy.loginfo("all aligned")
                    if(flight_mode_srv(custom_mode='AUTO.LAND')):
                        rospy.loginfo("land success, waiting for disarm")
                        landed=True
                        while(mavros_state.armed):
                            rospy.sleep(0.1)
                        rospy.loginfo("Resetting Odom and Map")
                        if(not reset_odom_srv()):
                            rospy.logerr("Failed to reset odom!")
                        if(not pause_odom_srv()):
                            rospy.logerr("Failed to pause odom!")
                        if(not reset_srv()):
                            rospy.logerr("Failed to reset map!")
                        if(not pause_srv()):
                            rospy.logerr("Failed to pause map!")
                        rospy.signal_shutdown("Deployment Finished")

                    else:
                        rospy.loginfo("land fail")

            # else:
            vx=x_error*x_p_gain
            vy=y_error*y_p_gain+yaw_error*yaw_vy_p_gain*fid.transform.translation.z
            vz=z_error*x_p_gain
            # pos yaw rate is clockwise
            vyaw=yaw_error*yaw_p_gain
            if close_enough:
                rospy.loginfo("Pubbing constructTargetHeight, Z not in error margin")
                target=constructTargetHeight(vx,vy,0.4,takeoff_height,vyaw)
            else:
                # target=constructTarget(vx,vy,vz,vyaw)
                target=constructTargetHeight(vx,vy,vz,takeoff_height,vyaw)
            if finished_takeoff:
                target_pub.publish(target)
                rospy.loginfo(f"PUBBING x {fid.transform.translation.z:.3f} xe{x_error:.3f} ye{y_error:.3f} ze{z_error:.3f} yawe{yaw_error:.3f}")
            else:
                rospy.loginfo(f"NOT PUBBING x {fid.transform.translation.z:.3f} xe{x_error:.3f} ye{y_error:.3f} ze{z_error:.3f} yawe{yaw_error:.3f}")
            # rospy.loginfo(f"vx{vx} vy{vy} vz{vz} vyaw{vyaw}")
            # change to pos x(default 1m) and vy, do not touch vz
def mavrosStateCb(data: State):
    global mavros_state
    mavros_state=data



def local_pose_callback(msg):
    global local_pose
    local_pose = msg

def takeoff_cb(msg):
    global takeoff
    if(msg.data==True):
        takeoff=True
local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, local_pose_callback)
takeoff_sub = rospy.Subscriber("takeoff", Bool, takeoff_cb)
'''
main ROS thread
'''
def main():
    global finished_takeoff, no_aruco

    rospy.Subscriber("fiducial_transforms",
                     FiducialTransformArray, fiducialCb, queue_size=1)
    rospy.Subscriber("mavros/state", State, mavrosStateCb, queue_size=1)
    
    # for i in range(200):
    #     if rospy.is_shutdown():
    #         break
    #     if local_pose is not None:
    #         break
    #     else:
    #         print("Waiting for initialization.")
    #         rospy.sleep(0.5)
    #     if i==199:
    #         rospy.logerr("Failed to get local pose message!")
    #         rospy.signal_shutdown("Failed to get local pose message!")
    # rospy.loginfo("Waiting for takeoff command")

    # def arm():
    #     if arm_srv(True):
    #         return True
    #     else:
    #         print("Vehicle arming failed!")
    #         return False

    # target_msg = constructTargetHeight(0, 0, 0.4,takeoff_height, 0)
    # rospy.loginfo("Waiting for Offboard")
    # while not (mavros_state.mode=="OFFBOARD" and takeoff):
    #     if rospy.is_shutdown():
    #         return
    #     target_pub.publish(target_msg)
    #     rospy.sleep(0.1)
    # rospy.loginfo("Offboard set, arming")
    
    # if(not reset_odom_srv()):
    #     rospy.logerr("Failed to reset odom!")
    # if(not reset_srv()):
    #     rospy.logerr("Failed to reset map!")
    # for _ in range(100):
    #     if not arm():
    #         rospy.logerr("Failed to arm after offboard!")
    #         return
    #     if local_pose.pose.position.z>takeoff_height:
    #         break
    #     target_pub.publish(target_msg)
    #     rospy.sleep(0.1)
    finished_takeoff=True
    target_msg = constructTargetHeight(0, 0, 0,takeoff_height, 0)
    rospy.loginfo("Waiting for aruco transform")
    x_list=[]
    y_list=[]
    z_list=[]
    yaw_list=[]
    landed=False
    close_enough=False
    while not rospy.is_shutdown():
        # rospy.sleep(0.1)
        try:
            (trans,_) = tf_listener.lookupTransform('/multijackal_03/base_link', '/target', rospy.Time(0))
            (_,rot) = tf_listener.lookupTransform('/marker_5','/multijackal_03/base_link' , rospy.Time(0))
            # rospy.loginfo("aruco detected")
            
            # Assuming xyz are the same
            x_list.append(trans[0])
            y_list.append(trans[1])
            z_list.append(trans[2])
            x_error=trans[0]
            y_error=trans[1]
            z_error=trans[2]
            euler = tf_conversions.transformations.euler_from_quaternion(
                        rot)
            yaw_idx=1 #TODO: check which idx is yaw
            print("Change yaw_idx to whichever looks like yaw",round(euler[0],2),round(euler[1],2),round(euler[2],2))
            yaw_error=euler[yaw_idx]
            if (not landed)  :
                vx=0
                vy=0
                vz=0
                vyaw=0
                if(abs(trans[0])<error_margin and abs(y_error)<error_margin and abs(yaw_error)<error_margin):
                    close_enough=True
                    rospy.loginfo("Close Enough is True")
                    vz=z_error*x_p_gain
                    if abs(z_error)<error_margin:
                        rospy.loginfo("all aligned")
                        if(flight_mode_srv(custom_mode='AUTO.LAND')):
                            rospy.loginfo("land success, waiting for disarm")
                            landed=True
                            while(mavros_state.armed):
                                rospy.sleep(0.1)
                            rospy.loginfo("Resetting Odom and Map")
                            if(not reset_odom_srv()):
                                rospy.logerr("Failed to reset odom!")
                            if(not pause_odom_srv()):
                                rospy.logerr("Failed to pause odom!")
                            if(not reset_srv()):
                                rospy.logerr("Failed to reset map!")
                            if(not pause_srv()):
                                rospy.logerr("Failed to pause map!")
                            rospy.signal_shutdown("Deployment Finished")

                        else:
                            rospy.loginfo("land fail")
                        break

                # else:
                vx=x_error*x_p_gain
                vy=y_error*y_p_gain+yaw_error*yaw_vy_p_gain*abs(x_error)
                vz=z_error*x_p_gain
                # pos yaw rate is clockwise
                vyaw=yaw_error*yaw_p_gain
                if close_enough:
                    rospy.loginfo("Pubbing constructTargetHeight, Z not in error margin")
                    target=constructTargetHeight(vx,vy,0.4,takeoff_height,vyaw)
                else:
                    # target=constructTarget(vx,vy,vz,vyaw)
                    target=constructTargetHeight(vx,vy,vz,takeoff_height,vyaw)

                target_pub.publish(target)
                # rospy.loginfo(f"PUBBING xe{x_error:.3f} ye{y_error:.3f} ze{z_error:.3f} yawe{yaw_error:.3f}")
                
            
        except:
            target_pub.publish(target_msg)
            continue
    

if __name__ == '__main__':
    main()