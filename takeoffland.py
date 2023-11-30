#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
# Import necessary libraries                                  
import json
import asyncio
import getpass
# Import the AstralTokens class from the module
from astral import Astral
# Create an instance of AstralTokens
reader = Astral()
# Read the Astral API token from a file
reader.read_Token()
# Extract drone information and API key
# Function to send a WebSocket message to Astral

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def update_pose(data):
    global localpose
    localpose=data
async def main():

    global localpose
    localpose=PoseStamped()
    takeoff_pose = PoseStamped()
    land_pose=PoseStamped()
    rospy.init_node("offb_node_py")
    reader.send_log_message("Takeoff and Land app initialized!")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    reader.send_log_message("Arming..")

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=update_pose)
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()
    while localpose.pose.position.x!=0:
        takeoff_pose=localpose# to get current r p y values
        break
    takeoff_pose.pose.position.z = 3
    

    # Send a few setpoints before starting
    for i in range(10):
        if(rospy.is_shutdown()):
            break
        local_pos_pub.publish(takeoff_pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'
    land_set_mode = SetModeRequest()
    land_set_mode.custom_mode = 'AUTO.LAND'
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    reader.send_log_message("Vehicle armed")

                last_req = rospy.Time.now()

        local_pos_pub.publish(takeoff_pose)
        reader.send_log_message(f"Taking off to {takeoff_pose.pose.position.z} meters ..")

        if localpose.pose.position.z > 2.6:

                for i in range(100):
                    local_pos_pub.publish(takeoff_pose)
                    rate.sleep()# to wait 5 sec.
                for i in range(2):
                    if(set_mode_client.call(land_set_mode).mode_sent == True):
                        reader.send_log_message("Landing..")
                        rospy.loginfo("LAND enabled")
                break                    
        rate.sleep()
if __name__ == "__main__":
    try:
        # Run the main function using asyncio
        asyncio.run(main())
    except Exception as e:
        # Print an error message if an exception occurs
        print(f"Error running main: {e}")