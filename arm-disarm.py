#!/usr/bin/env python3

# Import necessary libraries
import rospy                                   
import json
from mavros_msgs.srv import CommandBool
import asyncio
import getpass


# Import the Astral class from the module
from astral import Astral

# Create an instance of AstralTokens
reader = Astral()
# Read the Astral API token from a file
reader.read_Token()

# Function to send a WebSocket message to Astral

# Main function
async def main():
    # Initialize the ROS node
    rospy.init_node('arm_and_disarm')

    # Send an initialization log message
    reader.send_log_message("Arm and Disarm node initialized")

    # Send an arming log message
    reader.send_log_message("Arming...")

    try:
        # Wait for the arming service to be available
        rospy.wait_for_service('/mavros/cmd/arming')
        # Create a service proxy for arming
        arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        # Call the arming service
        response = arm_service(value=True)
    except rospy.ServiceException as e:
        # Send a log message if arming service call fails
        reader.send_log_message("Arming service call failed: %s" % e)

    # Wait for 5 seconds after arming
    reader.send_log_message("Waiting for 5 seconds after arming...")
    await asyncio.sleep(5)

    # Send a disarming log message
    reader.send_log_message("Disarming...")

    try:
        # Wait for the disarming service to be available
        rospy.wait_for_service('/mavros/cmd/arming')
        # Create a service proxy for disarming
        disarm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        # Call the disarming service
        response = disarm_service(value=False)
    except rospy.ServiceException as e:
        # Send a log message if disarming service call fails
        reader.send_log_message("Disarming service call failed: %s" % e)
        print("Service call failed: %s" % e)

    # Send a completion log message
    reader.send_log_message("Arm and Disarm mission completed.")

# Entry point of the script
if __name__ == "__main__":
    try:
        # Run the main function using asyncio
        asyncio.run(main())
    except Exception as e:
        # Print an error message if an exception occurs
        print(f"Error running main: {e}")
