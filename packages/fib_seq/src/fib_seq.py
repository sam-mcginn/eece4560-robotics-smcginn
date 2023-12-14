#!/usr/bin/env python3
import rospy
import time
import math
import actionlib
# import action messages to be used

class Fibonacci_Seq();
    def __init__(self):
        # Setup proxy to example_service service:
        
        # Setup client to the action from example_action_server:
        
        # Request Fibonacci sequences from both service and action:
        # Order 3 (three elements long)
        # Order 15 (15 elements long)
        
        # Record the time it takes to:
        # Return from the request to calculate the sequence
        # Receive the answer
        
        # Output to console
        # rospy.loginfo(string)
        
        pass
        
if __name__ == '__main__':
    try:
        rospy.init_node('fibonacci_client')
        client = Fibonacci_Seq()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
