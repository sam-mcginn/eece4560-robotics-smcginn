#!/usr/bin/env python3
import rospy
import actionlib
# import action messages to be used

class Fibonacci_Seq();
    def __init__(self):
        # Setup proxy to example_service service:
        
        # Setup client to the action from example_action_server:
        
        pass
        
if __name__ == '__main__':
    try:
        rospy.init_node('fibonacci_client')
        client = Fibonacci_Seq()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
