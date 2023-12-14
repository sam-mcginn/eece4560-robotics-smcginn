#!/usr/bin/env python3
import rospy
import time
import math
import actionlib
import example_action_server.msg
# import action messages to be used

class Fibonacci_Seq():
    def __init__(self):
        # Time variables to track:
        self.prev_time = 0
        self.request_time = 0
        self.response_time = 0
        
        # Setup proxy to example_service service:
        
        # Setup client to the action from example_action_server:
        # type = example_action_server.msg.FibonacciAction
        print("Starting client")
        self.client = actionlib.SimpleActionClient('fibonacci', example_action_server.msg.FibonacciAction)
        
        # Request Fibonacci sequences from both service and action:
        # Order 3 (three elements long)
        fib_result = self.fibonacci_client(3)
        info_msg = "Action result for Fib(3): "+str(fib_result)+", request time = "+str(self.request_time)+", response time = "+str(self.response_time)
        print(info_msg)
        rospy.loginfo(info_msg)
        
        # Order 15 (15 elements long)
        fib_result = self.fibonacci_client(15)
        info_msg = "Action result for Fib(15): "+str(fib_result)+", request time = "+str(self.request_time)+", response time = "+str(self.response_time)
        print(info_msg)
        rospy.loginfo(info_msg)
        
        # Record the time it takes to:
        # Return from the request to calculate the sequence
        # Receive the answer
        
        # Output to console
        # rospy.loginfo(string)
        
        pass
    
    def fibonacci_client(self, length):
        # Wait until action server starts listening
        print("Waiting for server")
        self.client.wait_for_server()
        
        # Send request to server
        self.prev_time = rospy.get_time()
        print("Sending goal")
        goal = example_action_server.msg.FibonacciGoal(order=length)
        
        # Send goal to server
        self.client.send_goal(goal)
        self.request_time = rospy.get_time() - self.prev_time
        
        # Wait for action to finish
        self.prev_time = rospy.get_time()
        self.client.wait_for_result()
        self.response_time = rospy.get_time() - self.prev_time
        
        # Return result
        return self.client.get_result()
        
if __name__ == '__main__':
    try:
        rospy.init_node('fibonacci_client')
        client = Fibonacci_Seq()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
