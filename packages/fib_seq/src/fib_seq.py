#!/usr/bin/env python3
import rospy
import time
import math
import actionlib
import example_action_server.msg
from example_service.srv import Fibonacci, FibonacciResponse
# import action messages to be used

class Fibonacci_Seq():
    def __init__(self):
        # Time variables
        self.prev_time = 0
        self.request_time = 0
        self.response_time = 0
        
        # Get Fibonacci sequences from service, log request time
        # Order 3 (three elements long)
        fib_result = self.req_fibonacci_service(2)
        info_msg = "Service result for Fib(2): "+str(fib_result)+", total request/response time = "+str(self.request_time)
        rospy.loginfo(info_msg)
        
        # Order 15 (15 elements long)
        fib_result = self.req_fibonacci_service(14)
        info_msg = "Service result for Fib(14): "+str(fib_result)+", total request/response time = "+str(self.request_time)
        rospy.loginfo(info_msg)
        
        
        # Setup client to the action from example_action_server:
        # type = example_action_server.msg.FibonacciAction
        #print("Starting client")
        self.client = actionlib.SimpleActionClient('fibonacci', example_action_server.msg.FibonacciAction)
        
        # Request Fibonacci sequences from action:
        # Order 3 (three elements long)
        fib_result = self.fibonacci_client(2)
        info_msg = "Action result for Fib(2): "+str(fib_result)+", request time = "+str(self.request_time)+", response time = "+str(self.response_time)
        #print(info_msg)
        # Log request/response time
        rospy.loginfo(info_msg)
        
        # Order 15 (15 elements long)
        fib_result = self.fibonacci_client(14)
        info_msg = "Action result for Fib(14): "+str(fib_result)+", request time = "+str(self.request_time)+", response time = "+str(self.response_time)
        #print(info_msg)
        # Log request/response time
        rospy.loginfo(info_msg)

    
    def req_fibonacci_service(self, length):
        # Wait for service to be available
        rospy.wait_for_service('calc_fibonacci')
        
        # Setup service proxy
        get_fibonacci = rospy.ServiceProxy('calc_fibonacci', Fibonacci)
        
        # Request service
        self.prev_time = rospy.get_time()
        response1 = calc_fibonacci(length)
        self.request_time = rospy.get_time() - self.prev_time
        
        # Get response
        return response1
        
    
    def fibonacci_client(self, length):
        # Wait until action server starts listening
        #print("Waiting for server")
        self.client.wait_for_server()
        
        # Send request to server
        self.prev_time = rospy.get_time()
        #print("Sending goal")
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
