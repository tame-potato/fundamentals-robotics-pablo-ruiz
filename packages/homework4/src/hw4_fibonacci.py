#! /usr/bin/env python3

import rospy
import actionlib
import example_action_server.msg
from example_service.srv import Fibonacci

def hw4_service_client(num_elements):
    
    rospy.wait_for_service('/calc_fibonacci')
    
    try:
        
        service_client = rospy.ServiceProxy( '/calc_fibonacci', Fibonacci )
        
        rospy.loginfo( "Time before sending service request" )
        output = service_client( num_elements ) 
        rospy.loginfo( "Time when service returns" )
        
        return output

    except rospy.ServiceException as e:
        
        print( "Service call failed: %s" %e )

def hw4_action_client( num_elements ):
    
    action_client = actionlib.SimpleActionClient( '/fibonacci', example_action_server.msg.FibonacciAction )

    action_client.wait_for_server()

    goal = example_action_server.msg.FibonacciGoal( order = num_elements )

    rospy.loginfo( "Time before action goal is sent" )
    
    action_client.send_goal( goal )

    rospy.loginfo( "Time after action goal is sent" )

    action_client.wait_for_result()

    rospy.loginfo( "Time when action returns with result" )

    return action_client.get_result()

if __name__ == '__main__':
    try:

        rospy.init_node("hw4_fibonacci_client")

        if rospy.has_param("num_elements"):
            num_elements = rospy.get_param("num_elements")
        else:
            num_elements = 10
        
        result_action = hw4_action_client(num_elements)
        print("Result:", ', '.join([str(n) for n in result_action.sequence]))
        
        result_service = hw4_service_client(num_elements)
        print("Result:", ', '.join([str(n) for n in result_service.sequence]))
        
    except rospy.ROSInterruptException:
        
        print("Program interrupted before completion", file=sys.stderr)


    
