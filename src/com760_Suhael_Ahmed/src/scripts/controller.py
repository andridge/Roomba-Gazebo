import rospy
import numpy as np
from com760_Suhael_Ahmed.srv import b00856266SetBugStatus

def run(runfile):
    with open(runfile, 'r') as rnf:
        exec(rnf.read())

algo = 'BUG0'
print('RUNNING {}...'.format(algo))

# Define the service callback function
def set_bug_status_callback(request):
    # Parse the request values
    enable_charging = request.enable_charging
    speed = request.speed
    direction = request.direction
    message = request.message

    # Handle the request based on the algorithm
    if algo == 'BUG0':
        rospy.init_node('bug0_algorithm')
        run('Bug0.py')
    elif algo == 'BUG1':
        rospy.init_node('bug1_algorithm')
        run('Bug1.py')
    elif algo == 'BUG2':
        rospy.init_node('bug2_algorithm')
        run('Bug2.py')
    else:
        print('UNKNOWN ALGORITHM ENTERED!')

    # Return a response message
    return "Algorithm '{}' executed with settings: enable_charging={}, speed={}, direction={}, message={}".format(algo, enable_charging, speed, direction, message)

# Initialize the service
rospy.init_node('bug_algorithm_launcher')
rospy.Service('set_bug_status', b00856266SetBugStatus, set_bug_status_callback)
rospy.spin()
