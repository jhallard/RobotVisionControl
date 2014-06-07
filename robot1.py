#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_mudd')
import rospy
import irobot_mudd
from std_msgs.msg import String
from irobot_mudd.srv import *
from irobot_mudd.msg import *
import time
import math


####
# D is our global system state
####

class Data: pass	# empty class for a generic data holder
D = Data()  # an object to hold our system's services and state
D.sdata = None


#####
#####
##	 Helper Functions, these functions help break up the state_functions into more modular pieces.
#####
#####

# Transition, this function is called between states. It checks all of the flags to see if there are any interrupts 
#(user override signal, stop signal, win signal) and then calls the next state function passed to it
def transition( time, next_state_fn ):
	""" time is a float, next_state_fn is a function """
	global D # our global data...

	# check if we need to stop...
	if D.STOP_FLAG == True:
		print "Stopping FSM"
		D.STOP_FLAG = False
		return  # stops the FSM
	# call the next state function after the given amount of time has passed
	rospy.Timer( rospy.Duration(time), next_state_fn, oneshot=True )
		

		
			
#####
#####
##	  State Functions - these functions each act as one particular state for the robot
#####
#####

# this state calculates the heading of the robot based on the robots last path
def adjust_heading_state( timer_event = None) :
	# Heading vector = (dx, dy)
	
	D.tank(0, 0) #stop the robot from moving while we determine its progress and the next state to call
	
	D.heading_vector = [D.cur_pos[0]-D.old_pos[0], D.cur_pos[1]-D.old_pos[1]]
	D.heading_angle = math.atan2(D.heading_vector[1], D.heading_vector[0])
	
	# tell the vision program our heading angle for it to display
	D.H_PUB.publish(str(D.heading_angle))

	# debugging artifacts
	#print 'heading vector : ', D.heading_angle	
	#print 'target vector : ', D.target_angle

	# normalize the target angle to {0, 2pi}
	while D.target_angle >= 2*3.14159 :
		D.target_angle = D.target_angle - 3.14159*2
	
	# normalize the heading angle
	while D.heading_angle >= 2*3.14159 :
		D.heading_angle -= 2*3.14159
	
	# if our vectors are in the correct order proceed as normal
	# (the target vector should be ahead of the heading vector)
	if(D.target_angle > D.heading_angle) :
		D.theta_diff = D.target_angle - D.heading_angle
		
		# if the angles are more than 180 degrees apart we go in the opposite direction
		if D.theta_diff > 3.14159 :
			D.theta_diff -= 2*3.14159
			
	# else subtract 2pi from the heading vector to maintain its position
	# but to put it behind the target vector numerically
	else :
		D.heading_angle -= 2*3.14159
		D.theta_diff = D.target_angle - D.heading_angle
		
		# if the angles are more than 180 degrees apart we go in the opposite direction
		if D.theta_diff > 3.14159 :
			D.theta_diff -= 2*3.14159
	
	# we set value of the angle that we need to rotate in order to align our heading with the target vector
	D.Turn_Angle = D.theta_diff
	
	#print 'theta_diff ', D.theta_diff  // checkpoint on the angle
	
	# now we make a decision on either moving forward or turning based on our heading vector and the target vector
	# if we are off course by more than 8 degrees (max_angle_error), we go into a rotation state based on the angle diff
	# between our current heading and desired heading
	if abs(D.Turn_Angle) >= D.MAX_ANGLE_ERROR :
		transition(0.1, turn_state)
		
	# otherwise we are facing within 8 degrees of our target and we go full speed ahead!
	else :
		D.speed = 200
		transition(0.1, move_state)



# this state moves the robot either forward or backward based on the D.speed global that is set by the calling function
def move_state( timer_event = None) :
	global D

	# save the current position into our old position variable, then after we move we will call adjust_heading_state 
	# and we will use our changed position to calculate the heading vector
	D.old_pos = [D.cur_pos[0], D.cur_pos[1]]
	
	# if we are in manual command mode, set our speed and than revert to start_state to wait for another command
	if D.Manual_Mode == True :
		D.tank(D.speed, D.speed)

	# if we are in vision control mode we move forward and then proceed to a state to calculate our robots heading.
	elif D.Vision_Mode == True :
		D.tank(D.speed, D.speed)
		transition( 0.6, adjust_heading_state)

		
		
		
# this state makes the robot turn in either direction based on the D.turn_angle global variable. 
##### -=TODO=- : Finish angle calibration, it is a little off currently
def turn_state( timer_event = None) :
	global D

	# if we are in manual mode, turn the desired pre-set angle and then go back to the start_state to wait for commands
	if D.Manual_Mode == True :
		if D.Turn_Angle > 0 :
			D.tank(-120, 120)
			D.speed = 0
			transition( 0.4, move_state )
		else :
			D.tank(120, -120)
			D.speed = 0
			transition( 0.4, move_state )

	elif D.Vision_Mode == True :
		time = (3*abs(D.Turn_Angle)+.02)/6.28
		if D.Turn_Angle > 0 :
			D.tank(100, -100)
			D.speed = 100
			transition( time, move_state )
		else :
			D.tank(-100, 100)
			D.speed = 100
			transition( time, move_state )


def start_state( timer_event = None ) :
	""" waits for start"""
	global D
	D.tank(0, 0) # hold still while we figure out our first move
	
	# if we have our data stream connected we can start
	if D.sdata != None :
		# if we are in vision mode, start off by going forward, then measure our heading to understand where we need to go
		if D.Vision_Mode == True :
			D.speed = 150
			transition( 0.1, move_state )
		
		elif D.Manual_Mode == True :
			return # wait for the keyboard callback function to call the next state

	# else we wait for the data stream to be connected by calling ourselves again
	else:
		transition( 0.1, start_state )


def win_state( timer_event = None) :
	print ' We have won!'
	D.speed = 0
	
	# if we win then we go into manual control mode, the user can then pick a different goal and press
	# 'v' to start up the vision control again
	D.Manual_Mode = True
	D.Vision_Mode = False
	transition(0.05, start_state)

#####
#####
##   Callback Functions for our data streams from the robot and other programs 
#####
#####

# gets robot position information from lab2.py program
def position_callback( data ) :
	s = data.data
	L = s.split()
	D.cur_pos[0] = [int(L[0]), int(L[1])]
	
	# if our position is within 30 pixels of our goal, we won!
	if abs(D.cur_pos[0] - D.goal[0]) < 30 and abs(D.cur_pos[1] - D.goal[1]) < 30 and D.Vision_Mode == True:
		transition(0.05, win_state)

# gets goal position information from lab2.py, the coordinates on the screen of the user defined goal
def goal_callback( data ) :
	s = data.data
	L = s.split()
	D.goal = [int(L[0]), int(L[1])]
	D.target_vector = [D.goal[0]-D.cur_pos[0], D.goal[1]-D.cur_pos[1]]
	D.target_angle = math.atan2(D.target_vector[1], D.target_vector[0])
		
# This function is called when the user manually presses a key to control the robot or switch states
def command_callback(data):

	message = data.data
	print "Command Received", message
	speed = 120

	# if the user wants to switch to command mode
	if message == 'c' :
		D.Manual_Mode = True
		D.Vision_Mode = False
		print 'Manual Mode Activated'
	# if the user wants to start vision control mode
	elif message == 'v' :
		D.Vision_Mode = True 
		D.Manual_Mode = False
		transition(.05, start_state)
		print 'Vision Command Mode Activated'


	# if we are in manual mode we will set the next state flags
	# then we will call the appropriate next state function
	if D.Manual_Mode == True :

		if message == 'w' :
			D.speed = +150
			transition(0.1, move_state)

		elif message == 'a' :
			D.Turn_Angle = 30
			transition(0.05, turn_state)

		elif message == 'd' :
			D.Turn_Angle = -30
			transition(0.05, turn_state)

		elif message == 's' :
			D.speed = -150
			D.Back = False
			transition(0.1, move_state)

		# space bar is the override stop key, it will stop the robot regardless of the current mode
		elif message == ' ' :
			D.speed = 0
			transition(0.1, move_state)




# this function receives the sensor data from the robot, it is used to stop us if we bump into something or we detect a cliff face
def sensor_callback( data ):
	D.sdata = data
	leftside = data.cliffLeftSignal
	rightside = data.cliffRightSignal
	left = data.cliffFrontLeftSignal
	right = data.cliffFrontRightSignal
	bumpleft = data.bumpLeft
	bumpright = data.bumpRight

	# wheeldrop? stop!
	if data.wheeldropCaster == True:
		D.STOP_FLAG = True

	if bumpleft == True or bumpright == True :
		D.BUMP_FLAG = True


#####
#####
##	Initializer function and main function. 
#####
#####

def init():
	global D # to hold system state
	

	# this subscribes to the stream
	# (1) it names the stream (stream_name)
	# (2) it indicates the type of message received (String)
	rospy.Subscriber( 'text_data', String, command_callback )

	# we need to give our program a ROS node name
	# the name is not important, so we use "lab1_node"
	rospy.init_node('lab1_node', anonymous=True)

	# we obtain the tank service
	rospy.wait_for_service('tank') # wait until the motors are available
	D.tank = rospy.ServiceProxy('tank', Tank) # D.tank is our "driver"
	
	# we obtain the song service
	rospy.wait_for_service('song') # wait until our voice is available
	D.song = rospy.ServiceProxy('song', Song) # D.song is our "speaker" 

	# set up a callback for the sensorPacket stream, i.e., "topic"
	rospy.Subscriber( 'sensorPacket', SensorPacket, sensor_callback )

	# this feed gives us a continuous stream of our coordinates
	rospy.Subscriber( 'visual_data', String, position_callback )

	#subscribe to the goal data stream, this will be called whenever we get a new goal sent to
	# us from the publisher in the kinnect program
	rospy.Subscriber('goal_data', String, goal_callback)

	D.H_PUB = rospy.Publisher('heading_data', String)


	  #these are overall modes of our program, if Manual mode is true than the program waits for movement
	  # commands from the user. If Vision_Mode is true than we use the kinnect to steer the robot
	D.Manual_Mode = True
	D.Vision_Mode = False


	# These Flags are set by the user when they use key presses to do manual commands
	# they are used by the fSM in the transition function to know what to do next
	D.Forward = False
	D.Back = False
	D.Right = False
	D.Left = False

	# Flags, these are used by our data subscriptions feeds to control program flow by letting the transistion function
	# know if it needs to do something special during the transition, like switch modes or stop, etc
	D.STOP_FLAG = False
	D.BUMP_FLAG = False

	## Values that need to be knowjn to do computation. These include the current position, old position, goal position, target and heading vectors
	# speed, turning angle, etc
	D.Speed = 150
	D.MAX_ANGLE_ERROR = 8.0*3.14/180.0   # maximum angl we can be off from the target and still consider ourselves 'on-course'
	D.MAX_DIST_ERROR = 20	# maximum distance we can be in pixels from the target before calling it a win
	D.Turn_Angle = 25
	D.theta_diff = 0
	D.old_pos = [0, 0]
	D.goal = [0, 0]
	D.cur_pos = [0, 0]
	D.target_vector = [0, 0]
	D.target_angle = 0
	D.heading_vector = [0, 0]
	D.heading_angle = 0


def main():
	global D
	# set up services and subscribe to data streams
	init()

	rospy.spin()

	# finish up...
	D.tank(0,0)
	print "Goodbye!"


if __name__ == "__main__":
   main()
