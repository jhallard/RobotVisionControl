#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_mudd')
import rospy
import cv
import sensor_msgs.msg as sm
from std_msgs.msg import String
from Tkinter import *

####
# kbd_publisher.py ~ showing off OpenCV's windowing and events
####


####
# main
####

def main():
    """ the main function that sets everything up
    """
    # Initialize this ROS node
    rospy.init_node('kbd_publisher') # any name will do

    # create our window - be sure the window has the focus!
    windows_init()

    # create a String publisher (pub)
    pub = rospy.Publisher('text_data',String)
    
    # main loop   
    while rospy.is_shutdown() == False:
        
        # Get incoming key events using cv.WaitKey (lowest 8 bits)
        key_code = cv.WaitKey(10) & 255 # gets the window's next key_code
        if key_code == 255: continue    # 255 means "no key pressed"
        key_press = chr(key_code)       # convert key_code to a character
        
        if ' ' <= key_press <= 'z': # if it's in our valid range
            print "Publishing ", str(key_press)
            pub.publish(String(str(key_press))) # publish!
            
        if key_code == 27 or key_press == 'q' : # if ESC or 'q' was pressed
            pub.publish(String('q')) # publish the string 'q' either way
            rospy.signal_shutdown( "Quitting..." )

    print "Bye!"



def windows_init():
    """ sets up all the windows we might want...
    """
    # Set up the windows using the OpenCV library
    cv.NamedWindow('events')            # create a window named 'events'
    cv.MoveWindow('events', 100, 100)   # move it to this screen location

    




####
# It all starts here...
#
# This is the "main" trick: it tells Python what code to run
# when you execute this file as a stand-alone script:
####

if __name__ == "__main__":
    main()