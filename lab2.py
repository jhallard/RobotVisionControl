#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_mudd')
import rospy
import cv_bridge
import cv
import math
import sensor_msgs.msg as sm
from std_msgs.msg import String


###################### GLOBAL SYSTEM STATE ########################

# class for a generic data holder
class Data:
    def __init__(self): pass    # empty constructor
    
# object (of class Data) to hold the global state of the system
D = Data()


#################### YOU NEED TO SET THESE! #######################

# are you using a Kinect? (alternatively, a webcam)
D.USE_KINECT = True

# do you want small windows (about 320 x 240)
D.half_size = False



#################### INITIALIZATION FUNCTIONS ######################

def init_globals():
    """ sets up the data we need in the global dictionary D
    """
    # get D so that we can change values in it
    global D

    D.goal = (0, 0)
    D.target_vector = (0, 0)
    D.heading_vector = 0.0
    
    # our publishers, D.K_PUB
    D.K_PUB = rospy.Publisher('text_data',String)
    D.V_PUB = rospy.Publisher('visual_data', String)
    D.G_PUB = rospy.Publisher('goal_data', String)

    rospy.Subscriber( 'heading_data', String, heading_callback )

    # put threshold values into D
    D.thresholds =  {'high_green': 141, 'low_green': 0, 'low_blue': 108, 'low_val': 106, 'high_hue': 204, 'high_val': 255, 'high_blue': 255, 'low_red': 160, 'low_hue': 113, 'high_red': 255, 'high_sat': 255, 'low_sat': 130}
    
    # Set up the windows containing the image from the camera,
    # the altered image, and the threshold sliders.
    cv.NamedWindow('image')
    cv.MoveWindow('image', 0, 0)
    
    cv.NamedWindow('threshold')
    THR_WIND_OFFSET = 640
    if D.half_size: THR_WIND_OFFSET /= 2
    cv.MoveWindow('threshold', THR_WIND_OFFSET, 0)
    
    cv.NamedWindow('sliders')
    SLD_WIND_OFFSET = 1280
    if D.half_size: SLD_WIND_OFFSET /= 2
    cv.MoveWindow('sliders', SLD_WIND_OFFSET, 0)
    

    # Create the sliders within the 'sliders' window
    cv.CreateTrackbar('low_red', 'sliders', D.thresholds['low_red'], 255, 
                          lambda x: change_slider('low_red', x) )
    cv.CreateTrackbar('high_red', 'sliders', D.thresholds['high_red'], 255, 
                          lambda x: change_slider('high_red', x) )
    cv.CreateTrackbar('low_green', 'sliders', D.thresholds['low_green'], 255, 
                          lambda x: change_slider('low_green', x) )
    cv.CreateTrackbar('high_green', 'sliders', D.thresholds['high_green'], 255, 
                          lambda x: change_slider('high_green', x) )
    cv.CreateTrackbar('low_blue', 'sliders', D.thresholds['low_blue'], 255, 
                          lambda x: change_slider('low_blue', x) )
    cv.CreateTrackbar('high_blue', 'sliders', D.thresholds['high_blue'], 255, 
                          lambda x: change_slider('high_blue', x) )
    cv.CreateTrackbar('low_hue', 'sliders', D.thresholds['low_hue'], 255, 
                          lambda x: change_slider('low_hue', x) )
    cv.CreateTrackbar('high_hue', 'sliders', D.thresholds['high_hue'], 255, 
                          lambda x: change_slider('high_hue', x) )
    cv.CreateTrackbar('low_sat', 'sliders', D.thresholds['low_sat'], 255, 
                          lambda x: change_slider('low_sat', x) )
    cv.CreateTrackbar('high_sat', 'sliders', D.thresholds['high_sat'], 255, 
                          lambda x: change_slider('high_sat', x) )
    cv.CreateTrackbar('low_val', 'sliders', D.thresholds['low_val'], 255, 
                          lambda x: change_slider('low_val', x) )
    cv.CreateTrackbar('high_val', 'sliders', D.thresholds['high_val'], 255, 
                          lambda x: change_slider('high_val', x) )

    # Set the method to handle mouse button presses
    cv.SetMouseCallback('image', onMouse, None)

    # We have not created our "scratchwork" images yet
    D.created_images = False

    # Variable for key presses
    D.last_key_pressed = 255

    # Create a connection to the Kinect
    D.bridge = cv_bridge.CvBridge()

    # camera for missile launcher
    D.camera = cv.CaptureFromCAM(-1)


def init_images():
    """ Creates all the images we'll need. Is separate from init_globals 
        since we need to know what size the images are before we can make
        them
    """
    # get D so that we can change values in it
    global D

    # Find the size of the image 
    # We set D.image right before calling this function
    D.size = cv.GetSize(D.image)

    # Create images for each color channel
    D.red = cv.CreateImage(D.size, 8, 1)
    D.blue = cv.CreateImage(D.size, 8, 1)
    D.green = cv.CreateImage(D.size, 8, 1)
    D.hue = cv.CreateImage(D.size, 8, 1)
    D.sat = cv.CreateImage(D.size, 8, 1)
    D.val = cv.CreateImage(D.size, 8, 1)

    # Create images to save the thresholded images to
    D.red_threshed = cv.CreateImage(D.size, 8, 1)
    D.green_threshed = cv.CreateImage(D.size, 8, 1)
    D.blue_threshed = cv.CreateImage(D.size, 8, 1)
    D.hue_threshed = cv.CreateImage(D.size, 8, 1)
    D.sat_threshed = cv.CreateImage(D.size, 8, 1)
    D.val_threshed = cv.CreateImage(D.size, 8, 1)

    # The final thresholded result
    D.threshed_image = cv.CreateImage(D.size, 8, 1)

    # Create an hsv image and a copy for contour-finding
    D.hsv = cv.CreateImage(D.size, 8, 3)
    D.copy = cv.CreateImage(D.size, 8, 1)
    D.storage = cv.CreateMemStorage(0) # Create memory storage for contours

    # bunch of keypress values
    # So we know what to show, depending on which key is pressed
    D.key_dictionary = {ord('w'): D.threshed_image,
                        ord('u'): D.red,
                        ord('i'): D.green,
                        ord('o'): D.blue,
                        ord('j'): D.red_threshed,
                        ord('k'): D.green_threshed,
                        ord('l'): D.blue_threshed,
                        ord('a'): D.hue,
                        ord('s'): D.sat,
                        ord('d'): D.val,
                        ord('z'): D.hue_threshed,
                        ord('x'): D.sat_threshed,
                        ord('c'): D.val_threshed,
                        }

    # set the default image for the second window
    D.current_threshold = D.threshed_image
  
################## END INITIALIZATION FUNCTIONS ####################
    

################### IMAGE PROCESSING FUNCTIONS #####################

def threshold_image():
    """ runs the image processing in order to create a 
        black and white thresholded image out of D.image
        into D.threshed_image.
    """
    # get D so that we can change values in it
    global D

    # Use OpenCV to split the image up into channels, saving them in gray images
    cv.Split(D.image, D.blue, D.green, D.red, None)

    # This line creates a hue-saturation-value image
    cv.CvtColor(D.image, D.hsv, cv.CV_RGB2HSV)
    cv.Split(D.hsv, D.hue, D.sat, D.val, None)

    # Here is how OpenCV thresholds the images based on the slider values:
    cv.InRangeS(D.red, D.thresholds["low_red"], D.thresholds["high_red"], D.red_threshed)
    cv.InRangeS(D.blue, D.thresholds["low_blue"], D.thresholds["high_blue"], D.blue_threshed)
    cv.InRangeS(D.green, D.thresholds["low_green"], D.thresholds["high_green"], D.green_threshed)
    cv.InRangeS(D.hue, D.thresholds["low_hue"], D.thresholds["high_hue"], D.hue_threshed)
    cv.InRangeS(D.sat, D.thresholds["low_sat"], D.thresholds["high_sat"], D.sat_threshed)
    cv.InRangeS(D.val, D.thresholds["low_val"], D.thresholds["high_val"], D.val_threshed)

    # Multiply all the thresholded images into one "output" image, D.threshed_image
    cv.Mul(D.red_threshed, D.green_threshed, D.threshed_image)
    cv.Mul(D.threshed_image, D.blue_threshed, D.threshed_image)
    cv.Mul(D.threshed_image, D.hue_threshed, D.threshed_image)
    cv.Mul(D.threshed_image, D.sat_threshed, D.threshed_image)
    cv.Mul(D.threshed_image, D.val_threshed, D.threshed_image)

    # Erode and Dilate shave off and add edge pixels respectively
    #cv.Erode(D.threshed_image, D.threshed_image, iterations = 1)
    #cv.Dilate(D.threshed_image, D.threshed_image, iterations = 1)


def find_biggest_region():
    """ finds all the contours in threshed image, finds the largest of those,
        and then marks in in the main image
    """
    # get D so that we can change values in it
    global D

    # Create a copy image of thresholds then find contours on that image
    cv.Copy( D.threshed_image, D.copy ) # copy threshed image

    # this is OpenCV's call to find all of the contours:
    contours = cv.FindContours(D.copy, D.storage, cv.CV_RETR_EXTERNAL, cv.CV_CHAIN_APPROX_SIMPLE)

    # Next we want to find the *largest* contour
    # this is the standard algorithm:
    #    walk the list of all contours, remembering the biggest so far:
    if len(contours) > 0:
        biggest = contours
        biggestArea = cv.ContourArea(contours)
        while contours != None:
            nextArea = cv.ContourArea(contours)
            if biggestArea < nextArea:
                biggest = contours
                biggestArea = nextArea
            contours = contours.h_next()
        
        # Use OpenCV to get a bounding rectangle for the largest contour
        br = cv.BoundingRect(biggest, update=0)

        # print the result of the cv.BoundingRect call...
        #print "br is", br

        # Draw a red box from (42,42) to (84,126), for now (you'll change this):
        upper_left = (br[0],br[1])
        lower_left = (br[0],br[1]+br[3])
        lower_right = (br[0]+br[2], br[1]+br[3])
        upper_right = (br[0]+br[2],br[1])
        cv.PolyLine(D.image, [[upper_left,lower_left,lower_right,upper_right]],
                    1, cv.RGB(255, 0, 0))

        # Draw the circle, at the image center for now (you'll change this)
        D.robot_loc = ((2*br[0]+br[2])/2, (2*br[1]+br[3])/2)
        cv.Circle(D.image, D.robot_loc, 10, cv.RGB(255, 255, 0), thickness=2,
                  lineType=8, shift=0)

        #draw the circle around the current goal that the user wants the robot to get to
        cv.Circle(D.image, D.goal, 10, cv.RGB(140, 100, 0), thickness=2,
                  lineType=8, shift=0)

        # calculate the target vector to get from our current location to the goal
        D.target_vector = (D.goal[0]-D.robot_loc[0], D.goal[1]-D.robot_loc[1])

        cv.Line(D.image, D.robot_loc, D.goal, cv.RGB(255, 0, 20), thickness = 1)

        temp = (int(D.robot_loc[0]+50*math.cos(D.heading_vector)), int(D.robot_loc[1]+50*math.sin(D.heading_vector)))
        cv.Line(D.image, D.robot_loc, temp, cv.RGB(255, 100, 20), thickness = 1)

        # Draw matching contours in white with inner ones in green
        cv.DrawContours(D.image, biggest, cv.RGB(255, 255, 255), 
                        cv.RGB(0, 255, 0), 1, thickness=2, lineType=8, 
                        offset=(0,0))

        D.G_PUB.publish(str(D.goal[0])+' '+str(D.goal[1]))
        D.V_PUB.publish(str(D.robot_loc[0])+' '+str(D.robot_loc[1]))


################# END IMAGE PROCESSING FUNCTIONS ###################

def heading_callback( data ) :
    s = data.data
    l = float(s)
    D.heading_vector = l



####################### CALLBACK FUNCTIONS #########################

def onMouse(event, x, y, flags, param):
    """ the method called when the mouse is clicked """
    global D

    # clicked the left button
    if event==cv.CV_EVENT_LBUTTONDOWN: 
        D.G_PUB.publish(str(x)+' '+str(y))
        center = (x, y)
        D.goal = center
        #print "x, y are", x, y, "    ",
        (b,g,r) = D.image[y,x]
        #print "r,g,b is", int(r), int(g), int(b), "    ",
        (h,s,v) = D.hsv[y,x]
        #print "h,s,v is", int(h), int(s), int(v)
        D.down_coord = (x,y)



def check_key_press(key_press):
    """ this handler is called when a real key press has been
        detected, and updates everything appropriately
    """
    # get D so that we can change values in it
    global D
    D.last_key_pressed = key_press

    # if it was ESC, make it 'q'
    if key_press == 27:
        key_press = ord('q')

    # publish the key press, if it's in range
    if ord(' ') <= key_press <= ord('z'):
        D.K_PUB.publish(String(str(chr(key_press))))

    # if a 'q' or ESC was pressed, we quit
    if key_press == ord('q'): 
        print "quitting"
        rospy.signal_shutdown( "Quit requested from keyboard" )
        return

    # help menu
    if key_press == ord('h'):
        print " Keyboard Command Menu"
        print " =============================="
        print " q    : quit"
        print " ESC  : quit"
        print " h    : help menu"
        print " w    : show total threshold image in threshold window"
        print " r    : show red image in threshold window"
        print " t    : show green image in threshold window"
        print " y    : show blue image in threshold window"
        print " f    : show thresholded red image in threshold window"
        print " g    : show thresholded blue image in threshold window"
        print " h    : show thresholded green image in threshold window"
        print " a    : show hue image in threshold window"
        print " s    : show saturation image in threshold window"
        print " d    : show value image in threshold window"
        print " z    : show thresholded hue image in threshold window"
        print " x    : show thresholded saturation image in threshold window"
        print " c    : show thresholded value image in threshold window"
        print " v    : saves threshold values to file (overwriting)"
        print " b    : loads threshold values from file"
        print " u    : mousedrags no longer set thresholds"
        print " i    : mousedrag set thresholds to area within drag"

    elif key_press == ord('v'):
        x = D.thresholds
        f = open( "./thresh.txt", "w" ) # open the file "thresh.txt" for writing
        print >> f, x # print x to the file object f
        f.close() # it's good to close the file afterwards
        print "(v) Wrote thresholds to thresh.txt. Use 'b' to load them."

    elif key_press == ord('b'):
        # should check if file exists!
        f = open( "./thresh.txt", "r" ) # open the file "thresh.txt" for reading
        data = f.read() # read everything from f into data
        x = eval( data ) # eval is Python's evaluation function
        # eval evaluates strings as if they were at the Python shell
        f.close() # its good to close the file afterwards
        print "(b) Loaded thresholds from thresh.txt. Use 'v' to save them."

        # Set threshold values in D
        D.thresholds = x

        # Update threshold values on actual sliders
        for i in ['low_red', 'high_red', 'low_green', 'high_green', 'low_blue', 'high_blue',
                      'low_hue', 'high_hue', 'low_sat', 'high_sat', 'low_val', 'high_val']:
            cv.SetTrackbarPos(i, 'sliders', D.thresholds[i])

    # threshold keypresses:
    elif key_press in D.key_dictionary.keys():
        D.current_threshold = D.key_dictionary[key_press]


# Function for changing the slider values
def change_slider(name, new_threshold):
    """ a small function to change a slider value """
    # get D so that we can change values in it
    global D
    D.thresholds[name] = new_threshold



def handle_image():
    """ this function organizes all of the processing
        done for each image from a camera or Kinect
    """
    # get D so that we can change values in it
    global D

    if D.orig_image == None: # did we get an image at all?
        print "No image"
        return

    # handle different image sizes
    if D.half_size == False: # keep the image size
        D.image = D.orig_image
    else: # halve the image size
        if D.created_images == False:
            w, h = D.orig_image.width, D.orig_image.height
            D.half_sz = (w/2,h/2)
            D.image = cv.CreateImage(D.half_sz, 8, 3)
        cv.Resize(D.orig_image,D.image)

    if D.created_images == False:   # have we set up the other images?
        init_images()               # Initialize the others needed
        D.created_images = True     # We only need to run this one time

    # Recalculate threshold image
    threshold_image()

    # Recalculate blob in main image
    find_biggest_region()

    # Get any incoming keypresses
    # To get input from keyboard, we use cv.WaitKey
    # Only the lowest eight bits matter (so we get rid of the rest):
    key_press_raw = cv.WaitKey(5) # gets a raw key press
    key_press = key_press_raw & 255 # sets all but the low 8 bits to 0
    
    # Handle key presses only if it's a real key (255 = "no key pressed")
    if key_press != 255:
        check_key_press(key_press)

    # Update the displays:
    # Main image:
    cv.ShowImage('image', D.image)

    # Currently selected threshold image:
    cv.ShowImage('threshold', D.current_threshold )



def handle_kinect_data(data):
    """ this function grabs images from the Kinect
    """
    # get D so that we can change values in it
    global D

    # Get the incoming image from the Kinect
    D.orig_image = D.bridge.imgmsg_to_cv(data, "bgr8")

    # now, handle that image...
    handle_image()
    


def handle_camera_data(data):
    """ this function grabs images from a webcamera
    """
    # get D so that we can change values in it
    global D

    while rospy.is_shutdown() == False:

        # Get the incoming image from the missle launcher or other camera
        D.orig_image = cv.QueryFrame(D.camera)

        # now, handle that image...
        handle_image()


##################### END CALLBACK FUNCTIONS #######################

        

############################## MAIN ################################

def main():
    """ the main program that sets everything up
    """
    global D
    # Initialize our node
    rospy.init_node('blobFinder')

    # Initialize all the global variables we will need
    init_globals()

    # Reroute appropriately...  (rospy.wait_for_message might work, but...)
    if D.USE_KINECT == True:
        print "Looking for a Kinect..."
        rospy.Subscriber('/camera/rgb/image_color', sm.Image, handle_kinect_data)
        rospy.spin()  # wait until a shutdown
    else: # using an ordinary camera
        print "No Kinect. Looking for a camera"
        handle_camera_data(0)  # this has the wait loop in it...



# this is the "main" trick - it tells Python
# what to run as a stand-alone script:
if __name__ == "__main__":
    main()