#!/home/unitree/Documents/python38_venv/bin/python

################################################################
##############    I N C L U D E S 
################################################################

#ZED
import pyzed.sl as sl

#ROS
import rospy
from geometry_msgs.msg import Twist

import cv2
import numpy as np
import requests




################################################################
##############    M A C R O S
################################################################

## Go1
POS_SIGN = 1
NEG_SIGN = -1
ANG_VEL_FOLLOWME_PERSON_JUSTAWAY_FROM_CENTRE = 0.9
ANG_VEL_FOLLOWME_PERSON_AT_CAMERA_BOUNDARY = 1.2
LNR_VEL_FOLLOWME_PERSON_VERY_FAR = 1.1
LNR_VEL_FOLLOWME_TOO_CLOSE = 0.3
LNR_VEL_FOLLOWME_OK_MAX = 1.0
LNR_VEL_FOLLOWME_OK_MIN = 0.3
LNR_VEL_FOLLOWME_GO_BACK = -0.2

ZERO_CMD_VEL = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}


## Follow me
FOLLOWME_SEARCHING_STATE_THRESHOLD = 7

#ZED
ZED_IMAGE_HEIGHT = 376
ZED_IMAGE_WIDTH = 672

## Object detection
OBJECT_DETECTION_ACCURACY_THRESHOLD = 40
OBJECT_DETECTION_ACCURACY_THRESHOLD_REASSIGN = 70
DIST_PERSON_CAMERA_TOBEKEPT = 80
DIST_PERSON_CAMERA_DIFF_THRESHOLD = 10
DIST_PERSON_CAMERA_TOO_CLOSE = 40
DIST_PERSON_CAMERA_VERY_FAR = 250
DIST_FROM_CAMERA_CENTRE_THRESHOLD = (int(ZED_IMAGE_WIDTH * 0.125))
DIST_FROM_CAMERA_CENTRE_TOO_FAR = (int(ZED_IMAGE_WIDTH * 0.34))
DIST_FROM_CAMERA_CENTRE_NEAR = (int(ZED_IMAGE_WIDTH * 0.0625))

POINT_X= 0
POINT_Y = 1
POINT_TOP_LEFT = 0
POINT_TOP_RIGHT = 1
POINT_BOTTOM_RIGHT = 2
POINT_BOTTOM_LEFT = 3

CENTRE_MAINTAINED = 0
CENTRE_ERR_DEBOUNCING = 1
CENTRE_CORRECTING = 2
CENTRE_ERR_DEBOUNCE_THRESHOLD = 2

TRACKING_ID_INI = 999

## OpenCV
COLOR_BLUE = [255,0,0]
COLOR_YELLOW = [0,255,255]
COLOR_GREEN = [0,255,0]
COLOR_RED = [0,0,255]
COLOR_ORANGE = [0,165,255]

OBJ_BB_THICKNESS = 2
DEFAULT_TEXT_SIZE = 1
DEFAULT_TEXT_COLOR = COLOR_BLUE
DEFAULT_TEXT_PIXEL = DEFAULT_TEXT_SIZE * 25 #Needed for adding text in images

ARROW_LENGTH = 50
ARROW_TIP = 0.5
ARROW_THICKNESS = 12

POS_IMAGE_TOP_LEFT_TEXT = [10,30]
POS_IMAGE_BOTTOM_RIGHT_TEXT_1 = [370,330]
POS_IMAGE_BOTTOM_RIGHT_TEXT_2 = [370,360]
POS_IMAGE_BOTTOM_RIGHT_ARROW = [500,250]





# ## LAB Testing values
# ANG_VEL_FOLLOWME_PERSON_JUSTAWAY_FROM_CENTRE = 0.3
# ANG_VEL_FOLLOWME_PERSON_AT_CAMERA_BOUNDARY = 0.7
# LNR_VEL_FOLLOWME_PERSON_VERY_FAR = 0.3
# LNR_VEL_FOLLOWME_TOO_CLOSE = 0.15
# LNR_VEL_FOLLOWME_OK_MAX = 0.2
# LNR_VEL_FOLLOWME_OK_MIN = 0.15
# LNR_VEL_FOLLOWME_GO_BACK = -0.111
# ## Object detection
# DIST_PERSON_CAMERA_TOBEKEPT = 80
# DIST_PERSON_CAMERA_DIFF_THRESHOLD = 10
# DIST_PERSON_CAMERA_TOO_CLOSE = 40
# DIST_PERSON_CAMERA_VERY_FAR = 150

################################################################
##############    C L A S S E S
################################################################

class FollowMe_Go1():

    def __init__(self):
        

        ######
        ## ROS INIT
        ######
        rospy.init_node('FollowMe_Go1', anonymous=False)
        self.followme_cmdvel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


        ######
        ## CAMERA INIT
        ######

        # Create a Camera object
        self.zed = sl.Camera()

        # Create a InitParameters object and set configuration parameters
        init_ZED_params = sl.InitParameters()
        # VGA : 672*376 (x2)
        # Available FPS for VGA: 15, 30, 60, 100
        init_ZED_params.camera_resolution = sl.RESOLUTION.VGA
        init_ZED_params.camera_fps = 30
        
        init_ZED_params.depth_mode = sl.DEPTH_MODE.NEURAL # Use ULTRA depth mode
        init_ZED_params.coordinate_units = sl.UNIT.CENTIMETER # Use millimeter units (for depth measurements)
        init_ZED_params.depth_minimum_distance = 20 
        init_ZED_params.depth_maximum_distance = 300 
        init_ZED_params.depth_stabilization = 30 

        # Open the camera
        zed_openerr = self.zed.open(init_ZED_params)
        if zed_openerr != sl.ERROR_CODE.SUCCESS:
            print("Camera Open Failed: "+repr(zed_openerr)+". Exit program.")
            exit()


        print("Object Detection: Loading Module...")
        obj_detctn_param = sl.ObjectDetectionParameters()
        obj_detctn_param.enable_tracking=True
        obj_detctn_param.enable_mask_output = True


        if obj_detctn_param.enable_tracking :
            positional_tracking_param = sl.PositionalTrackingParameters()
            self.zed.enable_positional_tracking(positional_tracking_param)

        obj_detctn_err = self.zed.enable_object_detection(obj_detctn_param)
        if obj_detctn_err != sl.ERROR_CODE.SUCCESS :
            print("Enable object detection Failed : "+repr(obj_detctn_err)+". Exit program.")
            self.zed.close()
            exit()



        ######
        ## VARIABLES
        ######


        self.person_depth = DIST_PERSON_CAMERA_TOBEKEPT

        self.image_height = self.zed.get_camera_information().camera_resolution.height
        self.image_width = self.zed.get_camera_information().camera_resolution.width
        print("ZED image height and width are " + str(self.image_height) + ", " + str(self.image_width))

        if(ZED_IMAGE_HEIGHT != self.image_height or ZED_IMAGE_WIDTH != self.image_width):
            print("Error! Mismatch between zed camera dimensions and code macros. Correct the macros ZED_IMAGE_HEIGHT and ZED_IMAGE_WIDTH .")
            self.zed.close()
            exit()


        self.image_middle_bottom_line = (int(self.image_width/2), self.image_height)
        self.image_vertical_centre_xpoint = int(self.image_width/2)

        self.camera_raw_op = np.zeros((self.image_height,self.image_width))
        self.camera_depth_op = ''
        self.camera_depth_map = ''

        self.state = 'INIT'
        self.prev_movement = 'NONE'
        self.person_detected = False
        self.person_tracked_id = TRACKING_ID_INI
        self.searching_state_cntr = 0

        self.centre_deviation_flag = CENTRE_MAINTAINED
        self.centre_deviation_cntr = 0


    def capture_camera(self):

        # Set object detection runtime parameters after opening the camera    
        obj_detctn_runtime_param = sl.ObjectDetectionRuntimeParameters()
        obj_detctn_runtime_param.detection_confidence_threshold = OBJECT_DETECTION_ACCURACY_THRESHOLD
        obj_detctn_runtime_param.object_class_filter = [sl.OBJECT_CLASS.PERSON]  # Only detect Persons


        # Set runtime parameters after opening the camera
        zed_runtime_param = sl.RuntimeParameters()
        zed_runtime_param.sensing_mode = sl.SENSING_MODE.STANDARD # Preserves edges and depth accuracy

        ## Safe value for person detected. Only set true, if the frame is read properly and person detected
        self.person_detected = False

        if self.zed.grab(zed_runtime_param) == sl.ERROR_CODE.SUCCESS:

            # Retrieving information
            read_image_temp = sl.Mat() 
            self.zed.retrieve_image(read_image_temp, sl.VIEW.LEFT) # Retrieve left image
            self.camera_raw_op = read_image_temp.get_data() # Convert sl.Mat to cv2.Mat

            read_depth_temp = sl.Mat()
            self.zed.retrieve_image(read_depth_temp, sl.VIEW.DEPTH) # Retrieve depth image
            self.camera_depth_op = read_depth_temp.get_data() # Convert sl.Mat to cv2.Mat

            read_depth_map_temp = sl.Mat()
            self.zed.retrieve_measure(read_depth_map_temp, sl.MEASURE.DEPTH) # Retrieve depth map
            self.camera_depth_map = read_depth_map_temp.get_data() # Convert sl.Mat to cv2.Mat

            
            objects_detected = sl.Objects()
            self.zed.retrieve_objects(objects_detected, obj_detctn_runtime_param)



            ## Show state
            self.camera_raw_op = addOpenCVText(self.camera_raw_op, self.state , POS_IMAGE_TOP_LEFT_TEXT, color_ip=COLOR_GREEN)


            ## Analyzing the objects and getting relevant informations
            if objects_detected.is_new :


                objects_detected_array = objects_detected.object_list
                print("\n\n\nObject(s) detected = " + str(len(objects_detected_array)))
         

                if len(objects_detected_array) > 0 :
                    

                    ## Temporary initialization
                    object_being_tracked = objects_detected_array[0]


                    print("\n\n " + str(len(objects_detected_array))+" Object(s) detected\n")

                    ## Checking if the tracked ID is present in the list
                    for object_detected in objects_detected_array:
                        print("Object attributes:")
                        print(" Label '"+repr(object_detected.label)+"' (conf. "+str(int(object_detected.confidence))+"/100)")
                        print(" Tracking ID: "+str(int(object_detected.id))+" tracking state: "+repr(object_detected.tracking_state)+" / "+repr(object_detected.action_state))
                       

                        temp_obj_bb2d = object_being_tracked.bounding_box_2d
                        temp_obj_bb2d = temp_obj_bb2d.astype(int)
                        ## Define points for easier access in drawing images
                        self.BB_CORNER_TOP_RIGHT_TEXT = (temp_obj_bb2d[POINT_TOP_RIGHT][POINT_X], temp_obj_bb2d[POINT_TOP_RIGHT][POINT_Y] + DEFAULT_TEXT_PIXEL)
                        idText = repr(object_detected.label) + " ID: "+ str(int(object_detected.id)) + " " + str(int(object_detected.confidence)) + "%"
                        self.camera_raw_op = addOpenCVText(self.camera_raw_op, idText ,self.BB_CORNER_TOP_RIGHT_TEXT)

                        if(self.person_tracked_id == int(object_detected.id)):
                            self.person_detected = True
                            object_being_tracked = object_detected

                    ## If not present, assign a new id only if there is only one object being detected and confidence is good
                    if(self.person_detected != True):
                        if (len(objects_detected_array) == 1) :
                            object_being_tracked = objects_detected_array[0]
                            if(int(object_being_tracked.confidence) > OBJECT_DETECTION_ACCURACY_THRESHOLD_REASSIGN):
                                self.person_tracked_id  = int(object_being_tracked.id)
                                self.person_detected = True
                            else:
                                print("The person cannot be tracked as confidence of detection is less!")
                                self.camera_raw_op = addOpenCVTextAtCentre(self.camera_raw_op, "DETECTION CONFIDENCE IS LESS", color_ip=COLOR_RED)
  
                        else:
                            print("The person cannot be tracked as many objects (PEOPLE) being detected!")
                            self.camera_raw_op = addOpenCVTextAtCentre(self.camera_raw_op, "TOO MANY DETECTIONS", color_ip=COLOR_RED)


                    ### Proceed with detected person
                    if(self.person_detected == True):

                        # Read the bounding box 2D
                        self.obj_bb2d = object_being_tracked.bounding_box_2d
                        self.obj_bb2d = self.obj_bb2d.astype(int)
                        print('Bounding Box : ' + ' '.join(map(str, self.obj_bb2d)))


                        ## Define points for easier access in drawing images
                        self.BB_CORNER_TOP_RIGHT_TEXT = (self.obj_bb2d[POINT_TOP_RIGHT][POINT_X], self.obj_bb2d[POINT_TOP_RIGHT][POINT_Y] + DEFAULT_TEXT_PIXEL)
                        self.BB_MIDDLE_BOTTOM_LINE =  ( self.obj_bb2d[POINT_BOTTOM_LEFT][POINT_X] + int((self.obj_bb2d[POINT_BOTTOM_RIGHT][POINT_X] - self.obj_bb2d[POINT_BOTTOM_LEFT][POINT_X])/2) , self.obj_bb2d[POINT_BOTTOM_RIGHT][POINT_Y])
                        

                        # Draw box
                        self.camera_raw_op = cv2.rectangle(self.camera_raw_op,
                                                        [self.obj_bb2d[POINT_TOP_LEFT][POINT_X] , self.obj_bb2d[POINT_TOP_LEFT][POINT_Y] ],
                                                        [self.obj_bb2d[POINT_BOTTOM_RIGHT][POINT_X] , self.obj_bb2d[POINT_BOTTOM_RIGHT][POINT_Y] ],
                                                        COLOR_BLUE, 
                                                        OBJ_BB_THICKNESS                                                       
                                                        )
                        
                        idText = repr(object_being_tracked.label) + " ID: "+ str(int(object_being_tracked.id)) + " " + str(int(object_being_tracked.confidence)) + "%"
                        self.camera_raw_op = addOpenCVText(self.camera_raw_op, idText ,self.BB_CORNER_TOP_RIGHT_TEXT)

                        # Make sure the mask is available for detected person
                        if object_being_tracked.mask.is_init():
                            
                            # Calcualte the depth value
                            self.person_depth, depth_map_masked  = self.process_depth(object_being_tracked.mask.get_data())

                        ## Draw depth line and mention depth
                        depthText = str(self.person_depth) + "cm"
                        print("Person Depth : ", depthText)
                        self.camera_raw_op = addOpenCVLine(self.camera_raw_op, self.image_middle_bottom_line, self.BB_MIDDLE_BOTTOM_LINE, color_ip=COLOR_RED)                    
                        self.camera_raw_op = addOpenCVText(self.camera_raw_op, depthText, position=(
                                        int((self.image_middle_bottom_line[0]+ self.BB_MIDDLE_BOTTOM_LINE[0])/2),
                                        int((self.image_middle_bottom_line[1]+ self.BB_MIDDLE_BOTTOM_LINE[1])/2)),
                                        color_ip=COLOR_RED
                                        )
                        


                        # Calculate centre point of camera and draw the lines for reference
                        print("Camera centre : ",self.image_vertical_centre_xpoint)
                        self.camera_raw_op = addOpenCVLine(self.camera_raw_op, (self.image_vertical_centre_xpoint,0), (self.image_vertical_centre_xpoint,self.image_height), color_ip=COLOR_YELLOW)
                        self.camera_raw_op = addOpenCVLine(self.camera_raw_op, (self.image_vertical_centre_xpoint-DIST_FROM_CAMERA_CENTRE_THRESHOLD,0), (self.image_vertical_centre_xpoint-DIST_FROM_CAMERA_CENTRE_THRESHOLD,self.image_height), color_ip=COLOR_YELLOW)
                        self.camera_raw_op = addOpenCVLine(self.camera_raw_op, (self.image_vertical_centre_xpoint+DIST_FROM_CAMERA_CENTRE_THRESHOLD,0), (self.image_vertical_centre_xpoint+DIST_FROM_CAMERA_CENTRE_THRESHOLD,self.image_height), color_ip=COLOR_YELLOW)
                        self.camera_raw_op = addOpenCVLine(self.camera_raw_op, (self.image_vertical_centre_xpoint-DIST_FROM_CAMERA_CENTRE_TOO_FAR,0), (self.image_vertical_centre_xpoint-DIST_FROM_CAMERA_CENTRE_TOO_FAR,self.image_height), color_ip=COLOR_ORANGE)
                        self.camera_raw_op = addOpenCVLine(self.camera_raw_op, (self.image_vertical_centre_xpoint+DIST_FROM_CAMERA_CENTRE_TOO_FAR,0), (self.image_vertical_centre_xpoint+DIST_FROM_CAMERA_CENTRE_TOO_FAR,self.image_height), color_ip=COLOR_ORANGE)
                    
                        # Calculate centre point of the detected person
                        print("Person centre : ",self.BB_MIDDLE_BOTTOM_LINE[POINT_X])
                        self.camera_raw_op = addOpenCVLine(self.camera_raw_op, (self.BB_MIDDLE_BOTTOM_LINE[POINT_X],0), (self.BB_MIDDLE_BOTTOM_LINE[POINT_X],self.image_height), color_ip=COLOR_GREEN)

                else:
                    ## No objects detected
                    self.camera_raw_op = addOpenCVTextAtCentre(self.camera_raw_op, "NO PERSON DETECTED", color_ip=COLOR_RED)

        else:
            print("\nZED Grab function failed.\n")

        

        

    def process_depth(self, mask_data ):


        depth_map_complete = np.zeros((self.image_height,self.image_width))
        # Place the max at the bounding box location in the empty image
        depth_map_complete[self.obj_bb2d[POINT_TOP_LEFT][POINT_Y]: self.obj_bb2d[POINT_BOTTOM_RIGHT][POINT_Y] , 
                           self.obj_bb2d[POINT_TOP_LEFT][POINT_X]: self.obj_bb2d[POINT_BOTTOM_RIGHT][POINT_X] ] = mask_data


        ## Replace infinite values
        self.camera_depth_map[np.isinf(self.camera_depth_map)] = 0

        # Find the indices where the array has 255s. Mask contains 255 where the object is present, rest 0
        indices_255 = np.where(depth_map_complete == 255)

        # Extract depth values of object from the depth map corresponding to the indices
        obj_depth_values = self.camera_depth_map[indices_255]
        # Calculate the mean of the array
        obj_detected_depth_mean = np.mean(obj_depth_values)      

        return obj_detected_depth_mean, depth_map_complete


    def starting(self):

        print("Initializing the follow me ")

        ##Init the coutners and necessary state
        self.searching_state_cntr = 0
        self.centre_deviation_flag = CENTRE_MAINTAINED
        self.centre_deviation_cntr = 0

        # Check if human is detected
        if(self.person_detected):
                
                    
            # Check if the human is almost at the centre

                # Check if they are close
                if(abs(self.image_vertical_centre_xpoint - self.BB_MIDDLE_BOTTOM_LINE[POINT_X]) < DIST_FROM_CAMERA_CENTRE_THRESHOLD) :
                    # If yes, change the state to following
                    self.state = 'FOLLOWING'
                    print("Its near")

                    cv2.waitKey(5)
                
                else:
                    print("Not near the centre")
                    self.camera_raw_op = addOpenCVTextAtCentre(self.camera_raw_op, "MOVE CLOSER TO CENTRE", color_ip=COLOR_RED)
        
        else:
            print("Person not detected...")


        

    def following(self):

        ## Declare the cmd_vel variable
        followme_cmd_vel = Twist()
        # Initialize all velocities to zero
        followme_cmd_vel.linear.x = 0.0
        followme_cmd_vel.linear.y = 0.0
        followme_cmd_vel.linear.z = 0.0
        followme_cmd_vel.angular.x = 0.0
        followme_cmd_vel.angular.y = 0.0
        followme_cmd_vel.angular.z = 0.0



        # Check if human is detected
        if(self.person_detected):

            print("Following the person ")

            # Calculate the depth difference from distance to be kept and current depth
            depth_diff_flag = False
            depth_diff = self.person_depth - DIST_PERSON_CAMERA_TOBEKEPT
            
            if(abs(self.person_depth - DIST_PERSON_CAMERA_TOBEKEPT) > DIST_PERSON_CAMERA_DIFF_THRESHOLD):
                depth_diff_flag = True
            print("Depth Flag : ", depth_diff_flag, " Depth Difference : ", depth_diff)


            # Calculate the angle difference from distance to be kept and current distance from centre
            centre_deviation = self.image_vertical_centre_xpoint - self.BB_MIDDLE_BOTTOM_LINE[POINT_X]

            if((abs(self.image_vertical_centre_xpoint - self.BB_MIDDLE_BOTTOM_LINE[POINT_X]) > DIST_FROM_CAMERA_CENTRE_THRESHOLD)
               and self.centre_deviation_flag != CENTRE_CORRECTING):
                self.centre_deviation_flag = CENTRE_ERR_DEBOUNCING
                self.centre_deviation_cntr += 1

                ## If too far, no need of debouncing
                if(abs(centre_deviation) > DIST_FROM_CAMERA_CENTRE_TOO_FAR):
                    self.centre_deviation_flag = CENTRE_CORRECTING
                    self.centre_deviation_cntr = CENTRE_ERR_DEBOUNCE_THRESHOLD
            else:
                self.centre_deviation_cntr = 0



            print("Centre Deviation Flag : ", self.centre_deviation_flag, " Centre Deviation : ", centre_deviation, " Counter : ",self.centre_deviation_cntr)




            if(depth_diff_flag):
                # If difference is greater than threshold, move forward
                if(depth_diff > 0):

                    if(self.person_depth > DIST_PERSON_CAMERA_VERY_FAR):
                        print("Moving forward fast as person is far away")
                        followme_cmd_vel.linear.x = POS_SIGN * LNR_VEL_FOLLOWME_PERSON_VERY_FAR
                        self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'up', color_ip=COLOR_ORANGE )

                    else:
                        print("Moving forward")
                        speed_slope = (LNR_VEL_FOLLOWME_OK_MAX - LNR_VEL_FOLLOWME_OK_MIN)/(DIST_PERSON_CAMERA_VERY_FAR - DIST_PERSON_CAMERA_TOBEKEPT)
                        followme_cmd_vel.linear.x = depth_diff * speed_slope + LNR_VEL_FOLLOWME_OK_MIN

                        print("Slope = " + str(speed_slope) + " linear speed calcualted " + str(followme_cmd_vel.linear.x))

                        self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'up' )
            
                # If less, move backward
                if(depth_diff < 0):

                    #####
                    # Check if the person is too close to the robot
                    #####
                    if(self.person_depth <= DIST_PERSON_CAMERA_TOO_CLOSE):
                        # If less, move backward
                        print("Person is too close. Moving backward")
                        followme_cmd_vel.linear.x = NEG_SIGN * LNR_VEL_FOLLOWME_TOO_CLOSE
                        self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'down', color_ip=COLOR_ORANGE)

                    else:
                        print("Moving backward")
                        followme_cmd_vel.linear.x = LNR_VEL_FOLLOWME_GO_BACK
                        self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'down' )

            else:
                print("Depth Maintained")
                followme_cmd_vel.linear.x = 0.0



            ## Only move left/right if the debounce threshold is reached
            if(self.centre_deviation_flag == CENTRE_CORRECTING):

                #####
                # Check if the person is too away from centre
                #####
                if(abs(centre_deviation) > DIST_FROM_CAMERA_CENTRE_TOO_FAR):

                    # If difference is greater than 0, move right
                    if(centre_deviation > 0):
                        print("Person moved too much to my Right")
                        ## Only turn right
                        followme_cmd_vel.angular.z = POS_SIGN * ANG_VEL_FOLLOWME_PERSON_AT_CAMERA_BOUNDARY
                        self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'right', color_ip=COLOR_ORANGE )
                        self.prev_movement = 'RIGHT'
                
                    # If less, move left
                    if(centre_deviation < 0):
                        print("Person moved too much to my Left")
                        ## Only turn left
                        followme_cmd_vel.angular.z = NEG_SIGN * ANG_VEL_FOLLOWME_PERSON_AT_CAMERA_BOUNDARY 
                        self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'left', color_ip=COLOR_ORANGE )
                        self.prev_movement = 'LEFT'

                else:
                    # If difference is greater than threshold, move right
                    if(centre_deviation > 0):
                        print("Person moved to my Right")
                        followme_cmd_vel.angular.z = POS_SIGN * ANG_VEL_FOLLOWME_PERSON_JUSTAWAY_FROM_CENTRE
                        self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'right' )
                        self.prev_movement = 'RIGHT'
                
                    # If less, move left
                    if(centre_deviation < 0):
                        print("Person moved to my Left")
                        followme_cmd_vel.angular.z = NEG_SIGN * ANG_VEL_FOLLOWME_PERSON_JUSTAWAY_FROM_CENTRE
                        self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'left' )
                        self.prev_movement = 'LEFT'


                ## Resetting of aligning to centre
                if(abs(centre_deviation) < DIST_FROM_CAMERA_CENTRE_NEAR):
                    self.centre_deviation_flag = CENTRE_MAINTAINED
                    self.centre_deviation_cntr = 0
                    print("Centre is maintained")

            else:
                print("Centre Maintained")
                followme_cmd_vel.angular.z = 0.0
                self.prev_movement = 'NO_TURN'

                if(self.centre_deviation_cntr >= CENTRE_ERR_DEBOUNCE_THRESHOLD):
                    self.centre_deviation_flag = CENTRE_CORRECTING
                elif(self.centre_deviation_cntr == 0):
                    self.centre_deviation_flag = CENTRE_MAINTAINED


        else:
            # Reset as no person seen
            # Not explicitly resetting as its already zero as init value
            print("Switching to searching state")
            self.state = 'SEARCHING'

        ## Publish cmd vel

        print(self.followme_cmdvel_pub)
        self.camera_raw_op = addOpenCVText(self.camera_raw_op, "Linear x  : " + str(followme_cmd_vel.linear.x)  , POS_IMAGE_BOTTOM_RIGHT_TEXT_1, color_ip=COLOR_GREEN)
        self.camera_raw_op = addOpenCVText(self.camera_raw_op, "Angular z : " + str(followme_cmd_vel.angular.z)  , POS_IMAGE_BOTTOM_RIGHT_TEXT_2, color_ip=COLOR_GREEN)
  

        self.followme_cmdvel_pub.publish(followme_cmd_vel)            



    def searching(self):
        print("Searching for the person ")

        ## Declare the cmd_vel variable
        followme_cmd_vel = Twist()
        # Initialize all velocities to zero
        followme_cmd_vel.linear.x = 0.0
        followme_cmd_vel.linear.y = 0.0
        followme_cmd_vel.linear.z = 0.0
        followme_cmd_vel.angular.x = 0.0
        followme_cmd_vel.angular.y = 0.0
        followme_cmd_vel.angular.z = 0.0


        print("Searching state counter ", self.searching_state_cntr)



        self.searching_state_cntr += 1
        if(self.searching_state_cntr <= FOLLOWME_SEARCHING_STATE_THRESHOLD):

            if(self.person_detected == True):
                self.searching_state_cntr = 0
                self.state = 'FOLLOWING'
                print("Person detected during searching. Going to following state")
                self.camera_raw_op = addOpenCVTextAtCentre(self.camera_raw_op, "DETECTED", color_ip=COLOR_RED)

            else:

                ## Look for previous state
                if(self.prev_movement == 'RIGHT'):
                    print("Searching for the person in the right")
                    followme_cmd_vel.angular.z = POS_SIGN * ANG_VEL_FOLLOWME_PERSON_AT_CAMERA_BOUNDARY
                    self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'right', color_ip=COLOR_ORANGE )
                    self.prev_movement = 'RIGHT'
                
                elif(self.prev_movement == 'LEFT'):
                    print("Searching for the person in the left")
                    followme_cmd_vel.angular.z = NEG_SIGN * ANG_VEL_FOLLOWME_PERSON_AT_CAMERA_BOUNDARY 
                    self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'left', color_ip=COLOR_ORANGE )
                    self.prev_movement = 'LEFT'

                else:
                    print("Not moving in Searching state as there was no possible movement")
                    followme_cmd_vel.angular.z = 0.0
                    self.prev_movement = 'NO_TURN'

            
        else:
            self.state = 'INIT'
            self.searching_state_cntr = 0
            print("Timed out! Going back to init state as no person detected during searching.")




        ## Publish cmd vel

        print(self.followme_cmdvel_pub)
        self.camera_raw_op = addOpenCVText(self.camera_raw_op, "Linear x  : " + str(followme_cmd_vel.linear.x)  , POS_IMAGE_BOTTOM_RIGHT_TEXT_1, color_ip=COLOR_GREEN)
        self.camera_raw_op = addOpenCVText(self.camera_raw_op, "Angular z : " + str(followme_cmd_vel.angular.z)  , POS_IMAGE_BOTTOM_RIGHT_TEXT_2, color_ip=COLOR_GREEN)
  

        self.followme_cmdvel_pub.publish(followme_cmd_vel)       


    def followme_run(self):

        print("Executing run")
        key=''

        while key!=113:
            ## Read the camera first
            self.capture_camera()

            if self.state == 'INIT':
                self.starting()

            elif self.state == 'FOLLOWING':
                self.following()

            if self.state == 'SEARCHING':
                self.searching()


            
            cv2.imshow("Camera", self.camera_raw_op) #Display image
            #cv2.imshow("Depth", self.camera_depth_op)
            key = cv2.waitKey(1)


                        
            # # Encode the frame as JPEG
            # _, jpeg = cv2.imencode('.jpg', self.camera_raw_op)
            # frame_bytes = jpeg.tobytes()

            # # Send the frame to the Flask server
            # try:
            #     response = requests.post('http://192.168.12.65:5000/update_frame', data=frame_bytes)
            #     if response.status_code != 200:
            #         print("Failed to send frame to server")
            # except requests.exceptions.RequestException as e:
            #     print("Error sending frame to server:", e)

            
            # try:
            #     response = requests.get('http://192.168.12.65:5000/get_stopcmd_value')
            #     data = response.json()
            #     print(data)

            #     if(data.get('stop_request_cmd') == 'STOP'):           
            #         key = 113
            #         print("STOP received ")
            #         response = requests.post('http://192.168.12.65:5000/update_stopcmd_value', data={'stop_request_cmd': 'Init'})

            # except requests.ConnectionError:
            #     print("")
            #     # print("Connection error: Failed to connect to the server:",requests.ConnectionError)	

            
            if(key == 113 ):
                print("\n\n\n\nEXITING!!!!!!!!!\n\n\n")


        ## Declare the cmd_vel variable
        followme_cmd_vel = Twist()
        # Initialize all velocities to zero
        followme_cmd_vel.linear.x = 0.0
        followme_cmd_vel.linear.y = 0.0
        followme_cmd_vel.linear.z = 0.0
        followme_cmd_vel.angular.x = 0.0
        followme_cmd_vel.angular.y = 0.0
        followme_cmd_vel.angular.z = 0.0
        ## Publish cmd vel
        self.followme_cmdvel_pub.publish(followme_cmd_vel)
                


        
        cv2.destroyAllWindows()
        # Close the camera
        self.zed.disable_object_detection()
        self.zed.close()





################################################################
##############    H E L P E R   F U N C T I O N S
################################################################
def addOpenCVText(image,text_ip,position = (200, 200), fontScale_ip = DEFAULT_TEXT_SIZE, color_ip = DEFAULT_TEXT_COLOR ):
    new_imagewithText = cv2.putText(    img = image,
                                        text = text_ip,
                                        org = position,
                                        fontFace = cv2.FONT_HERSHEY_SIMPLEX,
                                        fontScale = fontScale_ip,
                                        color = color_ip,
                                        thickness = 2
                                    )
    return new_imagewithText

def addOpenCVTextAtCentre(image,text_ip,fontScale_ip = DEFAULT_TEXT_SIZE, color_ip = DEFAULT_TEXT_COLOR ):

    textsize = len(text_ip)

    y = int(ZED_IMAGE_HEIGHT/2)
    x = int(ZED_IMAGE_WIDTH/2) - int(textsize/2) * (DEFAULT_TEXT_PIXEL - 5)

    new_imagewithText = cv2.putText(    img = image,
                                        text = text_ip,
                                        org = (x,y),
                                        fontFace = cv2.FONT_HERSHEY_SIMPLEX,
                                        fontScale = fontScale_ip,
                                        color = color_ip,
                                        thickness = 2
                                    )
    return new_imagewithText


def addOpenCVLine(image, start_pos = (0, 0), end_pos = (100, 100), color_ip = DEFAULT_TEXT_COLOR ):
    new_imagewithLine = cv2.line(   image, start_pos, end_pos,
                                    color = color_ip,
                                    thickness = 2
                                    )    
    return new_imagewithLine


def addOpenCVArrow(image, start_pos = [0, 0], direction = 'up', color_ip = DEFAULT_TEXT_COLOR ):

    end_pos = start_pos

    if(direction == 'up'):
        end_pos = [start_pos[POINT_X], start_pos[POINT_Y] - ARROW_LENGTH]

    elif(direction == 'down'):
        end_pos = [start_pos[POINT_X], start_pos[POINT_Y] + ARROW_LENGTH]

    elif(direction == 'right'):
        end_pos = [start_pos[POINT_X]- ARROW_LENGTH, start_pos[POINT_Y]]

    elif(direction == 'left'):
        end_pos = [start_pos[POINT_X]+ ARROW_LENGTH, start_pos[POINT_Y]]



    new_imagewithArrow = cv2.arrowedLine(   image, start_pos, end_pos,
                                    color = color_ip,
                                    thickness = ARROW_THICKNESS,
                                    tipLength = ARROW_TIP
                                    )    
    return new_imagewithArrow

################################################################
##############    M A I N
################################################################


if __name__ == "__main__":
    

    print("Started..")
    followme_go1 = FollowMe_Go1()

    followme_go1.followme_run()







