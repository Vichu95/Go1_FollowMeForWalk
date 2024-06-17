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
from time import sleep
import math


#Folow me
from go1_followme.msg import followme_state




################################################################
##############    M A C R O S
################################################################


#ZED
ZED_IMAGE_HEIGHT = 376
ZED_IMAGE_WIDTH = 672
ZED_RATIO_DEPTH_PIXEL = 0.261 # Check calibrating function #todo

## Go1
POS_SIGN = 1
NEG_SIGN = -1



## Object detection
OBJECT_DETECTION_ACCURACY_THRESHOLD = 55
OBJECT_DETECTION_ACCURACY_THRESHOLD_REASSIGN = 60

DIST_PERSON_CAMERA_TOBEKEPT = 85
DIST_PERSON_CAMERA_TOBEKEPT_PIXEL = 310 #Found out as an average value of pixel for 70cm depth by running the code [70,265] [60,280] [85,310]
DIST_PERSON_CAMERA_VALID_THRESHOLD = 5
DIST_PERSON_CAMERA_DIFF_THRESHOLD = 10
DIST_PERSON_CAMERA_VERY_FAR = 130
DIST_PERSON_CAMERA_TOO_CLOSE = 50
DIST_PERSON_CAMERA_NO_MOVE_LEFTTURN = 60 #todo remove
DIST_PERSON_CAMERA_INIT_HIGH_VALUE = 300

DIST_FROM_CAMERA_CENTRE_TOO_FAR = (int(ZED_IMAGE_WIDTH * 0.5))
DIST_FROM_CAMERA_CENTRE_THRESHOLD = (int(ZED_IMAGE_WIDTH * 0.075))
DIST_FROM_CAMERA_CENTRE_NEAR = (int(ZED_IMAGE_WIDTH * 0.03125))
DIST_FROM_CAMERA_CENTRE_TURN_AND_MOVE = (int(ZED_IMAGE_WIDTH * 0.3))
DIST_FROM_CAMERA_CENTRE_TO_CALC_TURN = (int(ZED_IMAGE_WIDTH * 0.50))

TURN_FROM_AXIS_THRESHOLD = math.radians(8)
TURN_FROM_AXIS_VERY_FAR = math.radians(25)
TURN_FROM_AXIS_NEAR = math.radians(5)

POINT_X= 0
POINT_Y = 1
POINT_TOP_LEFT = 0
POINT_TOP_RIGHT = 1
POINT_BOTTOM_RIGHT = 2
POINT_BOTTOM_LEFT = 3

DIFF_MAINTAINED = 0
DIFF_ERR_DEBOUNCING = 1
DIFF_CORRECTING = 2
CENTRE_ERR_DEBOUNCE_THRESHOLD = 2
DEPTH_ERR_DEBOUNCE_THRESHOLD = 3
TURN_ERR_DEBOUNCE_THRESHOLD = 2

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
TEXT_SIZE_OBJ_INFO = 0.5
TEXT_SIZE_CMD_VEL = 0.7
TEXT_SIZE_POS = 0.7
TRANSPARENCY_ALPHA_GRAPHS = 0.3

ARROW_LENGTH = 50
ARROW_TIP = 0.5
ARROW_THICKNESS = 12

POS_IMAGE_TOP_LEFT_TEXT = [10,30]
POS_IMAGE_BOTTOM_RIGHT_TEXT_1 = [370,330]
POS_IMAGE_BOTTOM_RIGHT_TEXT_2 = [370,360]
POS_IMAGE_TOP_RIGHT_TEXT_1_SIZE0_7 = [450,20]
POS_IMAGE_TOP_RIGHT_TEXT_2_SIZE0_7 = [450,40]
POS_IMAGE_TOP_RIGHT_TEXT_3_SIZE0_7 = [450,60]
POS_IMAGE_TOP_RIGHT_TEXT_4_SIZE0_7 = [450,80]
POS_IMAGE_BOTTOM_RIGHT_ARROW = [500,250]





# ## LAB Testing values
## Object detection
LNR_VEL_Y_MIN = 0.15

LNR_VEL_X_MIN = 0.15
LNR_VEL_X_OK_MIN = 0.25
LNR_VEL_X_OK_MAX = 0.5 #todo
LNR_VEL_X_ONLY_STRAIGHT_OK_MIN = 0.25
LNR_VEL_X_ONLY_STRAIGHT_OK_MAX = 1.0
LNR_VEL_X_STRAIGHT_RIGHT_OK_MIN = 0.4
LNR_VEL_X_STRAIGHT_RIGHT_OK_MAX = 1.2
LNR_VEL_X_LEFT_OK_MIN = 0.2
LNR_VEL_X_LEFT_OK_MAX = 0.4
LNR_VEL_X_TOO_FAR = 0.5
LNR_VEL_X_POS_STEP = 0.05

ANG_VEL_Z_MIN = 0.15
ANG_VEL_Z_OK_MIN = 0.5
ANG_VEL_Z_OK_MAX = 0.8
ANG_VEL_Z_TOO_FAR = 0.9
ANG_VEL_Z_POS_STEP = 0.2

################################################################
##############    C L A S S E S
################################################################

class FollowMe_Go1():

    def __init__(self):
        

        ######
        ## ROS INIT
        ######
        rospy.init_node('FollowMe_Go1_Side', anonymous=False)
        self.followme_cmdvel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.followme_state_pub = rospy.Publisher('/followme_state', followme_state, queue_size=10) #todo


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
        
        init_ZED_params.depth_mode = sl.DEPTH_MODE.ULTRA # or use NEURAL depth mode
        init_ZED_params.coordinate_units = sl.UNIT.CENTIMETER # Use millimeter units (for depth measurements)
        init_ZED_params.depth_minimum_distance = 25 
        init_ZED_params.depth_maximum_distance = 500 #5m
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


        self.person_depth = DIST_PERSON_CAMERA_INIT_HIGH_VALUE

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
        self.person_detected = False
        self.person_tracked_id = TRACKING_ID_INI

        self.centre_deviation_flag = DIFF_MAINTAINED
        self.centre_deviation_cntr = 0
        self.depth_diff_flag = DIFF_MAINTAINED
        self.depth_diff_cntr = 0
        self.turn_deviation_flag = DIFF_MAINTAINED
        self.turn_deviation_cntr = 0
        
        self.axis_origin = (int(self.image_width/2 + self.image_width/6 ), DIST_PERSON_CAMERA_TOBEKEPT_PIXEL)
        self.prev_cmd_vel_linear_x = 0.0
        self.prev_cmd_vel_angular_z = 0.0
        
        ## CALIBRATION
        self.depth_pixel_ratio_array = [ZED_RATIO_DEPTH_PIXEL] #todo
        self.depth_pixel_ratio_mean = ZED_RATIO_DEPTH_PIXEL # Calibrated on run


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
                       

                        temp_obj_bb2d = object_detected.bounding_box_2d
                        temp_obj_bb2d = temp_obj_bb2d.astype(int)
                        ## Define points for easier access in drawing images
                        self.BB_CORNER_TOP_RIGHT_TEXT = (temp_obj_bb2d[POINT_TOP_RIGHT][POINT_X], temp_obj_bb2d[POINT_TOP_RIGHT][POINT_Y] + DEFAULT_TEXT_PIXEL)
                        idText = repr(object_detected.label) + " ID: "+ str(int(object_detected.id)) + " " + str(int(object_detected.confidence)) + "%"
                        self.camera_raw_op = addOpenCVText(self.camera_raw_op, idText ,self.BB_CORNER_TOP_RIGHT_TEXT, fontScale_ip=TEXT_SIZE_OBJ_INFO)

                        if(self.person_tracked_id == int(object_detected.id)):

                            ## SEARCHING tracking state of ZED isnt reliable. Faced unwanted behaviours few times
                            ## So in case the tracked person state is going to be SEARCHING, we dont detect it
                            if(repr(object_detected.tracking_state) == 'OK'):
                                self.person_detected = True
                                object_being_tracked = object_detected
                            else:
                                print("Tracked person is not in proper ZED Tracking state.")

                    ## If not present, assign a new id only if there is only one object being detected and confidence is good
                    if(self.person_detected != True):
                        if (len(objects_detected_array) == 1) :
                            object_being_tracked = objects_detected_array[0]

                            if(repr(object_being_tracked.tracking_state) == 'OK'):
                                if(int(object_being_tracked.confidence) > OBJECT_DETECTION_ACCURACY_THRESHOLD_REASSIGN):
                                    self.person_tracked_id  = int(object_being_tracked.id)
                                    self.person_detected = True
                                else:
                                    print("The person cannot be tracked as confidence of detection is less!")
                                    self.camera_raw_op = addOpenCVTextAtCentre(self.camera_raw_op, "DETECTION CONFIDENCE IS LESS", color_ip=COLOR_RED)
                            else:
                                print("New detected person is not in proper ZED Tracking state.")
                                self.camera_raw_op = addOpenCVTextAtCentre(self.camera_raw_op, "IMPROPER TRACKING STATE", color_ip=COLOR_RED)
  
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
                        self.camera_raw_op = addOpenCVText(self.camera_raw_op, idText ,self.BB_CORNER_TOP_RIGHT_TEXT, fontScale_ip=TEXT_SIZE_OBJ_INFO)


                        # Make sure the mask is available for detected person
                        if object_being_tracked.mask.is_init():
                            
                            # Calcualte the depth value
                            temp_depth, depth_map_masked  = self.process_depth(object_being_tracked.mask.get_data())

                            ## Check if the depth is valid value
                            if(np.isfinite(temp_depth)):
                                self.person_depth = temp_depth


                        ## Draw depth line and mention depth
                        depthText = str(self.person_depth) + "cm"
                        print("Person Depth : ", depthText)
                        self.camera_raw_op = addOpenCVLine(self.camera_raw_op, (self.axis_origin[POINT_X],self.image_height), self.BB_MIDDLE_BOTTOM_LINE, color_ip=COLOR_RED)                    
                        self.camera_raw_op = addOpenCVText(self.camera_raw_op, depthText, position=(
                                        int((self.axis_origin[POINT_X] + self.BB_MIDDLE_BOTTOM_LINE[0])/2),
                                        int((self.image_height+ self.BB_MIDDLE_BOTTOM_LINE[1])/2)),
                                        color_ip=COLOR_RED , fontScale_ip=TEXT_SIZE_POS
                                        )
                        


                        # Draw thresholds
                        # self.calibrating()

                        # Calculate centre point of camera and draw the lines for reference
                        print("Camera centre : ",self.image_vertical_centre_xpoint, "Axis :", self.axis_origin)
                        self.camera_raw_op = addOpenCVLine(self.camera_raw_op, (self.axis_origin[POINT_X],0), (self.axis_origin[POINT_X],self.image_height), color_ip=COLOR_YELLOW, alpha=TRANSPARENCY_ALPHA_GRAPHS + 0.2)
                        self.camera_raw_op = addOpenCVLine(self.camera_raw_op, (self.axis_origin[POINT_X]-DIST_FROM_CAMERA_CENTRE_THRESHOLD,0), (self.axis_origin[POINT_X]-DIST_FROM_CAMERA_CENTRE_THRESHOLD,self.image_height), color_ip=COLOR_YELLOW, alpha=TRANSPARENCY_ALPHA_GRAPHS)
                        self.camera_raw_op = addOpenCVLine(self.camera_raw_op, (self.axis_origin[POINT_X]+DIST_FROM_CAMERA_CENTRE_THRESHOLD,0), (self.axis_origin[POINT_X]+DIST_FROM_CAMERA_CENTRE_THRESHOLD,self.image_height), color_ip=COLOR_YELLOW, alpha=TRANSPARENCY_ALPHA_GRAPHS)
                        self.camera_raw_op = addOpenCVLine(self.camera_raw_op, (self.axis_origin[POINT_X]-DIST_FROM_CAMERA_CENTRE_TURN_AND_MOVE,0), (self.axis_origin[POINT_X]-DIST_FROM_CAMERA_CENTRE_TURN_AND_MOVE,self.image_height), color_ip=COLOR_ORANGE, alpha=TRANSPARENCY_ALPHA_GRAPHS)
                        # self.camera_raw_op = addOpenCVLine(self.camera_raw_op, (self.axis_origin[POINT_X]+DIST_FROM_CAMERA_CENTRE_TURN_AND_MOVE,0), (self.axis_origin[POINT_X]+DIST_FROM_CAMERA_CENTRE_TURN_AND_MOVE,self.image_height), color_ip=COLOR_ORANGE, alpha=TRANSPARENCY_ALPHA_GRAPHS)
                        self.camera_raw_op = addOpenCVLine(self.camera_raw_op, (self.axis_origin[POINT_X]-DIST_FROM_CAMERA_CENTRE_TOO_FAR,0), (self.axis_origin[POINT_X]-DIST_FROM_CAMERA_CENTRE_TOO_FAR,self.image_height), color_ip=COLOR_RED, alpha=TRANSPARENCY_ALPHA_GRAPHS)
                        # self.camera_raw_op = addOpenCVLine(self.camera_raw_op, (self.axis_origin[POINT_X]+DIST_FROM_CAMERA_CENTRE_TOO_FAR,0), (self.axis_origin[POINT_X]+DIST_FROM_CAMERA_CENTRE_TOO_FAR,self.image_height), color_ip=COLOR_RED, alpha=TRANSPARENCY_ALPHA_GRAPHS)


                        self.camera_raw_op = addOpenCVLine(self.camera_raw_op, (0,DIST_PERSON_CAMERA_TOBEKEPT_PIXEL), (self.image_width, DIST_PERSON_CAMERA_TOBEKEPT_PIXEL), color_ip=COLOR_YELLOW, alpha=TRANSPARENCY_ALPHA_GRAPHS + 0.2)
                       

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


        ## Replace infinite values and nan
        self.camera_depth_map[np.isinf(self.camera_depth_map)] = 0
        self.camera_depth_map[np.isnan(self.camera_depth_map)] = 0

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
        self.centre_deviation_flag = DIFF_MAINTAINED
        self.centre_deviation_cntr = 0
        self.depth_diff_flag = DIFF_MAINTAINED
        self.depth_diff_cntr = 0
        self.turn_deviation_flag = DIFF_MAINTAINED
        self.turn_deviation_cntr = 0
    
        # Check if human is detected
        if(self.person_detected):
                
            # Check if they are close to centre and robot
            if((abs(self.axis_origin[POINT_X] - self.BB_MIDDLE_BOTTOM_LINE[POINT_X]) < DIST_FROM_CAMERA_CENTRE_THRESHOLD)
               and abs(self.person_depth) < DIST_PERSON_CAMERA_VERY_FAR):
                # If yes, change the state to following
                self.state = 'FOLLOWING'
                print("Its near")

                cv2.waitKey(5)
            
            else:
                print("Not near the robot centre")
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

            #####
            # DEPTH : Distance between human and robot standing side to side
            ####

            # Calculate the depth difference from distance to be kept and current depth
            depth_diff = self.person_depth - DIST_PERSON_CAMERA_TOBEKEPT
            
            if((abs(depth_diff) > DIST_PERSON_CAMERA_DIFF_THRESHOLD)
               and self.depth_diff_flag != DIFF_CORRECTING):
                self.depth_diff_flag = DIFF_ERR_DEBOUNCING
                self.depth_diff_cntr += 1

                ## If too far, no need of debouncing
                if(abs(self.person_depth) > DIST_PERSON_CAMERA_VERY_FAR):
                    self.depth_diff_flag = DIFF_CORRECTING
                    self.depth_diff_cntr = DEPTH_ERR_DEBOUNCE_THRESHOLD
            else:
                self.depth_diff_cntr = 0

            print("Depth Flag : ", self.depth_diff_flag, " Depth Difference : ", depth_diff, " Depth Difference Counter : ",self.depth_diff_cntr)


            #####
            # CENTRE : Distance human moved in forward or backward with robot on side
            ####


            centre_deviation = self.axis_origin[POINT_X] - self.BB_MIDDLE_BOTTOM_LINE[POINT_X]
            if((abs(centre_deviation) > DIST_FROM_CAMERA_CENTRE_THRESHOLD)
               and self.centre_deviation_flag != DIFF_CORRECTING):
                self.centre_deviation_flag = DIFF_ERR_DEBOUNCING
                self.centre_deviation_cntr += 1

                ## If too far, no need of debouncing
                if(abs(centre_deviation) > DIST_FROM_CAMERA_CENTRE_TOO_FAR):
                    self.centre_deviation_flag = DIFF_CORRECTING
                    self.centre_deviation_cntr = CENTRE_ERR_DEBOUNCE_THRESHOLD
            else:
                self.centre_deviation_cntr = 0

            print("Centre Deviation Flag : ", self.centre_deviation_flag, " Centre Deviation : ", centre_deviation, " Centre Deviation Counter : ",self.centre_deviation_cntr)


            #####
            # TURN : Distance human turned
            ####

            
            ## Calculate angle of turn : Angle made by line from person to axis origin
            slope_of_personDetected_withAxis = (self.BB_MIDDLE_BOTTOM_LINE[POINT_Y] - self.axis_origin[POINT_Y])/(DIST_FROM_CAMERA_CENTRE_TO_CALC_TURN - self.axis_origin[POINT_X])
            turn_deviation = math.atan(slope_of_personDetected_withAxis)
            angle_theta = math.degrees(turn_deviation)
            print("Angle is :", angle_theta, " Radians ", turn_deviation)
            self.camera_raw_op = addOpenCVText(self.camera_raw_op, "Turn Angle: " + str(angle_theta)  , POS_IMAGE_TOP_RIGHT_TEXT_4_SIZE0_7, color_ip=COLOR_GREEN, fontScale_ip=TEXT_SIZE_CMD_VEL)
            

            ## Is turn deviations needed to checked?
            # - Skip check when person is at centre, except when correcting
            # - Skip turning for now when person moves back
            check_turn_deviation = True
            if(((abs(centre_deviation) <= DIST_FROM_CAMERA_CENTRE_THRESHOLD) and self.turn_deviation_flag != DIFF_CORRECTING)
               or centre_deviation < 0):
                print("Skipping turn deviation check")
                check_turn_deviation = False
                self.turn_deviation_flag = DIFF_MAINTAINED
            
            # Angle is positive for forward right and negative for forward left
            if((abs(turn_deviation) > TURN_FROM_AXIS_THRESHOLD)
               and self.turn_deviation_flag != DIFF_CORRECTING
               and check_turn_deviation == True):
                self.turn_deviation_flag = DIFF_ERR_DEBOUNCING
                self.turn_deviation_cntr += 1

                ## If too far, no need of debouncing
                if(abs(turn_deviation) > TURN_FROM_AXIS_VERY_FAR):
                    self.turn_deviation_flag = DIFF_CORRECTING
                    self.turn_deviation_cntr = TURN_ERR_DEBOUNCE_THRESHOLD
            else:
                self.turn_deviation_cntr = 0

            print("Turn Deviation Flag : ", self.turn_deviation_flag, " Turn Deviation : ", turn_deviation, " Turn Deviation Counter : ",self.turn_deviation_cntr)



            #####
            # CORRECTING DEVIATIONS
            #####

            ## Handling edges at left turn. Between DIST_FROM_CAMERA_CENTRE_TURN_AND_MOVE and centre, if depth is too less, do only depth
            ## Outside DIST_FROM_CAMERA_CENTRE_TURN_AND_MOVE and camera edge, priority for turn only
            ## Between DIST_FROM_CAMERA_CENTRE_TURN_AND_MOVE and axis, do small linear and turn. Only do depth if depth low as point 1

            ## Only move left/right if the debounce threshold is reached
            if(self.centre_deviation_flag == DIFF_CORRECTING):                    

                # If difference is less than threshold, move back
                if(centre_deviation < 0):
                    print("Person moved back")
                    followme_cmd_vel.linear.x  = NEG_SIGN * LNR_VEL_X_MIN
                    self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'right' )



                if(centre_deviation > 0):

                    if(self.person_depth > DIST_FROM_CAMERA_CENTRE_TOO_FAR):
                        print("Person moved too front")
                        followme_cmd_vel.linear.x = POS_SIGN * LNR_VEL_X_TOO_FAR
                        self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'left', color_ip=COLOR_ORANGE )

                    else:

                        ## Case 1 : Only straight
                        if(self.turn_deviation_flag != DIFF_CORRECTING):             
                            print("Person moved front")
                            speed_slope = (LNR_VEL_X_ONLY_STRAIGHT_OK_MAX - LNR_VEL_X_ONLY_STRAIGHT_OK_MIN)/(DIST_FROM_CAMERA_CENTRE_TOO_FAR - DIST_FROM_CAMERA_CENTRE_NEAR)
                            followme_cmd_vel.linear.x  = abs(centre_deviation - DIST_FROM_CAMERA_CENTRE_NEAR) * speed_slope + LNR_VEL_X_ONLY_STRAIGHT_OK_MIN
                            print("Slope = " + str(speed_slope) + " Linear x speed [only straight] = " + str(followme_cmd_vel.linear.x))

                            self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'left' )



                        ## Case 2 : Straight and right turn
                        # Turn with more speed in linear
                        if(self.turn_deviation_flag == DIFF_CORRECTING and turn_deviation > 0):             
                            print("Person moved front right")
                            speed_slope = (LNR_VEL_X_STRAIGHT_RIGHT_OK_MAX - LNR_VEL_X_STRAIGHT_RIGHT_OK_MIN)/(DIST_FROM_CAMERA_CENTRE_TOO_FAR - DIST_FROM_CAMERA_CENTRE_NEAR)
                            followme_cmd_vel.linear.x  = abs(centre_deviation - DIST_FROM_CAMERA_CENTRE_NEAR) * speed_slope + LNR_VEL_X_STRAIGHT_RIGHT_OK_MIN
                            print("Slope = " + str(speed_slope) + " Linear x speed [straight + right] = " + str(followme_cmd_vel.linear.x))

                            self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'left' )




                        ## Case 3 : Straight and left turn

                            # Only if its not 'turn left correcting and depth is low region'

                            # No linear x bw DIST_FROM_CAMERA_CENTRE_TURN_AND_MOVE and camera edge
                            # Very low speed in other region close to axis
                        if((self.turn_deviation_flag == DIFF_CORRECTING and turn_deviation < 0)
                            and  abs(centre_deviation) > DIST_FROM_CAMERA_CENTRE_TURN_AND_MOVE ):                        
                            print("Person moved front left")

                            speed_slope = (LNR_VEL_X_LEFT_OK_MAX - LNR_VEL_X_LEFT_OK_MIN)/(DIST_FROM_CAMERA_CENTRE_TOO_FAR - DIST_FROM_CAMERA_CENTRE_NEAR)
                            followme_cmd_vel.linear.x  = abs(centre_deviation - DIST_FROM_CAMERA_CENTRE_NEAR) * speed_slope + LNR_VEL_X_LEFT_OK_MIN
                            print("Slope = " + str(speed_slope) + " Linear x speed [straight + left] = " + str(followme_cmd_vel.linear.x))

                            self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'left' )


                    print("Previous speed x",  self.prev_cmd_vel_linear_x)
                    ## Ramping up of cmd_vel to avoid sudden high values above min value
                    if(followme_cmd_vel.linear.x - self.prev_cmd_vel_linear_x > LNR_VEL_X_POS_STEP ):
                        # Increment with step size
                        followme_cmd_vel.linear.x = self.prev_cmd_vel_linear_x + LNR_VEL_X_POS_STEP                        
                        print("Ramping up the linear x by ", LNR_VEL_X_POS_STEP, " and is now ", followme_cmd_vel.linear.x )
                        if(followme_cmd_vel.linear.x < LNR_VEL_X_OK_MIN):
                            followme_cmd_vel.linear.x = LNR_VEL_X_OK_MIN
                            print("Keeping the linear x at minimum configured velocity")


                ## Resetting of aligning to centre
                if(abs(centre_deviation) < DIST_FROM_CAMERA_CENTRE_NEAR):
                    self.centre_deviation_flag = DIFF_MAINTAINED
                    self.centre_deviation_cntr = 0
                    print("Centre is maintained")

            else:
                print("Centre Maintained")
                followme_cmd_vel.linear.x = 0.0

                if(self.centre_deviation_cntr >= CENTRE_ERR_DEBOUNCE_THRESHOLD):
                    self.centre_deviation_flag = DIFF_CORRECTING
                elif(self.centre_deviation_cntr == 0):
                    self.centre_deviation_flag = DIFF_MAINTAINED



            ## Only turn left/right if the debounce threshold is reached
            if(self.turn_deviation_flag == DIFF_CORRECTING):
                
                ## Too big turn
                if(abs(turn_deviation) > TURN_FROM_AXIS_VERY_FAR ):

                    # If difference is less than threshold, turn left
                    if(turn_deviation < 0):
                        print("Person turned too left")
                        ## Only turn left
                        followme_cmd_vel.angular.z = POS_SIGN * ANG_VEL_Z_TOO_FAR                    
                        self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'bottomleft', color_ip=COLOR_ORANGE )
                
                    # If more, turn rgiht
                    if(turn_deviation > 0):
                        print("Person turned too right")
                        ## Only turn right
                        followme_cmd_vel.angular.z = NEG_SIGN * ANG_VEL_Z_TOO_FAR                    
                        self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'topleft', color_ip=COLOR_ORANGE )

                else:
                    ## Calculate the velocity
                    ang_vel_slope = (ANG_VEL_Z_OK_MAX - ANG_VEL_Z_OK_MIN)/(TURN_FROM_AXIS_VERY_FAR - TURN_FROM_AXIS_NEAR)
                    ang_vel_temp = (abs(turn_deviation) - TURN_FROM_AXIS_NEAR) * ang_vel_slope + ANG_VEL_Z_OK_MIN
                    print("Angular Vel Slope = " + str(ang_vel_slope) + " Angular z speed = " + str(ang_vel_temp))

                    # If difference is less than threshold, turn left
                    if(turn_deviation < 0):
                        print("Person turned left")
                        ## Only turn left
                        followme_cmd_vel.angular.z = POS_SIGN * ang_vel_temp                    
                        self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'bottomleft' )
                
                    # If more, turn right
                    if(turn_deviation > 0):
                        print("Person turned right")
                        ## Only turn right
                        followme_cmd_vel.angular.z = NEG_SIGN * ang_vel_temp                    
                        self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'topleft' )



                print("Previous angular speed z",  self.prev_cmd_vel_angular_z)
                ## Ramping up of cmd_vel to avoid sudden high values above min value
                
                # Turning left
                if(turn_deviation < 0):

                    if(followme_cmd_vel.angular.z - self.prev_cmd_vel_angular_z > ANG_VEL_Z_POS_STEP ):
                        # Increment with step size
                        followme_cmd_vel.angular.z = self.prev_cmd_vel_angular_z + ANG_VEL_Z_POS_STEP                        
                        print("Ramping up the angular z by ", ANG_VEL_Z_POS_STEP, " and is now ", followme_cmd_vel.angular.z )
                        if(followme_cmd_vel.angular.z < ANG_VEL_Z_MIN):
                            followme_cmd_vel.angular.z = ANG_VEL_Z_MIN
                            print("Keeping the angular z at minimum configured velocity of ", followme_cmd_vel.angular.z)
                    
                # Turning right
                if(turn_deviation > 0):

                    if(followme_cmd_vel.angular.z - self.prev_cmd_vel_angular_z < (NEG_SIGN * ANG_VEL_Z_POS_STEP) ):
                        # Increment with step size
                        followme_cmd_vel.angular.z = self.prev_cmd_vel_angular_z - ANG_VEL_Z_POS_STEP                        
                        print("Ramping down the angular z by ", ANG_VEL_Z_POS_STEP, " and is now ", followme_cmd_vel.angular.z )
                        if(followme_cmd_vel.angular.z > (NEG_SIGN * ANG_VEL_Z_MIN)):
                            followme_cmd_vel.angular.z = NEG_SIGN * ANG_VEL_Z_MIN
                            print("Keeping the angular z at minimum configured velocity of ", followme_cmd_vel.angular.z)



                ## Resetting of aligning to centre
                if(abs(turn_deviation) < TURN_FROM_AXIS_NEAR or centre_deviation <= 0):
                    followme_cmd_vel.angular.z = 0.0
                    self.turn_deviation_flag = DIFF_MAINTAINED
                    self.turn_deviation_cntr = 0
                    print("Turn is maintained")

            else:
                print("Turn Maintained")
                followme_cmd_vel.angular.z = 0.0

                if(self.turn_deviation_cntr >= TURN_ERR_DEBOUNCE_THRESHOLD):
                    self.turn_deviation_flag = DIFF_CORRECTING
                elif(self.turn_deviation_cntr == 0):
                    self.turn_deviation_flag = DIFF_MAINTAINED

            

            ## Only move in y direction, when turning else is not going on
            if(self.depth_diff_flag == DIFF_CORRECTING):
                
                if(self.turn_deviation_flag != DIFF_CORRECTING
                   or (self.person_depth < DIST_PERSON_CAMERA_TOO_CLOSE and abs(centre_deviation) < DIST_FROM_CAMERA_CENTRE_TURN_AND_MOVE)): #Handle left edge
                    # If difference is greater than threshold, move forward
                    if(depth_diff > 0):
                            
                        print("Moving right")
                        followme_cmd_vel.linear.y = NEG_SIGN * LNR_VEL_Y_MIN
                        self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'up' )

                    # If less, move backward
                    if(depth_diff < 0):
                            
                        print("Moving left")
                        followme_cmd_vel.linear.y = POS_SIGN * LNR_VEL_Y_MIN

                        if(self.person_depth < DIST_PERSON_CAMERA_TOO_CLOSE and abs(centre_deviation) < DIST_FROM_CAMERA_CENTRE_TURN_AND_MOVE): #Handle left edge
                            followme_cmd_vel.linear.x = 0.0
                            print("Resetting forward velocity")
                            self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'down' , color_ip=COLOR_RED)
                        else:
                            self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'down' )

                else:
                    print("Skipping depth deviation correction")


                ## Resetting of aligning to centre
                if(abs(depth_diff) <= DIST_PERSON_CAMERA_VALID_THRESHOLD):
                    self.depth_diff_flag = DIFF_MAINTAINED
                    self.depth_diff_cntr = 0
                    print("Depth is maintained")


            else:
                print("Depth Maintained")
                followme_cmd_vel.linear.y = 0.0

                if(self.depth_diff_cntr >= DEPTH_ERR_DEBOUNCE_THRESHOLD):
                    self.depth_diff_flag = DIFF_CORRECTING
                elif(self.depth_diff_cntr == 0):
                    self.depth_diff_flag = DIFF_MAINTAINED



        else:
            # Reset as no person seen
            # Not explicitly resetting as its already zero as init value
            print("No person detected")

        ## Publish cmd vel
        self.camera_raw_op = addOpenCVText(self.camera_raw_op, "Linear x  : " + str(followme_cmd_vel.linear.x)  , POS_IMAGE_TOP_RIGHT_TEXT_1_SIZE0_7, color_ip=COLOR_GREEN, fontScale_ip=TEXT_SIZE_CMD_VEL)
        self.camera_raw_op = addOpenCVText(self.camera_raw_op, "Linear y  : " + str(followme_cmd_vel.linear.y)  , POS_IMAGE_TOP_RIGHT_TEXT_2_SIZE0_7, color_ip=COLOR_GREEN, fontScale_ip=TEXT_SIZE_CMD_VEL)
        self.camera_raw_op = addOpenCVText(self.camera_raw_op, "Angular z : " + str(followme_cmd_vel.angular.z)  , POS_IMAGE_TOP_RIGHT_TEXT_3_SIZE0_7, color_ip=COLOR_GREEN, fontScale_ip=TEXT_SIZE_CMD_VEL)
  

        self.publish_cmdvel_safe(followme_cmd_vel)           




    def calibrating(self): #todo

            
        ## Using only data closer to the distance to be maintained
        if(abs(self.person_depth - DIST_PERSON_CAMERA_TOBEKEPT) <= DIST_PERSON_CAMERA_DIFF_THRESHOLD):


            self.depth_pixel_ratio_array.append(self.person_depth/ self.BB_MIDDLE_BOTTOM_LINE[POINT_Y])
            self.depth_pixel_ratio_mean = sum(self.depth_pixel_ratio_array) / len(self.depth_pixel_ratio_array)
            print("Current Depth Pixel ratio is ",self.depth_pixel_ratio_mean )
            ## RESULT: 0.215

        if(len(self.depth_pixel_ratio_array) > 100):
            self.depth_pixel_ratio_array = [self.depth_pixel_ratio_mean]
            print("Reseting the depth_pixel_ratio_array to save memory.")


    def publish_cmdvel_safe(self,followme_cmd_vel):
        # Control and Safety checks for final published cmd vel

        self.prev_cmd_vel_linear_x =  followme_cmd_vel.linear.x
        self.prev_cmd_vel_angular_z = followme_cmd_vel.angular.z

        # Set zeroes to all unused variables
        followme_cmd_vel.linear.z = 0.0
        followme_cmd_vel.angular.x = 0.0
        followme_cmd_vel.angular.y = 0.0

        # No angular when there is lienar y
        if(followme_cmd_vel.linear.y != 0.0):
            followme_cmd_vel.angular.z = 0.0
            print("Safe : Reseting angular velocity to zero")

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


                ## This function is used to calculate the pixel value that corresponds to the depth to be maintained with robot dog.
                ## Due to complications in mathematically calculating, it is calculated with test.
                ## KEEP ROBOT in lay down position
                ## NOT BOTH FOLLOWME AND CALIBRATING SHOULD BE ACTIVE
                #self.calibratng() # purposefully kept wrong spelling, so no accidentally activating

                self.following()



            self.followme_state_pub.publish(self.state) 

            
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
        self.publish_cmdvel_safe(followme_cmd_vel)
                


        
        cv2.destroyAllWindows()
        # Close the camera
        self.zed.disable_object_detection()
        self.zed.close()





################################################################
##############    H E L P E R   F U N C T I O N S
################################################################
def addOpenCVText(image,text_ip,position = (200, 200), fontScale_ip = DEFAULT_TEXT_SIZE, color_ip = DEFAULT_TEXT_COLOR ):
    
    howthick = 2
    if(fontScale_ip < 0.7):
        howthick = 1

    new_imagewithText = cv2.putText(    img = image,
                                        text = text_ip,
                                        org = position,
                                        fontFace = cv2.FONT_HERSHEY_SIMPLEX,
                                        fontScale = fontScale_ip,
                                        color = color_ip,
                                        thickness = howthick
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


def addOpenCVLine(image, start_pos = (0, 0), end_pos = (100, 100), color_ip = DEFAULT_TEXT_COLOR, alpha=1.0 ):

    # Create a copy of the original image to draw the text layer
    overlay = image.copy()

    cv2.line(   overlay, start_pos, end_pos,
                                    color = color_ip,
                                    thickness = 2
                                    ) 
    
    new_imagewithLine = cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0)

    return new_imagewithLine


def addOpenCVArrow(image, start_pos = [0, 0], direction = 'up', color_ip = DEFAULT_TEXT_COLOR ):

    end_pos = start_pos

    if(direction == 'up'):
        end_pos = [start_pos[POINT_X], start_pos[POINT_Y] - ARROW_LENGTH]

    elif(direction == 'down'):
        end_pos = [start_pos[POINT_X], start_pos[POINT_Y] + ARROW_LENGTH]

    elif(direction == 'left'):
        end_pos = [start_pos[POINT_X]- ARROW_LENGTH, start_pos[POINT_Y]]

    elif(direction == 'right'):
        end_pos = [start_pos[POINT_X]+ ARROW_LENGTH, start_pos[POINT_Y]]

    elif(direction == 'bottomleft'):
        end_pos = [start_pos[POINT_X]- ARROW_LENGTH, start_pos[POINT_Y] + ARROW_LENGTH]

    elif(direction == 'topleft'):
        end_pos = [start_pos[POINT_X]- ARROW_LENGTH, start_pos[POINT_Y] - ARROW_LENGTH]



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







