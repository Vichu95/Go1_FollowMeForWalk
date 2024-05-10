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
MIN_VEL_FOLLOME_POS = 0.25
MIN_VEL_FOLLOME_NEG = -0.25
ANG_VEL_FOLLOME_POS = 0.9
ANG_VEL_FOLLOME_NEG = -0.9


## Object detection
OBJECT_DETECTION_ACCURACY_THRESHOLD = 40
DIST_PERSON_CAMERA_TOBEKEPT = 60
DIST_PERSON_CAMERA_DIFF_THRESHOLD = 10
DIST_FROM_CAMERA_CENTRE_THRESHOLD = 84 #Image width/8 . Reinitialized in init

POINT_X= 0
POINT_Y = 1
POINT_TOP_LEFT = 0
POINT_TOP_RIGHT = 1
POINT_BOTTOM_RIGHT = 2
POINT_BOTTOM_LEFT = 3

## OpenCV
COLOR_BLUE = [255,0,0]
COLOR_YELLOW = [0,255,255]
COLOR_GREEN = [0,255,0]
COLOR_RED = [0,0,255]

OBJ_BB_THICKNESS = 2
DEFAULT_TEXT_SIZE = 1
DEFAULT_TEXT_COLOR = COLOR_BLUE
DEFAULT_TEXT_PIXEL = DEFAULT_TEXT_SIZE * 25 #Needed for adding text in images

ARROW_LENGTH = 50
ARROW_TIP = 0.5
ARROW_THICKNESS = 12

POS_IMAGE_TOP_LEFT_TEXT = [10,30]
POS_IMAGE_BOTTOM_RIGHT_ARROW = [500,250]

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
        self.image_middle_bottom_line = (int(self.image_width/2), self.image_height)
        self.image_vertical_centre_xpoint = int(self.image_width/2)

        global DIST_FROM_CAMERA_CENTRE_THRESHOLD
        DIST_FROM_CAMERA_CENTRE_THRESHOLD = int(self.image_width/8)

        self.camera_raw_op = np.zeros((self.image_height,self.image_width))
        self.camera_depth_op = ''
        self.camera_depth_map = ''

        self.state = 'INIT'
        self.person_detected = False

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
                    
                    ## TODO : store the id of person being tracked, check if the id is same in detected objects
                    self.person_detected = True

                    ## Get the first object
                    first_object = objects_detected_array[0]

                    # Read the bounding box 2D
                    self.obj_bb2d = first_object.bounding_box_2d
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
                    
                    idText = repr(first_object.label) + " ID: "+ str(int(first_object.id)) + " " + str(int(first_object.confidence)) + "%"
                    self.camera_raw_op = addOpenCVText(self.camera_raw_op, idText ,self.BB_CORNER_TOP_RIGHT_TEXT)

                    # Make sure the mask is available for detected person
                    if first_object.mask.is_init():
                        
                        # Calcualte the depth value
                        self.person_depth, depth_map_masked  = self.process_depth(first_object.mask.get_data())

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
                
                    # Calculate centre point of the detected person
                    print("Person centre : ",self.BB_MIDDLE_BOTTOM_LINE[POINT_X])
                    self.camera_raw_op = addOpenCVLine(self.camera_raw_op, (self.BB_MIDDLE_BOTTOM_LINE[POINT_X],0), (self.BB_MIDDLE_BOTTOM_LINE[POINT_X],self.image_height), color_ip=COLOR_GREEN)


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
            depth_diff = 0
            if(abs(self.person_depth - DIST_PERSON_CAMERA_TOBEKEPT) > DIST_PERSON_CAMERA_DIFF_THRESHOLD):
                depth_diff = self.person_depth - DIST_PERSON_CAMERA_TOBEKEPT
                depth_diff_flag = True

            print("Depth Flag : ", depth_diff_flag, " Depth Difference : ", depth_diff)

            if(depth_diff_flag):
                # If difference is greater than threshold, move forward
                if(depth_diff > 0):
                    print("Move forward")
                    followme_cmd_vel.linear.x = MIN_VEL_FOLLOME_POS
                    self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'up' )
            
                # If less, move backward
                if(depth_diff < 0):
                    print("Move backward")
                    followme_cmd_vel.linear.x = MIN_VEL_FOLLOME_NEG
                    self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'down' )

            else:
                print("Depth Maintained")
                followme_cmd_vel.linear.x = 0.0




            # Calculate the angle difference from distance to be kept and current distance from centre
            centre_deviation_flag = False
            centre_deviation = 0

            if(abs(self.image_vertical_centre_xpoint - self.BB_MIDDLE_BOTTOM_LINE[POINT_X]) > DIST_FROM_CAMERA_CENTRE_THRESHOLD) :
                centre_deviation = self.image_vertical_centre_xpoint - self.BB_MIDDLE_BOTTOM_LINE[POINT_X]
                centre_deviation_flag = True

            print("Centre Deviation Flag : ", centre_deviation_flag, " Centre Deviation : ", centre_deviation)


            if(centre_deviation_flag):
                # If difference is greater than threshold, move right
                if(centre_deviation > 0):
                    print("Person moved to my Right")

                    if(depth_diff_flag):
                        # Move forward towards right side
                        if(depth_diff > 0):
                            followme_cmd_vel.angular.z = ANG_VEL_FOLLOME_POS
                    
                        # Move backward towards left side
                        elif(depth_diff < 0):
                            followme_cmd_vel.angular.z = ANG_VEL_FOLLOME_POS
                    else:
                        ## Only turn right
                        followme_cmd_vel.angular.z = ANG_VEL_FOLLOME_POS   #working


                    self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'right' )
            
                # If less, move left
                if(centre_deviation < 0):
                    print("Person moved to my Left")                


                    if(depth_diff_flag):
                        # Move forward towards left side
                        if(depth_diff > 0):
                            followme_cmd_vel.angular.z = ANG_VEL_FOLLOME_NEG
                    
                        # Move backward towards right side
                        elif(depth_diff < 0):
                            followme_cmd_vel.angular.z = ANG_VEL_FOLLOME_NEG
                    else:
                        ## Only turn left
                        followme_cmd_vel.angular.z = ANG_VEL_FOLLOME_NEG  #working


                    self.camera_raw_op = addOpenCVArrow( self.camera_raw_op, start_pos = POS_IMAGE_BOTTOM_RIGHT_ARROW, direction = 'left' )

            else:
                print("Centre Maintained")
                followme_cmd_vel.angular.z = 0.0

        #else:
        # Reset as no person seen
        # Not explicitly resetting as its already zero as init value

        ## Publish cmd vel

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


            
            cv2.imshow("Camera", self.camera_raw_op) #Display image
            #cv2.imshow("Depth", self.camera_depth_op)
            key = cv2.waitKey(1)


                        
            # Encode the frame as JPEG
            _, jpeg = cv2.imencode('.jpg', self.camera_raw_op)
            frame_bytes = jpeg.tobytes()

            # Send the frame to the Flask server
            try:
                response = requests.post('http://192.168.12.65:5000/update_frame', data=frame_bytes)
                if response.status_code != 200:
                    print("Failed to send frame to server")
            except requests.exceptions.RequestException as e:
                print("Error sending frame to server:", e)

            
            try:
                response = requests.get('http://192.168.12.65:5000/get_stopcmd_value')
                data = response.json()
                print(data)

                if(data.get('stop_request_cmd') == 'STOP'):           
                    key = 113
                    print("STOP received ")
                    response = requests.post('http://192.168.12.65:5000/update_stopcmd_value', data={'stop_request_cmd': 'Init'})

            except requests.ConnectionError:
                print("")
                # print("Connection error: Failed to connect to the server:",requests.ConnectionError)	

            
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




