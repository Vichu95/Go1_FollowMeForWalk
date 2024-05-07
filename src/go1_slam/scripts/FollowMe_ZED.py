#!/usr/bin/env python

################################################################
##############    I N C L U D E S 
################################################################

#ZED
import pyzed.sl as sl

#ROS
import rospy

import cv2
import numpy as np




################################################################
##############    M A C R O S
################################################################




## Object detection
OBJECT_DETECTION_ACCURACY_THRESHOLD = 40
DIST_PERSON_CAMERA_TOBEKEPT = 100
DIST_FROM_CAMERA_CENTRE_THRESHOLD = 168 #Image width/4 . Reinitialized in init

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

OBJ_BB_THICKNESS = 2
DEFAULT_TEXT_SIZE = 1
DEFAULT_TEXT_COLOR = COLOR_BLUE
DEFAULT_TEXT_PIXEL = DEFAULT_TEXT_SIZE * 25 #Needed for adding text in images


################################################################
##############    C L A S S E S
################################################################

class FollowMe_Go1():

    def __init__(self):
        
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
        DIST_FROM_CAMERA_CENTRE_THRESHOLD = int(self.image_width/4)

        self.camera_raw_op = '' 
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

        
        ## Show state
        self.camera_raw_op = addOpenCVText(self.camera_raw_op, self.state , (0,0))


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

            ## Analyzing the objects and getting relevant informations
            if objects_detected.is_new :
                objects_detected_array = objects_detected.object_list
                print("\n\nObject(s) detected = " + str(len(objects_detected_array))+"\n")

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
                    
                    print(self.BB_MIDDLE_BOTTOM_LINE)
                    print(self.image_middle_bottom_line)

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
                    self.camera_raw_op = addOpenCVLine(self.camera_raw_op, self.image_middle_bottom_line, self.BB_MIDDLE_BOTTOM_LINE, color_ip=COLOR_YELLOW)                    
                    self.camera_raw_op = addOpenCVText(self.camera_raw_op, depthText, position=(
                                    int((self.image_middle_bottom_line[0]+ self.BB_MIDDLE_BOTTOM_LINE[0])/2),
                                    int((self.image_middle_bottom_line[1]+ self.BB_MIDDLE_BOTTOM_LINE[1])/2)),
                                    color_ip=COLOR_YELLOW
                                    )

            cv2.imshow("Camera", self.camera_raw_op) #Display image
            cv2.imshow("Depth", self.camera_depth_op)
            cv2.waitKey(1)

        else:
            print("\nZED Grab function failed.\n")


        

    def process_depth(self, mask_data ):

        print(mask_data.shape)


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


                # Calculate centre point of camera
                self.image_vertical_centre_xpoint
                # Calculate centre point of the detected person
                self.BB_MIDDLE_BOTTOM_LINE[POINT_X]
        
                # Check if they are close
                if(abs(self.image_vertical_centre_xpoint - self.BB_MIDDLE_BOTTOM_LINE[POINT_X]) < DIST_FROM_CAMERA_CENTRE_THRESHOLD) :
                    # If yes, change the state to following
                    self.state = 'FOLLOWING'
                
                else:
                    print("Not close")
        
        else:
            print("Person not detected...")


        

    def following(self):

        print("Following the person ")

        # Calculate the depth difference from distance to be kept and current depth

        # If difference is greater than threshold, move forward

        # If less, move backward


        # Check the difference from centre of camera
        # If not within threshold, rotate left or right



        


    def followme_run(self):

        print("Executing run")
        
        while True:
            ## Read the camera first
            self.capture_camera()

            if self.state == 'INIT':
                self.starting()

            elif self.state == 'FOLLOWING':
                self.following()
              


        
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


################################################################
##############    M A I N
################################################################


if __name__ == "__main__":
    

    print("Started..")
    followme_go1 = FollowMe_Go1()

    followme_go1.followme_run()


