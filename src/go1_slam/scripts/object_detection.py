########################################################################
#
# Copyright (c) 2022, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

import pyzed.sl as sl
import cv2
import numpy as np




def set_subarray(indices, input_array, subarray):
    '''
    Function used to set subarray values inside an array
    Parameters:
        indices         list of 2D coordinates (4 points defining the subarray "box" inside the array)
        input_array     array to modify
        subarray        subarray to take values from
    '''
    row_count = 0
    # Iterate over the rows of the array to modify
    for i in range(int(indices[0,1]),int(indices[3,1])):
        # Replace array values with subarray values
        input_array[i,int(indices[0,0]):int(indices[1,0])] = subarray[row_count]
        row_count = row_count + 1
    return input_array


def main():
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.VGA
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.METER
    init_params.sdk_verbose = 1

    init_params.depth_mode = sl.DEPTH_MODE.NEURAL # Use ULTRA depth mode
    init_params.coordinate_units = sl.UNIT.CENTIMETER # Use millimeter units (for depth measurements)
    init_params.depth_minimum_distance = 30 
    init_params.depth_stabilization = 30 
    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Camera Open : "+repr(err)+". Exit program.")
        exit()

    obj_param = sl.ObjectDetectionParameters()
    obj_param.enable_tracking=True
    #obj_param.enable_segmentation=True
    #obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX  
    obj_param.enable_mask_output = True
    
    if obj_param.enable_tracking :
        positional_tracking_param = sl.PositionalTrackingParameters()
        #positional_tracking_param.set_as_static = True
        zed.enable_positional_tracking(positional_tracking_param)

    print("Object Detection: Loading Module...")

    err = zed.enable_object_detection(obj_param)
    if err != sl.ERROR_CODE.SUCCESS :
        print("Enable object detection : "+repr(err)+". Exit program.")
        zed.close()
        exit()

    objects = sl.Objects()
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 40
    obj_runtime_param.object_class_filter = [sl.OBJECT_CLASS.PERSON]    # Only detect Persons


    # Set runtime parameters after opening the camera
    runtime = sl.RuntimeParameters()
    runtime.sensing_mode = sl.SENSING_MODE.STANDARD # Preserves edges and depth accuracy

    mat = sl.Mat() 
    depth_for_display = sl.Mat()
    depth_map = sl.Mat()
    iter = 0
    key = ''
    while key != 113:  # for 'q' key
        zed.grab(runtime)
        zed.retrieve_objects(objects, obj_runtime_param)
        zed.retrieve_image(mat, sl.VIEW.LEFT) # Retrieve left image
        zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH) # Retrieve depth
        cvImage = mat.get_data() # Convert sl.Mat to cv2.Mat

        zed.retrieve_image(depth_for_display, sl.VIEW.DEPTH)
        if objects.is_new :
            obj_array = objects.object_list
            print("\n\n " + str(len(obj_array))+" Object(s) detected\n")
            if len(obj_array) > 0 :
                first_object = obj_array[0]
                print("First object attributes:")
                print(" Label '"+repr(first_object.label)+"' (conf. "+str(int(first_object.confidence))+"/100)")
                if obj_param.enable_tracking :
                    print(" Tracking ID: "+str(int(first_object.id))+" tracking state: "+repr(first_object.tracking_state)+" / "+repr(first_object.action_state))
                position = first_object.position
                velocity = first_object.velocity
                dimensions = first_object.dimensions
                print(" 3D position: [{0},{1},{2}]\n Velocity: [{3},{4},{5}]\n 3D dimentions: [{6},{7},{8}]".format(position[0],position[1],position[2],velocity[0],velocity[1],velocity[2],dimensions[0],dimensions[1],dimensions[2]))
                if first_object.mask.is_init():
                    print(" 2D mask available")

                print(" Bounding Box 2D ")
                bounding_box_2d = first_object.bounding_box_2d
                print(bounding_box_2d[0])
                cvImage = cv2.rectangle(cvImage,[int(bounding_box_2d[0][0]),int(bounding_box_2d[0][1])], [int(bounding_box_2d[2][0]),int(bounding_box_2d[2][1])],(255,0,0),2)
                for it in bounding_box_2d :
                    print("    "+str(it),end='')
                print("\n Bounding Box 3D ")
                bounding_box = first_object.bounding_box
                for it in bounding_box :
                    print("    "+str(it),end='')

                depth_value = depth_map.get_value(
                    int((int(bounding_box_2d[1][0]) - int(bounding_box_2d[0][0]))/2)
                    , int((int(bounding_box_2d[2][1]) - int(bounding_box_2d[0][1]))/2)
                    )

                print("\n\nDepth is ", depth_value, "\n\n")

                # Make sure the mask is available for detected person
                if first_object.mask.is_init():
                    mask_data = first_object.mask.get_data()
                    print(type(mask_data))
                    print((mask_data.shape))
                    ###############################################################
                    # Display mask on top of left image
                    ###############################################################
                    # Create an empty overlay mat with the size of the original image
                    overlay = np.zeros((zed.get_camera_information().camera_resolution.height, zed.get_camera_information().camera_resolution.width,4), dtype='uint8')
                    # Convert the 2D mask into a 4-channel mat
                    output_mask = cv2.cvtColor(mask_data, cv2.COLOR_GRAY2BGRA)
                    # Replace overlay mat with mask data inside the given 2D bounding box (see sl.ObjectData.bounding_box_2d doc for more info)
                    bounding_box = first_object.bounding_box_2d
                    set_subarray(bounding_box, overlay, output_mask)
                    # Overlay mask on top of the left image

                    cv2.addWeighted(cvImage, 1, overlay, 0.5, 0.0, cvImage)



                    ## Calculating average depth in the mat
                    # Create a larger 2D array
                    # Get the dimensions of the matrix
                    cvheight, cvwidth, _ = cvImage.shape

                    print("Image size", cvheight, " ", cvwidth)
                    larger_array = np.zeros((cvheight, cvwidth))  # Example 5x5 array filled with zeros


                    # Define the coordinates of the rectangle in the larger array
                    x1, y1 = int(bounding_box_2d[0][0]), int(bounding_box_2d[0][1])  # Top-left corner
                    x2, y2 = int(bounding_box_2d[2][0]), int(bounding_box_2d[2][1])  # Bottom-right corner


                    print(x1,y1,x2,y2)

                    print("Mask data shape",mask_data.shape)
                    print("larger array data shape",larger_array.shape)
                    # # Calculate the dimensions of the smaller array based on the rectangle
                    # smaller_width = x2 - x1
                    # smaller_height = y2 - y1

                    # # Create the smaller array
                    # smaller_array = np.ones((smaller_height, smaller_width))  # Example smaller array filled with ones

                    # # Create a mask for non-zero elements
                    # mask = (smaller_array != 0)

                    # # Replace non-zero elements with 1
                    # smaller_array[mask] = 255

                    # Load the larger array
                    # Assuming larger_array is already defined or loaded

                    # Replace the region within the rectangle in the larger array with the smaller array
                    larger_array[y1:y2, x1:x2] = mask_data




                    #cv2.addWeighted(cvImage, 1, larger_array, 0.5, 0.0, cvImage)



                    # Display the larger array after replacement
                    #print(smaller_array)
                    #print("Larger Array after replacement:")
                    #print(larger_array)


                    # Generate a random 2D array with elements between 1 and 10
                    # random_array = np.random.randint(1, 11, size=(10, 10))

                    # # Print the random array
                    # print("Random 2D array:")
                    # print(random_array)


                    # Find the indices where the other array has 255s
                    indices = np.where(larger_array == 255)

                    depthvalue = depth_map.get_data()
                    print("depth data shape",depthvalue.shape)
                    # print("indxces data shape",indices.shape)

                    # Extract elements from the random array corresponding to the indices
                    corresponding_elements = depthvalue[indices]

                    # Print the corresponding elements
                    print("Corresponding elements from random array:", corresponding_elements)



                    # Calculate the mean of the array
                    mean_value = np.mean(corresponding_elements)

                    # Print the mean
                    print("Mean of the array:", mean_value)
                    



        iter = iter +1
        cv2.imshow("Camera", cvImage) #Display image
        cv2.imshow("Depth", depth_for_display.get_data())
        key = cv2.waitKey(1)

    cv2.destroyAllWindows()
    # Close the camera
    zed.disable_object_detection()
    zed.disable_positional_tracking()
    zed.close()

if __name__ == "__main__":
    main()

