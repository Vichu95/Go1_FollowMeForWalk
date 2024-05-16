#!/usr/bin/env python

import rospy
from go1_followme.msg import followme_state

import os
from time import sleep
import subprocess
######
## VARIABLES
######
prev_followme_state = ''
prev_followme_state_change_count = 0
prev_followme_state_detected_once = False

STATE_STABLE_DEBOUNCE = 4 # 250ms wait. so 2s



def followme_state_callback(data): 

    global prev_followme_state, prev_followme_state_change_count, prev_followme_state_detected_once

    # Check if the received state is different from the previous one
    if data.data != prev_followme_state:
        # If different, reset the counter and update the previous state
        print("State Changed from ",prev_followme_state, " to ", data.data)
        prev_followme_state_change_count = 0
        prev_followme_state = data.data
        prev_followme_state_detected_once = False
    else:
        # If same as previous state, increment the counter
        prev_followme_state_change_count += 1
        ## Wait for 250ms to make a proper delay
        sleep(0.25)     

        # If the state has persisted for more than STATE_STABLE_DEBOUNCE counts
        if prev_followme_state_change_count >= STATE_STABLE_DEBOUNCE and prev_followme_state_detected_once == False:
            prev_followme_state_detected_once = True


            print("State change is stable now. ")
            stable_state = prev_followme_state

            if(stable_state == 'INIT'):
                print("Announcing INIT 2")
                audio_cmd = "aplay -D plughw:2,0 ../audio/INIT_state_de_female.wav"
                # os.system(audio_cmd)
                print("Finished annoucing INIT")
            
            elif(stable_state == 'FOLLOWING'):
                print("Announcing FOLLOWING 2")
                audio_cmd = "aplay -D plughw:2,0 ../audio/FOLLOWING_state_de_female.wav"
                # os.system(audio_cmd)
                print("Finished annoucing FOLLOWING")
            


if __name__ == '__main__':

    print("Follow me client is starting...")

    rospy.init_node('followme_client_node', anonymous=True)    
    rospy.Subscriber('/followme_state', followme_state, followme_state_callback)



    rospy.spin()
