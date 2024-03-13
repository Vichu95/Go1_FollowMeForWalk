import rospy
from std_msgs.msg import Float64
from gazebo_msgs.srv import GetModelState
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelStates


    
def read_topic_data():
    # Replace 'String' with the appropriate message type
    message = rospy.wait_for_message('/gazebo/model_states', ModelStates)

    rospy.loginfo("Received data: %s", message.name)


if __name__ == '__main__':


    rospy.init_node('position_listner')


    rate = rospy.Rate(10)  # Publish rate (10 Hz in this example)



    command_msg = Float64()

    while not rospy.is_shutdown():

        rate.sleep()

        read_topic_data()
