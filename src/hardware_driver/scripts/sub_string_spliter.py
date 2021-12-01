#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

#global msgs #here we add all the part


def call_back_msgs_splitter(data):
    msgs = data.data
    txt1, txt2, txt3 = msgs.split(",",2)


    print(txt1 + txt3)
    pub1.publish(txt1)
    rate.sleep()




def splitter():
    pub1 = rospy.Publisher('bit1', String, queue_size=10)
    rospy.init_node('hallo_there', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("data_to_be_received", String, call_back_msgs_splitter)





if __name__ == '__main__':
    try:
        splitter()
    except rospy.ROSInterruptException:
        pass
