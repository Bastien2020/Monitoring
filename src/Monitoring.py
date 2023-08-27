#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from custom_msgs.msg import Luxmetre
from custom_msgs.msg import Uwb
from rostopic import ROSTopicHz
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
#import rostopic

"""
rostopic._rostopic_hz(['/uwb/odom','/luxmetre'], window_size=-1, filter_expr=None, use_wtime=False, tcp_nodelay=False)
"""
#_rostopic_hz('/uwb/odom', window_size=-1, filter_expr=None)

r = ROSTopicHz(-1,filter_expr=None)
rospy.init_node('monitoring')
s1 = rospy.Subscriber('/anchor1', Uwb, r.callback_hz, callback_args='/anchor1')
s2 = rospy.Subscriber('/anchor2', Uwb, r.callback_hz, callback_args='/anchor2')
s3 = rospy.Subscriber('/anchor3', Uwb, r.callback_hz, callback_args='/anchor3')
s4 = rospy.Subscriber('/anchor4', Uwb, r.callback_hz, callback_args='/anchor4')
s5 = rospy.Subscriber('/anchor5', Uwb, r.callback_hz, callback_args='/anchor5')
s6 = rospy.Subscriber('/anchor6', Uwb, r.callback_hz, callback_args='/anchor6')
s7 = rospy.Subscriber('/anchor7', Uwb, r.callback_hz, callback_args='/anchor7')
s8 = rospy.Subscriber('/anchor8', Uwb, r.callback_hz, callback_args='/anchor8')
s = rospy.Subscriber('/uwb/odom', Odometry, r.callback_hz, callback_args='/uwb/odom')
l1= rospy.Subscriber('/luxmetre1', Luxmetre, r.callback_hz, callback_args='/luxmetre1')
l2= rospy.Subscriber('/luxmetre2', Luxmetre, r.callback_hz, callback_args='/luxmetre2')
l3= rospy.Subscriber('/luxmetre3', Luxmetre, r.callback_hz, callback_args='/luxmetre3')
l4= rospy.Subscriber('/luxmetre4', Luxmetre, r.callback_hz, callback_args='/luxmetre4')

#L1= rospy.Subscriber('/luxmetre1', Luxmetre, callback_luxmetre1)

pub1 = rospy.Publisher('/anchor_communication', Float32MultiArray, queue_size=10)
pubL1 = rospy.Publisher('/Luxmetre1_comm', Float32MultiArray, queue_size=10)
pubL2 = rospy.Publisher('/Luxmetre2_comm', Float32MultiArray, queue_size=10)
pubL3 = rospy.Publisher('/Luxmetre3_comm', Float32MultiArray, queue_size=10)
pubL4 = rospy.Publisher('/Luxmetre4_comm', Float32MultiArray, queue_size=10)
pub2 = rospy.Publisher("/monitoring", String, queue_size=10)
rate = rospy.Rate(1)
value_min_anchor_communication = rospy.get_param("value_min_anchor_communication")
min_anchor_communication = True



#Function that defines how many anchors are effectively communicating
def number_anchors_comm(publish_state_anchors):
    for value in publish_state_anchors:
        if value is not None:
            publishing_anchors.append([value[1]])

    longueur = len(publishing_anchors)
    print(longueur)

    if longueur < value_min_anchor_communication:
        msg = Float32MultiArray(data=[0])
        pub1.publish(msg)

    else:
        msg = Float32MultiArray(data=[1])
        pub1.publish(msg)

    #print(min_anchor_communication)
def state_comm_luxmetre(publish_state_luxmetres):

    if publish_state_luxmetres[0] is not None:
        lux_data = [1, publish_state_luxmetres[0][0]]
        msg = Float32MultiArray(data=lux_data)
        pubL1.publish(msg)
    else:
        lux_data = 0
        msg = Float32MultiArray(data=lux_data)
        pubL1.publish(msg)

    if publish_state_luxmetres[1] is not None:
        lux_data = [1, publish_state_luxmetres[1][0]]
        msg = Float32MultiArray(data=lux_data)
        pubL2.publish(msg)
    else:
        lux_data = 0
        msg = Float32MultiArray(data=lux_data)
        pubL2.publish(msg)

    if publish_state_luxmetres[2] is not None:
        lux_data = [1, publish_state_luxmetres[2][0]]
        msg = Float32MultiArray(data=lux_data)
        pubL3.publish(msg)
    else:
        lux_data = 0
        msg = Float32MultiArray(data=lux_data)
        pubL3.publish(msg)

    if publish_state_luxmetres[3] is not None:
        lux_data = [1, publish_state_luxmetres[3][0]]
        msg = Float32MultiArray(data=lux_data)
        pubL4.publish(msg)
    else:
        lux_data = 0
        msg = Float32MultiArray(data=lux_data)
        pubL4.publish(msg)




while not rospy.is_shutdown():
    # Evaluate the frequency of publication of each anchor
    publishing_anchors = []
    a1 = r.get_hz('/anchor1')
    a2 = r.get_hz('/anchor2')
    a3 = r.get_hz('/anchor3')
    a4 = r.get_hz('/anchor4')
    a5 = r.get_hz('/anchor5')
    a6 = r.get_hz('/anchor6')
    a7 = r.get_hz('/anchor7')
    a8 = r.get_hz('/anchor8')

    publish_state_anchors = [a1, a2, a3, a4, a5, a6, a7, a8]
    number_anchors_comm(publish_state_anchors)

    # Evaluate the frequency of publication of each anchor
    L1 = r.get_hz('/luxmetre1')
    L2 = r.get_hz('/luxmetre2')
    L3 = r.get_hz('/luxmetre3')
    L4 = r.get_hz('/luxmetre4')

    publish_state_luxmetres = [L1, L2, L3, L4]

    state_comm_luxmetre(publish_state_luxmetres)

    v = r.get_hz('/uwb/odom')
    w = r.get_hz('/luxmetre')

    print("monitoring anchor1", a1)
    print("monitoring anchor2", a2)
    print("monitoring anchor3", a3)
    print("monitoring anchor4", a4)
    print("monitoring anchor5", a5)
    print("monitoring anchor6", a6)
    print("monitoring anchor7", a7)
    print("monitoring anchor8", a8)
    """
    print("monitoring uwb", v)
    print("monitoring luxmetre", w)

    if v == None:
        pub.publish("Probleme publication position robot")
    if w == None:
        pub.publish("Probleme publication luxmetre")
    """
    rate.sleep()


