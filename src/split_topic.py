#!/usr/bin/env python3

import rospy
from custom_msgs.msg import Uwb
from custom_msgs.msg import Luxmetre


def callback_uwb(msg):

    anchorId = msg.anchorId
    #anchorId_distance = msg.range

    if anchorId == anchorList[0]:
        pub_anchor1.publish(msg)
    elif anchorId == anchorList[1]:
        pub_anchor2.publish(msg)
    elif anchorId == anchorList[2]:
        pub_anchor3.publish(msg)
    elif anchorId == anchorList[3]:
        pub_anchor4.publish(msg)
    elif anchorId == anchorList[4]:
        pub_anchor5.publish(msg)
    elif anchorId == anchorList[5]:
        pub_anchor6.publish(msg)
    elif anchorId == anchorList[6]:
        pub_anchor7.publish(msg)
    elif anchorId == anchorList[7]:
        pub_anchor8.publish(msg)

    #dict = {anchorId : anchorId_distance}

def callback_luxmetre(msg):

    lux_number = msg.lux_number
    #measure = msg.measure

    if lux_number == 0:
        pub_luxmetre1.publish(msg)
    elif lux_number == 1:
        pub_luxmetre2.publish(msg)
    elif lux_number == 2:
        pub_luxmetre3.publish(msg)
    elif lux_number == 3:
        pub_luxmetre4.publish(msg)

"""
    # Supposez que data contient les données des 4 capteurs
    sensor1_data = data.sensor1
    sensor2_data = data.sensor2
    sensor3_data = data.sensor3
    sensor4_data = data.sensor4

    # Publier les données individuelles sur les nouveaux topics
    pub_sensor1.publish(sensor1_data)
    pub_sensor2.publish(sensor2_data)
    pub_sensor3.publish(sensor3_data)
    pub_sensor4.publish(sensor4_data)

def get_params_with_keyword(ancre):
    params = rospy.get_param_names()  # Obtient la liste des noms de paramètres
    matching_params = {}

    for param in params:
        if keyword in param:
            param_value = rospy.get_param(param)
            matching_params[param] = param_value

    return matching_params
"""

if __name__ == '__main__':
    rospy.init_node('splitting_node')
    anchorList = []
    # Créer les nouveaux publishers pour les topics individuels
    for i in range(8):  # Utilisez range(6) pour générer les indices valides (0 à 5)
        anchorList.append(rospy.get_param(f"anchor{i + 1}"))
    print(anchorList)
    msg = Uwb()

    pub_anchor1 = rospy.Publisher('/anchor1', Uwb, queue_size=10)
    pub_anchor2 = rospy.Publisher('/anchor2', Uwb, queue_size=10)
    pub_anchor3 = rospy.Publisher('/anchor3', Uwb, queue_size=10)
    pub_anchor4 = rospy.Publisher('/anchor4', Uwb, queue_size=10)
    pub_anchor5 = rospy.Publisher('/anchor5', Uwb, queue_size=10)
    pub_anchor6 = rospy.Publisher('/anchor6', Uwb, queue_size=10)
    pub_anchor7 = rospy.Publisher('/anchor7', Uwb, queue_size=10)
    pub_anchor8 = rospy.Publisher('/anchor8', Uwb, queue_size=10)

    # Souscrire au topic d'origine Uwb
    rospy.Subscriber('/Distance', Uwb, callback_uwb)

    pub_luxmetre1 = rospy.Publisher('/luxmetre1', Luxmetre, queue_size=10)
    pub_luxmetre2 = rospy.Publisher('/luxmetre2', Luxmetre, queue_size=10)
    pub_luxmetre3 = rospy.Publisher('/luxmetre3', Luxmetre, queue_size=10)
    pub_luxmetre4 = rospy.Publisher('/luxmetre4', Luxmetre, queue_size=10)

    # Souscrire au topic d'origine Luxmetre
    rospy.Subscriber('/luxmetre', Luxmetre, callback_luxmetre)

    rospy.spin()
    """
    pub_sensor1 = rospy.Publisher('/sensor1_data', SensorData, queue_size=10)
    pub_sensor2 = rospy.Publisher('/sensor2_data', SensorData, queue_size=10)
    pub_sensor3 = rospy.Publisher('/sensor3_data', SensorData, queue_size=10)
    pub_sensor4 = rospy.Publisher('/sensor4_data', SensorData, queue_size=10)

    

    # Démarrer la boucle ROS
    rospy.spin()
    """