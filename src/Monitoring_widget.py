#!/usr/bin/env python3

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QLabel, QHBoxLayout
from PyQt5.QtGui import QColor, QPainter
from PyQt5.QtCore import Qt, QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import rospy
import tf
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix
import numpy as np
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import Vector3Stamped, TwistStamped, TwistWithCovarianceStamped, PoseWithCovarianceStamped, Pose, \
    Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from custom_msgs.msg import Luxmetre
from scipy.spatial.transform import Rotation

from custom_msgs.msg import Uwb
from std_msgs.msg import Float32MultiArray


class Visualiser:

    def __init__(self, anchorList):  # , anchorIdList, luxIdList):
        # self.fig, self.axs = plt.subplots(4,1, sharex=True)
        self.fig, self.axs = plt.subplots()
        self.anchorList = anchorList
        # self.fig1, self.axs1 = plt.subplots()
        """
        
        self.luxIdList = luxIdList
        """
        self.dstPlot = {}
        self.dstData = {}
        self.tsDstData = {}
        for p in anchorList:
            self.dstData[p] = []
            self.tsDstData[p] = []
        """
        self.luxPlot = {}
        self.luxData = {}
        self.tsluxData = {}
        for p in anchorIdList:
            # self.dstPlot += [self.axs[2].plot([],[], '-')]
            # self.dstData += [ [] ]
            # self.tsDstData += [ [] ]
            self.dstPlot[p], = self.axs[0, 0].plot([], [], '-', label=f"{p}")
            self.dstData[p] = []
            self.tsDstData[p] = []


        for i in luxIdList:
            self.luxPlot[i], = self.axs[0, 1].plot([], [], '-', label=f"luxmètre {i}")
            self.luxData[i] = []
            self.tsluxData[i] = []
        """
        self.lnEstimate, = self.axs.plot([], [], 'k-', label="Position avec UWB")
        #self.lnOdomFiltered, = self.axs.plot([], [], 'm-', label="Position sans UWB")
        # self.lnthetaEstimate, = self.axs[1, 1].plot([], [], 'k-')
        self.xEstimate_data, self.yEstimate_data = [], []
        """
        self.thetaEstimate_data = []
        self.tstheta_data = []
        """
        self.xFiltered_data, self.yFiltered_data = [], []
        # self.dst0, = self.axs[2].plot([],[], 'r-')
        # self.dst1, = self.axs[2].plot([],[], 'g-')
        # self.dst2, = self.axs[2].plot([],[], 'm-')
        # self.dst3, = self.axs[2].plot([],[], 'b-')

        self.ts_data = []

        # self.dst0_data = []
        # self.dst1_data = []
        # self.dst2_data = []
        # self.dst3_data = []

        self.ts_init = None
        self.ts_init2 = None
        # self.ts_init3 = None

    def plot_init(self):
        """self.axs[0, 0].set_xlim(0, 400)
        self.axs[0, 0].set_ylim(0, 13000)
        # self.axs[1].set_ylim(0, 12000)
        self.axs[0, 0].legend(loc="upper right")
        # self.axs[0].set_title("Distance aux ancres", loc='left')
        self.axs[0, 0].set_xlabel("Temps")
        self.axs[0, 0].set_ylabel("Distance en cm")
        """

        self.axs.set_xlim(-2, 107)
        self.axs.set_ylim(-1, 25)
        self.axs.legend(loc="upper right")
        # self.axs[1].set_title("Estimation position", y = 0.1)
        self.axs.set_xlabel("X")
        self.axs.set_ylabel("Y")
        """plt.text(0.5, 0.9, "Distance aux ancres", horizontalalignment='center', fontsize=10,
                 transform=self.axs[0, 0].transAxes)
        """
        plt.text(0.5, 0.9, "Estimation position", horizontalalignment='center', fontsize=10,
                 transform=self.axs.transAxes)
        return

    def odomEstimate_callback(self, msg):
        # yaw_angle = self.getYaw(msg.pose.pose)
        # self.y_data.append(yaw_angle)
        # x_index = len(self.x_data)
        # self.x_data.append(x_index+1)
        self.xEstimate_data.append(msg.pose.pose.position.x)
        self.yEstimate_data.append(msg.pose.pose.position.y)
        """
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        quat_df = (x, y, z, w)
        rot = Rotation.from_quat(quat_df)
        rot_euler = rot.as_euler('xyz', degrees=True)
        theta = rot_euler[2]
        self.thetaEstimate_data.append(theta)
        
        ts = rospy.get_time()

        if self.ts_init2 == None:
            self.ts_init2 = ts

        trueTS = ts - self.ts_init2
        self.tstheta_data.append(trueTS)
        """
    """
    def odomFiltered_callback(self, msg):
        # yaw_angle = self.getYaw(msg.pose.pose)
        # self.y_data.append(yaw_angle)
        # x_index = len(self.x_data)
        # self.x_data.append(x_index+1)
        self.xFiltered_data.append(msg.pose.pose.position.x)
        self.yFiltered_data.append(msg.pose.pose.position.y)
    """
    def dst_callback(self, msg):
        # yaw_angle = self.getYaw(msg.pose.pose)
        # self.y_data.append(yaw_angle)
        # x_index = len(self.x_data)
        # print("odom message: ", msg.pose.pose.position.x, msg.pose.pose.position.y)
        # ts = float(msg.header.stamp.secs)+float(msg.header.stamp.nsecs)*1e-9

        ts = rospy.get_time()
        id = msg.anchorId
        rge = msg.range

        if self.ts_init2 == None:
            self.ts_init2 = ts

        trueTS = ts - self.ts_init2

        # print("callback: "+id, " ", rge, trueTS)
        self.dstData[id].append(rge)
        self.tsDstData[id].append(trueTS)
        # print(self.dstData[id])
        # print(self.tsDstData[id])

        # return self.dstData[id], global self.tsDstData[id]
        # self.xThetaSpeedTrue_data.append(ts-self.ts_init)
        # self.yThetaSpeedTrue_data.append(msg.twist.twist.angular.z)

    def update_plot(self, frame):
        print("calling updateplot")
        # anchorIdList = anchorList
        # anchorIdList = ["5564182D", "D144436", "55650125", "15648C04","55650997","D1449AF"]
        # self.posOdomX.set_data(self.ts_data, self.x_data)
        # self.posOdomY.set_data(self.ts_data, self.y_data)
        # self.theta.set_data(self.ts_data, self.theta_data)
        # self.thetaEstimate.set_data(self.ts_thetaEstimate_data, self.thetaEstimate_data)
        """
        for p in self.anchorIdList:
            self.dstPlot[p].set_data(self.tsDstData[p], self.dstData[p])
        for i in self.luxIdList:
            self.luxPlot[i].set_data(self.tsluxData[i], self.luxData[i])
        """
        self.lnEstimate.set_data(self.xEstimate_data, self.yEstimate_data)
        # self.lnOdomFiltered.set_data(self.xFiltered_data, self.yFiltered_data)
        # self.lnthetaEstimate.set_data(self.tstheta_data, self.thetaEstimate_data)

        # self.thetaSpeed1.set_data(self.xThetaSpeedTrue_data, self.yThetaSpeedTrue_data)
        # self.thetaSpeed2.set_data(self.xThetaSpeedFiltered_data, self.yThetaSpeedFiltered_data)

        return  # self.posOdomX


class VirtualThetaSensor:
    def __init__(self, naverage, viz=True, quatRobot_initial=None):

        self.ts_init = None

        if viz:
            #print(luxList)
            self.visualizer = Visualiser(anchorList) #anchorList, luxList)
            self.sub1 = rospy.Subscriber('/uwb/odom', Odometry, self.visualizer.odomEstimate_callback)
            #self.sub2 = rospy.Subscriber('/odometry/filtered', Odometry, self.visualizer.odomFiltered_callback)
            self.sub3 = rospy.Subscriber('/Distance', Uwb, self.visualizer.dst_callback)

            # self.sub2 = rospy.Subscriber(topic, Odometry, self.visualizer.odom_callback)
            """if len(anchorList) > 0:
                self.sub3 = rospy.Subscriber(topicDst, Uwb, self.visualizer.dst_callback)
            if len(luxList) > 0:
                self.sub4 = rospy.Subscriber('/luxmetre', Luxmetre, self.visualizer.luxmetre_callback)
            """

            #self.ani = FuncAnimation(self.visualizer.fig, self.visualizer.update_plot, init_func=self.visualizer.plot_init)

class Indicator(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        self._status = False  # Variable pour stocker l'état du critère
        self._value = 0

    def set_status(self, status):
        self._status = status
        self.update()  # Met à jour l'affichage de l'indicateur

    def set_value(self, value):
        self._value = value
        self.update()  # Met à jour l'affichage de l'indicateur

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Définir la couleur de l'indicateur en fonction de l'état du critère
        if self._status == 1:
            painter.setBrush(Qt.green)
        else:
            painter.setBrush(Qt.red)

        # Dessiner l'indicateur comme un cercle rempli
        size = min(self.width(), self.height())
        painter.drawEllipse((self.width() - size) / 2, (self.height() - size) / 2, size, size)

        # Dessiner la valeur à l'intérieur du cercle
        font = painter.font()
        font.setPointSize(12)  # Taille de police
        painter.setFont(font)

        text = str(self._value)  # Convertir la valeur en chaîne
        text_rect = painter.boundingRect(self.rect(), Qt.AlignCenter, text)
        painter.drawText(text_rect, Qt.AlignCenter, text)

class Indicator1(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        self._status = False  # Variable pour stocker l'état du critère
        self._value = 0

    def set_status(self, status):
        self._status = status
        self.update()  # Met à jour l'affichage de l'indicateur


    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Définir la couleur de l'indicateur en fonction de l'état du critère
        if self._status == 1:
            painter.setBrush(Qt.green)
        else:
            painter.setBrush(Qt.red)

        # Dessiner l'indicateur comme un cercle rempli
        size = min(self.width(), self.height())
        painter.drawEllipse((self.width() - size) / 2, (self.height() - size) / 2, size, size)




def anchor_communication_callback(msg):
    #anchor_communication = msg.data
    indicator.set_status(msg.data[0])

"""
def callback_luxmetre1_1(msg):
    indicator_lux_1.set_status(msg.data[0])

def callback_luxmetre1_2(msg):
    #indicator_lux_1.set_value(msg.measure)
    luxmetre_data[0] = msg.measure


def callback_luxmetre2_1(msg):
    indicator_lux_2.set_status(msg.data[0])

def callback_luxmetre2_2(msg):
    #indicator_lux_2.set_value(msg.measure)
    luxmetre_data[1] = msg.measure

def callback_luxmetre3_1(msg):
    indicator_lux_3.set_status(msg.data[0])

def callback_luxmetre3_2(msg):
    #indicator_lux_3.set_value(msg.measure)
    luxmetre_data[2] = msg.measure

def callback_luxmetre4_1(msg):
    indicator_lux_4.set_status(msg.data[0])

def callback_luxmetre4_2(msg):
    #indicator_lux_4.set_value(msg.measure)
    luxmetre_data[3] = msg.measure
"""

class LuxmetreUpdater:
    def __init__(self, indicator_lux_1, indicator_lux_2, indicator_lux_3, indicator_lux_4):
        self.indicator_lux_1 = indicator_lux_1
        self.indicator_lux_2 = indicator_lux_2
        self.indicator_lux_3 = indicator_lux_3
        self.indicator_lux_4 = indicator_lux_4
        self.luxmetre_status = [0, 0, 0, 0]
        self.luxmetre_data = [0, 0, 0, 0]  # Initialiser avec des valeurs par défaut
        rospy.Subscriber('/Luxmetre1_comm', Float32MultiArray, self.callback_luxmetre1_1)
        rospy.Subscriber('/luxmetre1', Luxmetre, self.callback_luxmetre1_2)

        rospy.Subscriber('/Luxmetre2_comm', Float32MultiArray, self.callback_luxmetre2_1)
        rospy.Subscriber('/luxmetre2', Luxmetre, self.callback_luxmetre2_2)

        rospy.Subscriber('/Luxmetre3_comm', Float32MultiArray, self.callback_luxmetre3_1)
        rospy.Subscriber('/luxmetre3', Luxmetre, self.callback_luxmetre3_2)

        rospy.Subscriber('/Luxmetre4_comm', Float32MultiArray, self.callback_luxmetre4_1)
        rospy.Subscriber('/luxmetre4', Luxmetre, self.callback_luxmetre4_2)

        self.current_indicator_index = 0
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_indicators)
        self.timer.start(100)  # Démarrer le timer avec un intervalle de 100 ms

    def callback_luxmetre1_1(self, msg):
        self.luxmetre_status[0] = msg.data[0]

    def callback_luxmetre1_2(self, msg):
        # indicator_lux_1.set_value(msg.measure)
        self.luxmetre_data[0] = msg.measure

    def callback_luxmetre2_1(self, msg):
        self.luxmetre_status[1] = msg.data[0]

    def callback_luxmetre2_2(self, msg):
        # indicator_lux_2.set_value(msg.measure)
        self.luxmetre_data[1] = msg.measure

    def callback_luxmetre3_1(self, msg):
        self.luxmetre_status[2] = msg.data[0]

    def callback_luxmetre3_2(self, msg):
        # indicator_lux_3.set_value(msg.measure)
        self.luxmetre_data[2] = msg.measure

    def callback_luxmetre4_1(self, msg):
        self.luxmetre_status[3] = msg.data[0]

    def callback_luxmetre4_2(self, msg):
        # indicator_lux_4.set_value(msg.measure)
        self.luxmetre_data[3] = msg.measure


    def update_indicators(self):
        """
        if self.current_indicator_index == 0:
            # Mettre à jour le graphique
            self.visualizer.update_plot(None)
        """
        if self.current_indicator_index ==0:
            # Mettre à jour l'indicateur correspondant
            self.indicator_lux_1.set_status(self.luxmetre_status[0])
            self.indicator_lux_1.set_value(self.luxmetre_data[0])
        elif self.current_indicator_index ==1:
            # Mettre à jour l'indicateur correspondant
            self.indicator_lux_2.set_status(self.luxmetre_status[1])
            self.indicator_lux_2.set_value(self.luxmetre_data[1])
        elif self.current_indicator_index ==2:
            # Mettre à jour l'indicateur correspondant
            self.indicator_lux_3.set_status(self.luxmetre_status[2])
            self.indicator_lux_3.set_value(self.luxmetre_data[2])
        elif self.current_indicator_index ==3:
            # Mettre à jour l'indicateur correspondant
            self.indicator_lux_4.set_status(self.luxmetre_status[3])
            self.indicator_lux_4.set_value(self.luxmetre_data[3])
        self.current_indicator_index = (self.current_indicator_index + 1) % 4


if __name__ == '__main__':
    anchorList = []
    for i in range(1, 9):
        anchorList.append(str(rospy.get_param(f"anchor{i}")))

    app = QApplication(sys.argv)
    # Création de la fenêtre principale
    window = QMainWindow()

    # Création du widget central
    central_widget = QWidget()
    window.setCentralWidget(central_widget)

    # Création du layout vertical
    layout = QVBoxLayout(central_widget)
    layout.setSpacing(5)  # Ajustez la valeur selon vos besoins


    # Ajout du graphique dans le layout
    rospy.init_node('theta_virtual_sensor')
    vs = VirtualThetaSensor(500)
    canvas = FigureCanvas(vs.visualizer.fig)
    layout.addWidget(canvas)

   # Ajout de l'indicateur Communication des ancres dans le layout
    indicator_label = QLabel('Communication Ancres')
    indicator_label.setAlignment(Qt.AlignHCenter)
    layout.addWidget(indicator_label)

    sub = rospy.Subscriber('/anchor_communication', Float32MultiArray, anchor_communication_callback)

    indicator = Indicator1()
    layout.addWidget(indicator)
    indicator.set_status(1)

    #création autre layout
    layout_luxmeters = QHBoxLayout()

    # Ajouter les indicateurs luxmètre dans le layout_luxmeters

    luxmeter_layout_1 = QVBoxLayout()
    luxmeter_label_1 = QLabel('Luxmètre 1')
    luxmeter_label_1.setAlignment(Qt.AlignHCenter)
    luxmeter_layout_1.addWidget(luxmeter_label_1)

    indicator_lux_1 = Indicator()
    luxmeter_layout_1.addWidget(indicator_lux_1)
    layout_luxmeters.addLayout(luxmeter_layout_1)

    luxmeter_layout_2 = QVBoxLayout()
    luxmeter_label_2 = QLabel('Luxmètre 2')
    luxmeter_label_2.setAlignment(Qt.AlignHCenter)
    luxmeter_layout_2.addWidget(luxmeter_label_2)

    indicator_lux_2 = Indicator()
    luxmeter_layout_2.addWidget(indicator_lux_2)
    layout_luxmeters.addLayout(luxmeter_layout_2)

    luxmeter_layout_3 = QVBoxLayout()
    luxmeter_label_3 = QLabel('Luxmètre 3')
    luxmeter_label_3.setAlignment(Qt.AlignHCenter)
    luxmeter_layout_3.addWidget(luxmeter_label_3)

    indicator_lux_3 = Indicator()
    luxmeter_layout_3.addWidget(indicator_lux_3)
    layout_luxmeters.addLayout(luxmeter_layout_3)

    luxmeter_layout_4 = QVBoxLayout()
    luxmeter_label_4 = QLabel('Luxmètre 4')
    luxmeter_label_4.setAlignment(Qt.AlignHCenter)
    luxmeter_layout_4.addWidget(luxmeter_label_4)

    indicator_lux_4 = Indicator()
    luxmeter_layout_4.addWidget(indicator_lux_4)
    layout_luxmeters.addLayout(luxmeter_layout_4)

    # Create a LuxmeterUpdater instance
    luxmeter_updater = LuxmetreUpdater(indicator_lux_1, indicator_lux_2, indicator_lux_3, indicator_lux_4)
    """
    #rospy.Subscriber('/Luxmetre1_comm', Float32MultiArray, callback_luxmetre1_1)
    rospy.Subscriber('/luxmetre1', Luxmetre, callback_luxmetre1_2)

    #rospy.Subscriber('/Luxmetre2_comm', Float32MultiArray, callback_luxmetre2_1)
    rospy.Subscriber('/luxmetre2', Luxmetre, callback_luxmetre2_2)

    #rospy.Subscriber('/Luxmetre3_comm', Float32MultiArray, callback_luxmetre3_1)
    rospy.Subscriber('/luxmetre3', Luxmetre, callback_luxmetre3_2)

    #rospy.Subscriber('/Luxmetre4_comm', Float32MultiArray, callback_luxmetre4_1)
    rospy.Subscriber('/luxmetre4', Luxmetre, callback_luxmetre4_2)
    """
    # Ajouter le layout_luxmeters dans le layout principal
    layout.addLayout(layout_luxmeters)

    """
    #Limite entre les deux modèles
    layout_lux1 = QVBoxLayout()

    # Ajout de l'indicateur Communication du luxmètre 1 dans le layout
    indicator_label_lux1 = QLabel('Communication Luxmètre 1')
    indicator_label_lux1.setAlignment(Qt.AlignHCenter)
    layout_lux1.addWidget(indicator_label_lux1)

    rospy.Subscriber('/Luxmetre1_comm', Float32MultiArray, callback_luxmetre1_1)
    rospy.Subscriber('/luxmetre1', Luxmetre, callback_luxmetre1_2)

    indicator_lux1 = Indicator()
    layout_lux1.addWidget(indicator_lux1)
    indicator_lux1.set_value(500)

    # Ajouter le layout_lux1 dans le layout principal
    layout.addLayout(layout_lux1)

    indicator_lux_list.append(indicator_lux1)

    """

    animation = FuncAnimation(vs.visualizer.fig, vs.visualizer.update_plot, init_func=vs.visualizer.plot_init)
    # Affichage de la fenêtre
    window.setWindowTitle('Mon application')
    window.show()

    # Exécution de l'application
    sys.exit(app.exec_())
"""
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Créer l'indicateur
        self.indicator = Indicator()
        layout.addWidget(self.indicator)


        # Simuler un changement d'état de l'indicateur
        # Changez True en False pour voir la couleur rouge
        self.indicator.set_status(True)


if __name__ == '__main__':

    app = QApplication(sys.argv)
    window = MainWindow()
    window.setWindowTitle('Indicateur Visuel')
    window.show()
    rospy.init_node('theta_virtual_sensor')
    vs = Virtua
"""