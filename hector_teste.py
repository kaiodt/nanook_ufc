#! /usr/bin/env python
# coding: utf-8

import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from tf.transformations import euler_from_quaternion

from math import sqrt, sin, cos, pi

class HectorTest(object):

    def __init__(self):

        ### Inicialização do nó ###

        rospy.init_node('klancar_laser_circle')

        # Frequência principal

        self.frequency = rospy.get_param('~frequency', 10)    # [Hz]
        self.Ts = 1.0 / self.frequency

        # Dimensões do robô

        self.wheel_diameter = rospy.get_param('~wheel_diameter', 0.17)      # [m]
        self.wheel_separation = rospy.get_param('~wheel_separation', 0.408) # [m]

        # Pose inicial

        self.x_0 = rospy.get_param('~x_0', 0.0)             # [m]
        self.y_0 = rospy.get_param('~y_0', 0.0)             # [m]
        self.theta_0 = rospy.get_param('~theta_0', 0.0)     # [rad]

        ### Subscribers ###

        # Pose via laser (hector_mapping)

        rospy.Subscriber("slam_out_pose", PoseStamped, self.get_pose)

        # Pose e velocidade da base

        self.x = self.x_0                # Posição no eixo x [m]
        self.y = self.y_0                # Posição no eixo y [m]
        self.theta = self.theta_0        # Orientação [rad]

        # Nova pose e velocidades da base recebidas (válida na próxima iteração)

        self.new_x = self.x_0            # Posição no eixo x [m]
        self.new_y = self.y_0            # Posição no eixo y [m]
        self.new_theta = self.theta_0    # Orientação [rad]


    def get_pose(self, pose):

        """Callback do tópico 'slam_out_pose'.

        Parâmetros:
            pose (PoseStamped): Pose estimada pelo laser (hector_mapping)

        """

        # Convertendo a orientação de Quaternion para Euler

        orientation = pose.pose.orientation

        (row, pitch, yaw) = euler_from_quaternion([orientation.x,
                                                   orientation.y,
                                                   orientation.z,
                                                   orientation.w])

        # Normalizando ângulo

        yaw += self.theta_0

        if yaw > 2 * pi:
            yaw -= 2 * pi
        elif yaw < 0:
            yaw += 2 * pi

        # Atualizando a pose

        self.new_x = pose.pose.position.x + self.x_0    # Posição no eixo x [m]
        self.new_y = pose.pose.position.y + self.y_0    # Posição no eixo y [m]
        self.new_theta = yaw                            # Orientação [rad]


    def spin(self):

        """Laço Principal"""

        while not rospy.is_shutdown():
            # Tempo no início da iteração

            t_iter = rospy.get_time()

            # Atualizando a pose

            self.x = self.new_x
            self.y = self.new_y
            self.theta = self.new_theta

            print("* Pose Atual:")
            print("- x = %.5f m" % self.x)
            print("- y = %.5f m" % self.y)
            print("- theta = %.5f rad" % self.theta)

            # Duração da iteração

            dt_iter = rospy.get_time() - t_iter
            
            print

            # Aguarda o tempo restante para completar o período de amostragem

            rospy.sleep(self.Ts - dt_iter)


if __name__ == '__main__':

    # Instanciação do nó

    teste = HectorTest()

    # Execução do laço principal

    teste.spin()