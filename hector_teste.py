#! /usr/bin/env python
# coding: utf-8

import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose2D, Quaternion, Twist
from tf.transformations import euler_from_quaternion

from math import sqrt, sin, cos, pi

class HectorTest(object):

    def __init__(self):

        ### Inicialização do nó ###

        rospy.init_node('hector_test')

        # Frequência principal

        self.frequency = rospy.get_param('~frequency', 10)    # [Hz]
        self.Ts = 1.0 / self.frequency

        # Dimensões do robô

        self.wheel_diameter = rospy.get_param('~wheel_diameter', 0.17)      # [m]
        self.wheel_separation = rospy.get_param('~wheel_separation', 0.408) # [m]

        # Pose inicial

        self.x_0 = rospy.get_param('~x_0', -0.5)             # [m]
        self.y_0 = rospy.get_param('~y_0', -0.5)             # [m]
        self.theta_0 = rospy.get_param('~theta_0', pi/2)     # [rad]

        ### Publishers ###

        # Pose inicial

        self.base_pose_pub = rospy.Publisher('set_base_pose', Pose2D, queue_size = 10)

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

        ### Publicação da pose inicial da base ###

        self.publish_base_pose(self.x_0, self.y_0, self.theta_0)


    def publish_base_pose(self, x, y, theta):

        """Publicação de uma pose para a base no tópico 'set_base_pose'.

        Parâmetros:
            x (float): Posição no eixo x [m]
            y (float): Posição no eixo y [m]
            theta (float): Orientação [rad]

        """

        # Criação da mensagem a ser publicada

        pose = Pose2D()
        pose.x = x
        pose.y = y
        pose.theta = theta

        # Publicação da pose em "set_base_pose"

        print 'Publicando pose inicial'
        self.base_pose_pub.publish(pose)
        print 'Pose inicial publicada'

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

        # Pose atual estimada pelo laser (Em relação ao frame inicial do robô)

        x = pose.pose.position.x    # Posição no eixo x [m]
        y = pose.pose.position.y    # Posição no eixo y [m]
        theta = yaw                 # Orientação [rad]

        # Atualizando a pose (e transformando para o frame inercial)

        self.new_x = cos(self.theta_0) * x - sin(self.theta_0) * y + self.x_0
        self.new_y = sin(self.theta_0) * x + cos(self.theta_0) * y + self.y_0
        self.new_theta = theta + self.theta_0

        # Restringindo a orientação no intervalo [-pi, pi]

        if self.new_theta > pi:
            self.new_theta -= 2*pi

        if self.new_theta < -pi:
            self.new_theta += 2*pi


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
