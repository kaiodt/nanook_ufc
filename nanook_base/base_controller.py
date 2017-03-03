#! /usr/bin/env python
# coding: utf-8

###########################################################################################
###########################################################################################
## Projeto: Nanook UFC
## Autor: Kaio Douglas Teófilo Rocha
## Email: kaiodtr@gmail.com
###########################################################################################
## Arquivo: Nó Controlador de Base
## Revisão: 2 [03/03/2017]
###########################################################################################
###########################################################################################

import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist

from base_driver import NanookBase

from math import sin, cos, pi

###########################################################################################
class BaseController(object):
###########################################################################################

    #######################################################################################

    def __init__(self):

        """Construtor

        Parâmetros:
            base_frequency (float): Frequência de atualização da base [Hz]

            wheel_diameter (float): Diâmetro das rodas [m]
            wheel_separation (float): Separação entre duas rodas [m]

            base_serial_port (str): Porta serial da base (FRDM K64F)
            base_baud_rate (int): Velocidade da conexão serial com a base

            x_0 (float): Posição inicial no eixo x [m]
            y_0 (float): Posição inicial no eixo y [m]
            theta_0 (float): Orientação inicial [rad]

            base_frame_id (str): Nome do sistema de coordenadas da base
            odom_frame_id (str): Nome do sistema de coordenadas da odometria

        """

        ### Inicialização do nó ###

        rospy.init_node('base_controller')

        ### Definição dos parâmetros ###

        # Frequência principal

        self.base_frequency = rospy.get_param('~base_frequency', 50)    # [Hz]
        self.Ts = 1.0 / self.base_frequency

        # Dimensões do robô

        self.wheel_diameter = rospy.get_param('~wheel_diameter', 0.17)      # [m]
        self.wheel_separation = rospy.get_param('~wheel_separation', 0.405) # [m]

        # Driver do controlador da base (FRDM K64F)

        self.base_serial_port = rospy.get_param('~base_serial_port', '/dev/ttyACM0')
        self.base_baud_rate = rospy.get_param('~base_baud_rate', 115200)

        # Pose inicial

        x_0 = rospy.get_param('~x_0', 0.0)          # [m]
        y_0 = rospy.get_param('~y_0', 0.0)          # [m]
        theta_0 = rospy.get_param('~theta_0', 0.0)  # [rad]

        # Nomenclatura dos sistemas de coordenadas

        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')

        ### Conexão com o driver controlador da base (FRDM K64F) ###

        # Instanciação 

        self.base = NanookBase(self.base_serial_port, self.base_baud_rate)

        # Conexão

        self.base.connect()

        # Inicialização

        self.base.reset_base()
        self.base.set_pose(x_0, y_0, theta_0)
        rospy.sleep(0.5)

        ### Publishers ###

        # Odometria

        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size = 10)

        # Transformação base_link -> odom

        self.odom_broad = tf.TransformBroadcaster()

        ### Subscribers ###

        # Velocidade de referência

        rospy.Subscriber('cmd_vel', Twist, self.set_speed_ref)

        ### Variáveis ###

        # Novas velocidades de referência da base (válidas na próxima iteração)

        self.new_v_ref = 0.0    # Velocidade linear [m/s]
        self.new_w_ref = 0.0    # Velocidade angular [rad/s]

        # Velocidades atuais de referência da base

        self.v_ref = 0.0    # Velocidade linear [m/s]
        self.w_ref = 0.0    # Velocidade angular [rad/s]

        # Pose e velocidade atuais da base do robô

        self.x = x_0            # Posição no eixo x [m]
        self.y = y_0            # Posição no eixo y [m]
        self.theta = theta_0    # Orientação [rad]

        self.v = 0.0            # Velocidade linear [m/s]
        self.w = 0.0            # Velocidade angular [rad/s]

        # Controle de tempo

        self.last_time = rospy.get_time()   # Instante de tempo da última leitura

    #######################################################################################

    #######################################################################################

    def set_speed_ref(self, reference):

        """Callback do tópico 'cmd_vel'."""

        # Recebimento de referência de velocidade tipo Twist (linear + angular).
        # Estas referências serão válidas na próxima iteração.

        self.new_v_ref = reference.linear.x         # Velocidade linear em x [m/s]
        self.new_w_ref = reference.angular.z        # Velocidade angular em z [rad/s]

        print('* Novas referências recebidas:')
        print('- v =  %f m/s' % self.new_v_ref)
        print('- w =  %f rad/s' % self.new_w_ref)

    #######################################################################################

    #######################################################################################

    def publish_odometry(self):

        """Publicação da odometria nos tópicos 'odom' e 'tf'."""

        # Representação da orientação no formato quaternion

        orient_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, self.theta)

        # Decomposição da velocidade linear da base em x e y

        v_x = self.v * cos(self.theta)  # [m/s]
        v_y = self.v * sin(self.theta)  # [m/s]

        # Publicação em "odom"

        odom = Odometry()
        odom.header.stamp = rospy.Time.from_sec(self.last_time)
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*orient_quaternion)
        odom.twist.twist.linear.x = v_x
        odom.twist.twist.linear.y = v_y
        odom.twist.twist.angular.z = self.w

        self.odom_pub.publish(odom)

        # Publicação em "tf" via broadcaster

        self.odom_broad.sendTransform((self.x, self.y, 0),
                                      orient_quaternion,
                                      rospy.Time.from_sec(self.last_time),
                                      self.base_frame_id,
                                      self.odom_frame_id)

    #######################################################################################

    #######################################################################################

    def spin(self):

        """Laço Principal

        Em cada iteração, a nova referência de velocidade é enviada para a base e as 
        informações atualizadas de odometria são recebidas.

        """

        while not rospy.is_shutdown():

            # Tempo no início da iteração

            t_iter = rospy.get_time()

            print('*** Nova Iteração ***')

            # Atualização das velocidades de referência

            self.v_ref = self.new_v_ref     # Velocidade linear [m/s]
            self.w_ref = self.new_w_ref     # Velocidade angular [m/s]

            # Envio da nova referência para o controlador da base

            self.base.set_vel_ref(self.v_ref, self.w_ref)

            # Tempo para finalizar envio da referência e liberar serial

            rospy.sleep(0.005)

            # Recebimento dos dados de odometria da base

            (self.x, self.y, self.theta, self.v, self.w) = self.base.get_odometry()

            # Intervalo de tempo entre leituras (deve ser próximo de Ts)

            self.dt_readings = rospy.get_time() - self.last_time
            self.last_time = rospy.get_time()

            print('* Intervalo entre Leituras: %.5f s' % self.dt_readings)

            print('* Odometria:')
            print('- x = %.5f m' % self.x)
            print('- y = %.5f m' % self.y)
            print('- 0 = %.5f graus' % (self.theta * 180 / pi))
            print('- v = %.5f m/s' % self.v)
            print('- w = %.5f rad/s' % self.w)

            # Publicação da odometria

            self.publish_odometry()

            # Duração da iteração

            dt_iter = rospy.get_time() - t_iter

            print('* Duração da Iteração: %.5f s' % dt_iter)
            print

            # Aguardar o tempo restante para completar o período de amostragem

            rospy.sleep(self.Ts - dt_iter)

    #######################################################################################

    #######################################################################################

    def finalize(self):

        """Finalização do nó."""

        # Reinicialização da base

        self.base.reset_base()

        # Encerramento da conexão com a base

        self.base.disconnect()

    #######################################################################################

###########################################################################################
### EXECUÇÃO
###########################################################################################

if __name__ == '__main__':

    # Instanciação do controlador de base

    base_controller = BaseController()

    # Execução do laço principal

    base_controller.spin()

    # Finalização

    base_controller.finalize()

###########################################################################################
###########################################################################################