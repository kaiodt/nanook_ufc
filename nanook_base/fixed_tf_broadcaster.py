#! /usr/bin/env python
# coding: utf-8

###########################################################################################
###########################################################################################
## Projeto: Nanook UFC
## Autor: Kaio Douglas Teófilo Rocha
## Email: kaiodtr@gmail.com
###########################################################################################
## Arquivo: Nó Publicador de Transformações Fixas
## Revisão: 1 [26/02/2017]
###########################################################################################
###########################################################################################

import rospy
import tf

###########################################################################################
class FixedTransformations(object):
###########################################################################################

    #######################################################################################

    def __init__(self):

        """Construtor

        Parâmetros:
            publishing_frequency (float): Frequência de publicação [Hz]

            laser_x (float): Posição do laser no eixo x em relação ao centro da base [m]
            laser_y (float): Posição do laser no eixo y em relação ao centro da base [m]
            laser_z (float): Posição do laser no eixo z em relação ao centro da base [m]

            base_frame_id (str): Nome do sistema de coordenadas da base
            laser_frame_id (str): Nome do sistema de coordenadas do laser

        """

        ### Inicialização do nó ###

        rospy.init_node('fixed_tf_broadcaster')

        ### Definição dos parâmetros ###

        # Frequência de publicação

        self.publishing_frequency = rospy.get_param('~publishing_frequency', 25)    # [Hz]
        self.Ts = 1.0 / self.publishing_frequency

        # Posição do laser (em relação ao centro da base)

        self.laser_x = rospy.get_param('~laser_x', 0.2)     # [m]
        self.laser_y = rospy.get_param('~laser_y', 0.0)     # [m]
        self.laser_z = rospy.get_param('~laser_z', 0.4)     # [m]

        # Nomenclatura dos frames

        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')
        self.laser_frame_id = rospy.get_param('~laser_frame_id', 'laser_scanner')

        ### Broadcaster ###

        self.tf_broadcaster = tf.TransformBroadcaster()

    #######################################################################################

    #######################################################################################

    def publish_laser(self):

        """Publicação da transformação laser_scanner -> base_link em 'tf'."""

        self.tf_broadcaster.sendTransform((self.laser_x, self.laser_y, self.laser_z),
                                          (0.0, 0.0, 0.0, 1.0),
                                          rospy.Time.now(),
                                          self.laser_frame_id,
                                          self.base_frame_id)

    #######################################################################################

    #######################################################################################

    def spin(self):

        """Laço Principal"""

        while not rospy.is_shutdown():

            # Tempo no início da iteração

            t_iter = rospy.get_time()

            # Publicação da transformação do laser

            self.publish_laser()

            # Duração da iteração

            dt_iter = rospy.get_time() - t_iter

            # Aguarda o tempo restante para completar o período de amostragem

            rospy.sleep(self.Ts - dt_iter)

    #######################################################################################

###########################################################################################
### EXECUÇÃO
###########################################################################################

if __name__ == '__main__':

    # Instanciação do publicador

    fixed_trasformations = FixedTransformations()

    # Execução do laço principal

    fixed_trasformations.spin()

###########################################################################################
###########################################################################################