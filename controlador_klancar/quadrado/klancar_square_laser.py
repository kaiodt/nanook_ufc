#! /usr/bin/env python
# coding: utf-8

###########################################################################################
###########################################################################################
## Projeto: Nanook UFC
## Autor: Kaio Douglas Teófilo Rocha
## Email: kaiodtr@gmail.com
###########################################################################################
## Arquivo: Nó Controlador de Trajetória de Klancar (Quadrado)
## Revisão: 1 [05/04/2017]
###########################################################################################
###########################################################################################

import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from tf.transformations import euler_from_quaternion

from math import sqrt, sin, cos, pi
from numpy import sign

from os.path import expanduser

###########################################################################################
class KlancarSquare(object):
###########################################################################################

    #######################################################################################

    def __init__(self):

        ### Inicialização do nó ###

        rospy.init_node('klancar_laser_square')

        ### Definição dos parâmetros ###

        # Parâmetros definidos pelo usuário

        self.ensaio = int(raw_input('Número do Ensaio: '))
        self.side = float(raw_input('Comprimento do Lado [m]: '))
        self.base_lin_speed = float(raw_input('Velocidade Linear da Base [m/s]: '))

        # Frequência principal

        self.frequency = rospy.get_param('~frequency', 10)    # [Hz]
        self.Ts = 1.0 / self.frequency

        # Dimensões do robô

        self.wheel_diameter = rospy.get_param('~wheel_diameter', 0.17)      # [m]
        self.wheel_separation = rospy.get_param('~wheel_separation', 0.408) # [m]

        # Parâmetros do controlador

        self.ksi = rospy.get_param("~ksi", 0.8)
        self.g = rospy.get_param("~g", 30.0)
        self.w_n = rospy.get_param("~w_n", 
            sqrt(self.g * pow(self.base_lin_speed, 2)))

        # Valores máximos de velocidade

        self.v_max = rospy.get_param("~v_max", 0.5)
        self.w_max = rospy.get_param("~w_max", 1.0)

        # Pose inicial

        self.x_0 = rospy.get_param('~x_0', 0.0)             # [m]
        self.y_0 = rospy.get_param('~y_0', 0.0)             # [m]
        self.theta_0 = rospy.get_param('~theta_0', 0.0)     # [rad]

        ### Listas com os dados do ensaio ###

        self.time_list = []         # Tempo [s]
        self.x_ref_list = []        # Referência de posição no eixo x [m]  
        self.y_ref_list = []        # Referência de posição no eixo y [m]
        self.theta_ref_list = []    # Referência de orientação [rad]
        self.v_ref_list = []        # Referência de velocidade linear da base [m/s]
        self.w_ref_list = []        # Referência de velocidade angular da base [rad/s]
        self.x_list = []            # Posição no eixo x [m]
        self.y_list = []            # Posição no eixo y [m]
        self.theta_list = []        # Orientação [rad]
        self.v_list = []            # Velocidade linear da base [m/s]
        self.w_list = []            # Velocidade angular da base [rad/s]
        self.u_v_list = []          # Comando de velocidade linear da base [m/s]
        self.u_w_list = []          # Comando de velocidade angulat da base [rad/s]
        self.x_error_list = []      # Erro de posição no eixo x [m]
        self.y_error_list = []      # Erro de posição no eixo y [m]
        self.theta_error_list = []  # Erro de orientação [rad]

        ### Gerando a trajetória ###

        self.generate_square()

        # Número de iterações

        self.N_iters = len(self.x_ref_list)

        ### Publishers ###

        # Velocidades de referência da base

        self.speed_ref_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)

        ### Subscribers ###

        # Pose via laser (hector_mapping)

        rospy.Subscriber("slam_out_pose", PoseStamped, self.get_pose)

        # Velocidades da base via encoders

        rospy.Subscriber("odom", Odometry, self.get_speeds)

        ### Variáveis ###

        # Índice da trajetória de referência

        self.index = 0

        # Pose e velocidade da base

        self.x = self.x_0                   # Posição no eixo x [m]
        self.y = self.y_0                   # Posição no eixo y [m]
        self.theta = self.theta_0           # Orientação [rad]

        self.v = 0.0        # Velocidade linear da base [m/s]
        self.w = 0.0        # Velocidade angular da base [rad/s]

        # Nova pose e velocidades da base recebidas (válida na próxima iteração)

        self.new_x = self.x_0               # Posição no eixo x [m]
        self.new_y = self.y_0               # Posição no eixo y [m]
        self.new_theta = self.theta_0       # Orientação [rad]

        self.new_v = 0.0    # Velocidade linear da base [m/s]
        self.new_w = 0.0    # Velocidade angular da base [rad/s]

        # Sinais de controle de velocidades linear e angular a serem enviados para a base

        self.u_v = 0.0      # Comando de velocidade linear da base [m/s]
        self.u_w = 0.0      # Comando de velocidade angular da base [rad/s]

    #######################################################################################

    #######################################################################################

    def generate_square(self):

        """Geração do quadrado que deverá ser seguido."""

        # Número de pontos por lado

        Nl = int(self.side / (self.base_lin_speed * self.Ts)) + 1

        # Número total de pontos

        N = 4 * Nl + 1

        # Atualizando as listas com parâmetros constantes da trajetória

        self.v_ref_list = [self.base_lin_speed] * N
        self.w_ref_list = [0.0] * N        

        # Inicializando as listas relativas à pose

        self.x_ref_list = [0.0] * N
        self.y_ref_list = [0.0] * N
        self.theta_ref_list = [0.0] * N

        # Gerando os pontos de referência de pose

        # Trecho 1
        for k in range(1, Nl+1):
            self.x_ref_list[k] = self.x_ref_list[k-1] + self.base_lin_speed * self.Ts
            self.y_ref_list[k] = self.y_ref_list[k-1]
            self.theta_ref_list[k] = 0

        # Trecho 2
        for k in range(Nl+1, 2*Nl+1):
            self.x_ref_list[k] = self.x_ref_list[k-1]
            self.y_ref_list[k] = self.y_ref_list[k-1] + self.base_lin_speed * self.Ts
            self.theta_ref_list[k] = pi/2

        # Trecho 3
        for k in range(2*Nl+1, 3*Nl+1):
            self.x_ref_list[k] = self.x_ref_list[k-1] - self.base_lin_speed * self.Ts
            self.y_ref_list[k] = self.y_ref_list[k-1]
            self.theta_ref_list[k] = pi

        # Trecho 4
        for k in range(3*Nl+1, 4*Nl+1):
            self.x_ref_list[k] = self.x_ref_list[k-1]
            self.y_ref_list[k] = self.y_ref_list[k-1] - self.base_lin_speed * self.Ts
            self.theta_ref_list[k] = 3 * pi/2

    #######################################################################################

    #######################################################################################

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

    #######################################################################################

    #######################################################################################

    def get_speeds(self, odometry):

        """Callback do tópico 'odom'. Obtenção das velocidades linear e angular da base.

        Parâmetros:
            odometry (Odometry): Odometria formnecida pelos encoders

        """

        # Convertendo a velocidade para o frame da base (somente no eixo x)

        # v = odometry.twist.twist.linear.x / cos(self.new_theta)

        v = sqrt(pow(odometry.twist.twist.linear.x, 2) + \
                 pow(odometry.twist.twist.linear.y, 2))

        # Atualizando as velocidades

        self.new_v = v
        self.new_w = odometry.twist.twist.angular.z

    #######################################################################################

    #######################################################################################

    def control_pose(self):

        """Controlador de trajetória de Klancar."""

        # Cálculo dos erros

        x_error = self.x_ref_list[self.index] - self.x
        y_error = self.y_ref_list[self.index] - self.y
        theta_error = self.theta_ref_list[self.index] - self.theta

        if theta_error < - pi:
            theta_error = - theta_error

        # Atualização das listas de erros

        self.x_error_list.append(x_error)
        self.y_error_list.append(y_error)
        self.theta_error_list.append(theta_error)

        # Fatores e

        e1 = cos(self.theta) * x_error + sin(self.theta) * y_error
        e2 = cos(self.theta) * y_error - sin(self.theta) * x_error
        e3 = theta_error

        # Ganhos

        k1 = 2 * self.ksi * self.w_n
        k2 = self.g * abs(self.v_ref_list[self.index])
        k3 = k1

        v1 = -k1 * e1
        v2 = -sign(self.v_ref_list[self.index]) * k2 * e2 - k3 * e3

        # Velocidades linear e angular de referência para a base

        self.u_v = self.v_ref_list[self.index] * cos(e3) - v1
        self.u_w = self.w_ref_list[self.index] - v2

        # Limitação das velocidades

        if self.u_v > self.v_max:
            self.u_v = self.v_max

        if self.u_v < -self.v_max:
            self.u_v = -self.v_max

        if self.u_w > self.w_max:
            self.u_w = self.w_max

        if self.u_w < -self.w_max:
            self.u_w = -self.w_max

    #######################################################################################

    #######################################################################################

    def publish_speed_refs(self):

        """Publicação das referências de velocidades linear e angular da base no
           tópico 'cmd_vel'."""

        # Criação da mensagem tipo Twist

        reference = Twist()
        reference.linear.x = self.u_v      # Velocidade linear em x [m/s]
        reference.angular.z = self.u_w     # Velocidade angular em z [rad/s]

        # Publicação da referência

        self.speed_ref_pub.publish(reference)

    #######################################################################################

    #######################################################################################

    def spin(self):

        """Laço Principal"""

        while not rospy.is_shutdown() and (self.index < self.N_iters):

            # Tempo no início da iteração

            t_iter = rospy.get_time()

            print("*** Iteração % d ***" % (self.index + 1))

            # Atualizando a pose e velocidade da base para as últimas recebidas

            self.x = self.new_x
            self.y = self.new_y
            self.theta = self.new_theta

            self.v = self.new_v
            self.w = self.new_w

            print("* Pose de Referência:")
            print("- x_ref = %.5f m" % self.x_ref_list[self.index])
            print("- y_ref = %.5f m" % self.y_ref_list[self.index])
            print("- theta_ref = %.5f rad" % self.theta_ref_list[self.index])
            print("- v_ref = %.5f m/s" % self.v_ref_list[self.index])
            print("- w_ref = %.5f rad/s" % self.w_ref_list[self.index])

            print("* Pose Atual:")
            print("- x = %.5f m" % self.x)
            print("- y = %.5f m" % self.y)
            print("- theta = %.5f rad" % self.theta)
            print("- v = %.5f m/s" % self.v)
            print("- w = %.5f rad/s" % self.w)

            # Iteração do controle de pose
        
            self.control_pose()

            # Publicação das velocidades de referência para o nó controlador de base

            self.publish_speed_refs()

            # Atualização das listas com os novos dados

            self.time_list.append(t_iter)
            self.x_list.append(self.x)
            self.y_list.append(self.y)
            self.theta_list.append(self.theta) 
            self.v_list.append(self.v)
            self.w_list.append(self.w)
            self.u_v_list.append(self.u_v)
            self.u_w_list.append(self.u_w)

            # Incremento do índice

            self.index += 1

            # Duração da iteração

            dt_iter = rospy.get_time() - t_iter

            print("* Duração da Iteração: %.5f s" % dt_iter)
            print

            # Aguarda o tempo restante para completar o período de amostragem

            rospy.sleep(self.Ts - dt_iter)

    #######################################################################################

    #######################################################################################

    def write_file(self):            
        
        """Salva os dados do ensaio em um arquivo."""

        ### Arquivo para salvar os dados ###

        # Diretório

        home = expanduser('~')
        path = home + '/ros_catkin_ws/src/nanook_ufc/controlador_klancar'
        path += '/quadrado/resultados/ensaio_%d.txt' % self.ensaio

        # Abertura do arquivo

        data_file = open(path, 'w')

        # Registro dos parâmetros do ensaio

        params_str  = '### Ensaio do Controlador Klancar com Laser (Quadrado) ###\n\n'
        params_str += '# Parâmetros #\n\n'
        params_str += '# Número do Ensaio: %d\n' % self.ensaio
        params_str += '# Frequência: %.1f Hz\n' % self.frequency
        params_str += '# Lado: %.2f m\n' % self.side
        params_str += '# Velocidade Linear da Base: %.2f m/s\n' % self.base_lin_speed
        params_str += '# Parâmetro ksi: %.2f\n' % self.ksi
        params_str += '# Parâmetro w_n: %.2f\n' % self.w_n
        params_str += '# Parâmetro g: %.2f\n\n' % self.g

        data_file.write(params_str)

        # Registro das listas de dados

        # Desconto do instante inicial (deve ser 0 s)

        t0 = self.time_list[0]
        self.time_list = [(t - t0) for t in self.time_list]

        # Salvando resultados no arquivo

        for idx in range(len(self.time_list)):
            data_file.write('* {0} {1} {2} {3} {4} {5} {6} {7} {8} {9} {10} \
                               {11} {12} {13} {14} {15} {16}\n'.format(
                idx + 1,                        # Amostra
                self.time_list[idx],            # Tempo [s]
                self.x_ref_list[idx],           # Referência de posição no eixo x [m]
                self.y_ref_list[idx],           # Referência de posição no eixo y [m]
                self.theta_ref_list[idx],       # Referência de orientação [rad]
                self.v_ref_list[idx],           # Referência de velocidade linear da base [m/s]
                self.w_ref_list[idx],           # Referência de velocidade angular da base [rad/s]
                self.x_list[idx],               # Posição no eixo x [m]
                self.y_list[idx],               # Posição no eixo y [m]
                self.theta_list[idx],           # Orientação [rad]
                self.v_list[idx],               # Velocidade linear da base [m/s]
                self.w_list[idx],               # Velocidade angular da base [rad/s]
                self.u_v_list[idx],             # Comando de velocidade linear da base [m/s]
                self.u_w_list[idx],             # Comando de velocidade angular da base [rad/s]
                self.x_error_list[idx],         # Erro de posição no eixo x [m]
                self.y_error_list[idx],         # Erro de posição no eixo y [m]
                self.theta_error_list[idx]))    # Erro de orientação [rad]

        # Fechando arquivo com os dados

        data_file.close()

    #######################################################################################
    
    #######################################################################################

    def finalize(self):            
        
        """Finalização."""

        # Parar o robô

        self.u_v = 0.0
        self.u_w = 0.0
        self.publish_speed_refs()

###########################################################################################
### EXECUÇÃO
###########################################################################################

if __name__ == '__main__':

    # Instanciação do controlador de trajetória

    klancar_square = KlancarSquare()

    # Execução do laço principal

    klancar_square.spin()

    # Registro dos dados do ensaio em um arquivo

    klancar_square.write_file()

    # Finalização

    klancar_square.finalize()

    print("[INFO] Trajetória concluída.")

###########################################################################################
###########################################################################################