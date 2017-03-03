#! /usr/bin/env python
# coding: utf-8

###########################################################################################
###########################################################################################
## Projeto: Nanook UFC
## Autor: Kaio Douglas Teófilo Rocha
## Email: kaiodtr@gmail.com
###########################################################################################
## Arquivo: Ensaio do Controlador Embarcado da Base
## Revisão: 1 [03/03/2017]
###########################################################################################
###########################################################################################

import rospy
import time
from math import sin, cos, pi
from os.path import expanduser

from base_driver import NanookBase

###########################################################################################
### INICIALIZAÇÃO
###########################################################################################

### Inicialização do nó ###

rospy.init_node('ensaio_controlador_base')

### Conexão com o driver controlador da base (FRDM K64F) ###

# Instanciação 

base = NanookBase('/dev/ttyACM0', 115200)

# Conexão

base.connect()

# Inicialização

base.reset_base()
base.set_pose(0.0, 0.0, pi/2)
rospy.sleep(0.5)

### Parâmetros definidos pelo usuário ###

ensaio = int(raw_input('Número do Ensaio: '))
freq = float(raw_input('Frequência de Amostragem [Hz]: '))
n_samples = int(raw_input('Número de Amostras: '))
left_vel_ref = float(raw_input('Velocidade de Referência Esquerda [RPM]: '))
right_vel_ref = float(raw_input('Velocidade de Referência Direita [RPM]: '))

### Parâmetros fixos ###

# Período de amostragem

Ts = 1.0 / freq

# Dimensões do robô

wheel_diameter = 0.17       # Diâmetro das rodas [m]
wheel_separation = 0.405    # Separação entre duas rodas [m]

### Arquivo para salvar os resultados ###

# Diretório

home = expanduser('~')
path = home + '/ros_catkin_ws/src/nanook_ufc/ensaios'
path += '/controlador_base/resultados/ensaio_%d.txt' % ensaio

# Abertura do arquivo

data_file = open(path, 'w')

# Registro dos parâmetros do ensaio

params_str  = '*** Ensaio do Controlador Embarcado da Base ***\n\n'
params_str += '* Parâmetros *\n\n'
params_str += '- Número do Ensaio: %d\n' % ensaio
params_str += '- Frequência de Amostragem: %.1f Hz\n' % freq
params_str += '- Número de Amostras: %d\n' % n_samples
params_str += '- Velocidade de Referência Esquerda: %.2f RPM\n' % left_vel_ref
params_str += '- Velocidade de Referência Direita: %.2f RPM\n\n' % right_vel_ref

data_file.write(params_str)

### Variáveis ###

# Controle de tempo

last_time = rospy.get_time()   # Instante de tempo da última leitura

###########################################################################################
### FUNÇÕES AUXILIARES
###########################################################################################

###########################################################################################

def convert_to_base_vel(left_vel, right_vel):

    """Conversão de velocidade individuais [RPM] para velocidades linear [m/s]
       e angular [rad/s] da base

    Parâmetros:
        left_vel (float): Velocidade esquerda [RPM]
        right_vel (float): Velocidade direita [RPM]

    Retorno:
        tuple: (Velocidade linear [m/s],
                Velocidade angular [rad/s])

    """

    # Conversão das velocidades individuais de [RPM] para [m/s]

    left_vel *= pi * wheel_diameter / 60.0
    right_vel *= pi * wheel_diameter / 60.0

    # Conversão para velocidades linear e angular da base

    v = (right_vel + left_vel) / 2.0                # Velocidade linear [m/s]
    w = (right_vel - left_vel) / wheel_separation   # Velocidade angular [rad/s]

    # Retorno das velocidades convertidas

    return (v, w)

###########################################################################################

###########################################################################################

def convert_to_wheel_vel(v, w):

    """Conversão de velocidades linear [m/s] e angular [rad/s] da base para velocidades
       individuais [RPM]

    Parâmetros:
        v (float): Velocidade linear [m/s]
        w (float): Velocidade linear [rad/s]

    Retorno:
        tuple: (Velocidade esquerda [RPM],
                Velocidade direita [RPM])

    """

    # Conversão para velocidades individuais  [m/s]

    left_vel  = 1.0 * v - wheel_separation * w / 2.0
    right_vel = 1.0 * v + wheel_separation * w / 2.0

    # Transformação das velocidades para [RPM]

    left_vel  *= 60.0 / (pi * wheel_diameter)
    right_vel *= 60.0 / (pi * wheel_diameter)

    # Retorno das velocidades convertidas

    return (left_vel, right_vel)

###########################################################################################

###########################################################################################

def spin():

    """Laço Principal"""

    ### Inicialização ###

    sample = 1                      # Amostra inicial
    t_loop = rospy.get_time()       # Tempo inicial do loop
    last_time = rospy.get_time()    # Último valor de tempo

    ### Executar até interrupção ou conclusão da amostragem ###

    while (not rospy.is_shutdown()) and (sample <= n_samples):

        # Tempo no início da iteração

        t_iter = rospy.get_time()

        print('*** Amostra %d ***' % sample)

        # Recebimento de dados de odometria da base

        (x, y, theta, v, w) = base.get_odometry()

        # Intervalo de tempo entre leituras (Deve ser próximo de Ts!)

        dt_readings = rospy.get_time() - last_time
        last_time = rospy.get_time()

        print('* Odometria:')
        print('- x = %.5f m' % x)
        print('- y = %.5f m' % y)
        print('- 0 = %.5f graus' % (theta * 180 / pi))
        print('- v = %.5f m/s' % v)
        print('- w = %.5f rad/s' % w)

        # Conversão das velocidades linear e angular para velocidades individuais [RPM]

        (left_vel, right_vel) = convert_to_wheel_vel(v, w)

        print('* Velocidades:')
        print('- V_l = %.5f RPM' % left_vel)
        print('- V_r = %.5f RPM' % right_vel)

        # Registro das leituras no arquivo

        sample_str = '%d' % sample                  # Amostra
        time_str = '%.5f' % last_time               # Tempo [s]
        left_ref_str = '%.3f' % left_vel_ref        # Velocidade de referência esquerda [RPM]
        left_vel_str = '%.5f' % left_vel            # Velocidade esquerda [RPM]
        right_ref_str = '%.3f' % right_vel_ref      # Velocidade de referência direita [RPM]
        right_vel_str = '%.5f' % right_vel          # Velocidade direita [RPM]
        x_str = '%.5f' % x                          # Posição no eixo x [m]
        y_str = '%.5f' % y                          # Posição no eixo y [m]
        theta_str = '%.5f' % (theta * 180 / pi)     # Orientação [graus]
        v_str = '%.5f' % v                          # Velocidade linear [m/s]
        w_str = '%.5f' % w                          # Velocidade angular [rad/s]

        data_file.write('# {0} {1} {2} {3} {4} {5} {6} {7} {8} {9} {10}\n'.format(
            sample_str, time_str, left_ref_str, left_vel_str, right_ref_str, right_vel_str,
            x_str, y_str, theta_str, v_str, w_str))

        # Incremento do contador de amostras

        sample += 1

        # Tempos de execução

        # Duração do loop (Deve ser próximo de Ts!)

        dt_loop = rospy.get_time() - t_loop
        t_loop = rospy.get_time()

        print('Duração do Loop (Ts): %.5f s' % dt_loop)

        # Duração da iteração

        dt_iter = rospy.get_time() - t_iter

        print('Duração da Iteração: %.5f s' % dt_iter)
        print

        # Aguardar o tempo restante para completar o período de amostragem

        rospy.sleep(Ts - dt_iter)

###########################################################################################

###########################################################################################

def finalize():

    """Finalização"""

    # Reinicialização da base

    base.reset_base()

    # Encerramento da conexão com a base

    base.disconnect()

    # Fechamento do arquivo com os dados

    data_file.close()

###########################################################################################

###########################################################################################
### EXECUÇÃO
###########################################################################################

if __name__ == '__main__':

    # Conversão das velocidades individuais de referência [RPM] para velocidades 
    # linear [m/s] e angular [rad/s] de referência da base

    (v_ref, w_ref) = convert_to_base_vel(left_vel_ref, right_vel_ref)

    # Envio da referência de velocidade para a base

    base.set_vel_ref(v_ref, w_ref)
    time.sleep(0.005)

    # Execução do laço principal

    spin()

    # Finalização

    print('[INFO] Ensaio encerrado normalmente.')
    
    finalize()

###########################################################################################
###########################################################################################