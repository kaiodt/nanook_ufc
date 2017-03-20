#! /usr/bin/env python
# coding: utf-8

###########################################################################################
###########################################################################################
## Projeto: Nanook UFC
## Autor: Kaio Douglas Teófilo Rocha
## Email: kaiodtr@gmail.com
###########################################################################################
## Arquivo: Ensaio do Controlador Embarcado da Base
## Revisão: 2 [19/03/2017]
###########################################################################################
###########################################################################################

###########################################################################################
# INSTRUÇÕES
###########################################################################################
# 1. Faça upload do código "Nanook_Base_Controller" para a FRDM-K64F
# 2. Certifique-se de que os encoders estão conectados à placa de interface
# 3. Certifique-se de que o driver dos motores está conectado à placa de interface
# 4. Encontre a porta serial da conexão. Pode-se utilizar o comando:
#    $ ls /dev/ttyACM*
# 5. Alterar o primeiro argumento na instanciação da base "NanookBase" para o valor
#    enconcontrado
# 6. Executar o script
# 7. Fornecer os parâmetros do ensaio
# 8. Executar o script "plot_ensaio_controlador_base.py" para plotar os resultados
###########################################################################################

import rospy
import time
from math import sin, cos, pi
from os.path import expanduser

from base_driver import NanookBase

###########################################################################################
### INICIALIZAÇÃO
###########################################################################################

##### Inicialização do nó #####

rospy.init_node('ensaio_controlador_base')

##### Conexão com o driver controlador da base (FRDM K64F) #####

# Instanciação 

base = NanookBase('/dev/ttyACM0', 115200)

# Conexão

base.connect()

# Inicialização

base.reset_base()
base.set_pose(0.0, 0.0, pi/2)
rospy.sleep(0.5)

##### Parâmetros definidos pelo usuário #####

ensaio = int(raw_input('Número do Ensaio: '))
freq = float(raw_input('Frequência de Amostragem [Hz]: '))
n_samples = int(raw_input('Número de Amostras: '))
right_vel_ref = float(raw_input('Velocidade de Referência Direita [RPM]: '))
left_vel_ref = float(raw_input('Velocidade de Referência Esquerda [RPM]: '))

##### Parâmetros fixos #####

# Período de amostragem

Ts = 1.0 / freq

# Dimensões do robô

wheel_diameter = 0.17       # Diâmetro das rodas [m]
wheel_separation = 0.405    # Separação entre duas rodas [m]

##### Listas com os dados do ensaio #####

time_list = []          # Tempo [s]
right_vel_list = []     # Velocidade direita [RPM]
left_vel_list = []      # Velocidade esquerda [RPM]
x_list = []             # Posição no eixo x [m]
y_list = []             # Posição no eixo y [m]
theta_list = []         # Orientação [rad]
v_list = []             # Velocidade linear da base [m/s]
w_list = []             # Velocidade angular da base [rad/s]

##### Arquivo para salvar os resultados #####

# Diretório

home = expanduser('~')
path = home + '/ros_catkin_ws/src/nanook_ufc/ensaios'
path += '/controlador_base/resultados/ensaio_%d.txt' % ensaio

# Abertura do arquivo

data_file = open(path, 'w')

# Registro dos parâmetros do ensaio

params_str  = '### Ensaio do Controlador Embarcado da Base ###\n\n'
params_str += '# Parâmetros #\n\n'
params_str += '# Número do Ensaio: %d\n' % ensaio
params_str += '# Frequência de Amostragem: %.1f Hz\n' % freq
params_str += '# Número de Amostras: %d\n' % n_samples
params_str += '# Velocidade de Referência Esquerda: %.2f RPM\n' % left_vel_ref
params_str += '# Velocidade de Referência Direita: %.2f RPM\n\n' % right_vel_ref

data_file.write(params_str)

##### Variáveis #####

# Controle de tempo

last_time = rospy.get_time()   # Instante de tempo da última leitura

###########################################################################################
### FUNÇÕES AUXILIARES
###########################################################################################

def convert_to_base_vel(right_vel, left_vel):

    """Conversão de velocidade individuais [RPM] para velocidades linear [m/s]
       e angular [rad/s] da base

    Parâmetros:
        right_vel (float): Velocidade direita [RPM]
        left_vel (float): Velocidade esquerda [RPM]

    Retorno:
        tuple: (Velocidade linear [m/s],
                Velocidade angular [rad/s])

    """

    # Conversão das velocidades individuais de [RPM] para [m/s]

    right_vel *= pi * wheel_diameter / 60.0
    left_vel *= pi * wheel_diameter / 60.0

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
        tuple: (Velocidade direita [RPM],
                Velocidade esquerda [RPM])

    """

    # Conversão para velocidades individuais  [m/s]

    right_vel = 1.0 * v + wheel_separation * w / 2.0
    left_vel  = 1.0 * v - wheel_separation * w / 2.0

    # Transformação das velocidades para [RPM]

    right_vel *= 60.0 / (pi * wheel_diameter)
    left_vel  *= 60.0 / (pi * wheel_diameter)

    # Retorno das velocidades convertidas

    return (right_vel, left_vel)

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

        (right_vel, left_vel) = convert_to_wheel_vel(v, w)

        print('* Velocidades:')
        print('- V_r = %.5f RPM' % right_vel)
        print('- V_l = %.5f RPM' % left_vel)

        # Atualização das listas com os dados

        time_list.append(t_iter)
        right_vel_list.append(right_vel)
        left_vel_list.append(left_vel)
        x_list.append(x)
        y_list.append(y)
        theta_list.append(theta) 
        v_list.append(v)
        w_list.append(w)

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

    # Registro dos resultados no arquivo

    # Listas com as velocidades de referência

    global time_list
    
    right_vel_ref_list = [right_vel_ref] * len(time_list)
    left_vel_ref_list = [left_vel_ref] * len(time_list)

    # Desconto do instante inicial (deve ser 0 s)

    t0 = time_list[0]
    time_list = [(t - t0) for t in time_list]

    # Salvando resultados no arquivo

    for idx in range(len(time_list)):
        data_file.write('* {0} {1} {2} {3} {4} {5} {6} {7} {8} {9} {10}\n'.format(
            idx + 1,                    # Amostra
            time_list[idx],             # Tempo [s]
            right_vel_ref_list[idx],    # Velocidade de referência direita [RPM]
            right_vel_list[idx],        # Velocidade direita [RPM]
            left_vel_ref_list[idx],     # Velocidade de referência esquerda [RPM]
            left_vel_list[idx],         # Velocidade esquerda [RPM]
            x_list[idx],                # Posição no eixo x [m]
            y_list[idx],                # Posição no eixo y [m]
            theta_list[idx],            # Orientação [rad]
            v_list[idx],                # Velocidade linear da base [m/s]
            w_list[idx]))               # Velocidade angulas da base [rad/s]

    # Fechando arquivo com os dados

    data_file.close()

###########################################################################################
### EXECUÇÃO
###########################################################################################

if __name__ == '__main__':

    # Conversão das velocidades individuais de referência [RPM] para velocidades 
    # linear [m/s] e angular [rad/s] de referência da base

    (v_ref, w_ref) = convert_to_base_vel(right_vel_ref, left_vel_ref)

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