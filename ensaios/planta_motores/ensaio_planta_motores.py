#! /usr/bin/env python
# coding: utf-8

###########################################################################################
###########################################################################################
## Projeto: Nanook UFC
## Autor: Kaio Douglas Teófilo Rocha
## Email: kaiodtr@gmail.com
###########################################################################################
## Arquivo: Ensaio para Levantamento das Plantas dos Motores
## Revisão: 1 [16/03/2017]
###########################################################################################
###########################################################################################

###########################################################################################
# INSTRUÇÕES
###########################################################################################
# 1. Faça upload do código "Nanook_Motor_Plant" para a FRDM-K64F
# 2. Certifique-se de que os encoders estão conectados à placa de interface
# 3. Certifique-se de que o driver dos motores está conectado à placa de interface
# 4. Encontre a porta serial da conexão. Pode-se utilizar o comando:
#    $ ls /dev/ttyACM*
# 5. Alterar a variável "serial_port" para o valor encontrado
# 6. Executar o script
# 7. Fornecer os parâmetros do ensaio
# 8. Executar o script "plot_ensaio_planta_motores.py" para plotar os resultados
###########################################################################################

import serial
import time
from os.path import expanduser

###########################################################################################
# INICIALIZAÇÃO
###########################################################################################

##### Parâmetros definidos pelo usuário #####

ensaio = int(raw_input('Número do Ensaio: '))
freq = float(raw_input('Frequência de Amostragem [Hz]: '))
n_samples = int(raw_input('Número de Amostras: '))
right_vel_ref = float(raw_input('Velocidade de Referência Direita [-1.0, 1.0]: '))
left_vel_ref = float(raw_input('Velocidade de Referência Esquerda [-1.0, 1.0]: '))

# Período de amostragem

Ts = 1.0 / freq

### Arquivo para salvar os resultados ###

# Diretório

home = expanduser('~')
path = home + '/ros_catkin_ws/src/nanook_ufc/ensaios'
path += '/planta_motores/resultados/ensaio_%d.txt' % ensaio

# Abertura do arquivo

data_file = open(path, 'w')

# Registro dos parâmetros do ensaio

params_str  = '*** Ensaio para Levantamento das Plantas dos Motores ***\n\n'
params_str += '* Parâmetros *\n\n'
params_str += '- Número do Ensaio: %d\n' % ensaio
params_str += '- Frequência de Amostragem: %.1f Hz\n' % freq
params_str += '- Número de Amostras: %d\n' % n_samples
params_str += '- Velocidade de Referência Direita: %.1f \n' % right_vel_ref
params_str += '- Velocidade de Referência Esquerda: %.1f \n\n' % left_vel_ref

data_file.write(params_str)

##### Porta serial #####

serial_port = '/dev/ttyACM0'

##### Conexão #####

try:

    print('Conectando-se à base em %s.' % serial_port)

    base = serial.Serial(serial_port, 115200)

    time.sleep(1)

except serial.SerialException:

    print('Falha ao tentar conectar com a base em %s.' % serial_port)
    raise SystemExit

else:

    print('Conexão com a base estabelecida!')

##### Reinicialização da base #####

base.write('r')
time.sleep(0.5)

###########################################################################################
# FUNÇÕES AUXILIARES
###########################################################################################

def set_vel_ref(v_right, v_left):

    """Definição de referência de velocidade (direita e esquerda).

    Argumentos:
        v_right (float): Velocidade de referência direita [-1.0, 1.0]
        v_left (float): Velocidade de referência esquerda [-1.0, 1.0]

    """

    # Preparação da string a ser enviada para a base
    # Formato: v[sig][d1].[d2][sig][e1].[e2]
    # Nota: Os pontos decimais não são enviados, por isso os valores são multiplicados
    # por 10

    ref_str = 'v{:=+03.0f}{:=+03.0f}'.format(v_right * 10, v_left * 10)

    # Envio da string com a referência, um caractere por vez

    for char in ref_str:
        base.write(char)

###########################################################################################

###########################################################################################

def get_vel():

    """Informações de velocidade da base.

    Retorno:
        tuple: (Velocidade direita [RPM],
                Velocidade esquerda [RPM])

    """

    # Envio da solicitação de leituras de velocidade

    base.write('l')

    # Recebimento dos 12 bytes contendo os dados das velocidades

    vel_str = base.read(12)

    # Extração dos valores numéricos dos dados das velocidades a partir da string recebida
    # Formato da string recebida:
    # [sig][d1][d2].[d3][d4][d5]
    # [sig][e1][e2].[e3][e4][e5]
    # Nota: Os pontos decimais não são recebidos

    # Extração da velocidade direita [RPM]

    v_right = int(vel_str[1]) * 10 + \
              int(vel_str[2]) + \
              int(vel_str[3]) * 0.1 + \
              int(vel_str[4]) * 0.01 + \
              int(vel_str[5]) * 0.001

    # Teste de valor negativo

    if (vel_str[0] == '-'):
        v_right *= -1

    # Extração da velocidade esquerda [RPM]

    v_left = int(vel_str[7])  * 10 + \
             int(vel_str[8])  + \
             int(vel_str[9])  * 0.1 + \
             int(vel_str[10]) * 0.01 + \
             int(vel_str[11]) * 0.001

    # Teste de valor negativo

    if (vel_str[6] == '-'):
        v_left *= -1

    # Retorno dos valores extraídos

    return (v_right, v_left)

###########################################################################################
# EXECUÇÃO
###########################################################################################

##### Envio da referência #####

set_vel_ref(right_vel_ref, left_vel_ref)
time.sleep(0.05)

print('Referência enviada')

##### Listas com os dados do ensaio #####

right_vel_list = []
left_vel_list = []
time_list = []

##### Laço #####

# Contador de amostras

sample = 1

while sample <= n_samples:
    try:
        # Tempo no início da iteração

        t0_iter = time.time()

        # Leitura das velocidades

        (v_right, v_left) = get_vel()

        # Apresentação dos valores

        print('Velocidades')
        print('Velocidade Direita: %f RPM' % v_right)
        print('Velocidade Esquerda: %f RPM' % v_left)
        print

        # Atualização das listas com os dados

        right_vel_list.append(v_right)
        left_vel_list.append(v_left)
        time_list.append(t0_iter)

        # Incremento do contador de amostras

        sample += 1

        # Duração da iteração

        dt_iter = time.time() - t0_iter

        print('Tempo da Iteração: %.5f' % dt_iter)
        print

        # Aguardar até que o período de amostragem seja concluído

        time.sleep(Ts - dt_iter)

    except KeyboardInterrupt:
        break

###########################################################################################
# FINALIZAÇÃO
###########################################################################################

# Parada dos motores

set_vel_ref(0.0, 0.0)

# Finalização da conexão com a base

base.close()
print('Conexão com a base terminada.')
print

##### Registro dos resultados no arquivo #####

# Listas com as velocidades de referência

right_max_vel = 30.30606    # Máxima velocidade direita
left_max_vel = 29.87750     # Máxima velocidade esquerda

right_vel_ref_list = [right_vel_ref * right_max_vel] * len(time_list)
left_vel_ref_list = [left_vel_ref * left_max_vel] * len(time_list)

# Desconto do instante inicial (deve ser 0 s)

t0 = time_list[0]
time_list = [(t - t0) for t in time_list]

# Salvando resultados no arquivo

for idx in range(len(time_list)):
    data_file.write('# {0} {1} {2} {3} {4} {5}\n'.format(
        idx + 1,                    # Amostra
        time_list[idx],             # Tempo [s]
        right_vel_ref_list[idx],    # Velocidade de referência direita [RPM]
        right_vel_list[idx],        # Velocidade direita [RPM]
        left_vel_ref_list[idx],     # Velocidade de referência esquerda [RPM]
        left_vel_list[idx]))        # Velocidade esquerda [RPM]

# Fechando arquivo

data_file.close()

###########################################################################################
###########################################################################################