#! /usr/bin/env python
# coding: utf-8

###########################################################################################
###########################################################################################
## Projeto: Nanook UFC
## Autor: Kaio Douglas Teófilo Rocha
## Email: kaiodtr@gmail.com
###########################################################################################
## Arquivo: Ensaio de Ajuste dos Encoders
## Revisão: 2 [16/03/2017]
###########################################################################################
###########################################################################################

###########################################################################################
# INSTRUÇÕES
###########################################################################################
# 1. Faça upload do código "Nanook_Encoder_Adjust" para a FRDM-K64F
# 2. Certifique-se de que os encoders estão conectados à placa de interface
# 3. Certifique-se de que o driver dos motores está conectado à placa de interface
# 4. Encontre a porta serial da conexão. Pode-se utilizar o comando:
#    $ ls /dev/ttyACM*
# 5. Alterar a variável "serial_port" para o valor encontrado
# 6. Executar o script
# 7. Fornecer os parâmetros do ensaio
###########################################################################################

import serial
import time
import matplotlib.pyplot as plt

##### Parâmetros definidos pelo usuário #####

freq = float(raw_input('Frequência de Amostragem [Hz]: '))
n_samples = int(raw_input('Número de Amostras: '))
right_vel_ref = float(raw_input('Velocidade de Referência Direita [-1.0, 1.0]: '))
left_vel_ref = float(raw_input('Velocidade de Referência Esquerda [-1.0, 1.0]: '))

# Período de amostragem

Ts = 1.0 / freq

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

##### Funções auxiliares #####

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

#######################################################################################

#######################################################################################

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

#######################################################################################

def get_encoder_deltas():

    """Leitura das variações dos encoders.

    Retorno:
        tuple: (Variação do encoder direito,
                Variação do encoder esquerdo)

    """

    # Recebimento dos 4 bytes da variação do encoder direito

    data = base.read(4)

    # Recuperação do valor da variação (união dos 4 bytes)

    right_delta = \
        ord(data[0]) | (ord(data[1])<<8) | (ord(data[2])<<16) | (ord(data[3])<<24)

    # Teste de valor negativo

    if right_delta >= 0x80000000:
        right_delta -= 0x100000000

    # Recebimento dos 4 bytes da variação do encoder esquerdo

    data = base.read(4)

    # Recuperação do valor da variação (união dos 4 bytes)

    left_delta = \
        ord(data[0]) | (ord(data[1])<<8) | (ord(data[2])<<16) | (ord(data[3])<<24)

    # Teste de valor negativo

    if left_delta >= 0x80000000:
        left_delta -= 0x100000000

    # Retorno das variações

    return (right_delta, left_delta)

#######################################################################################

##### Envio da referência #####

set_vel_ref(right_vel_ref, left_vel_ref)
time.sleep(0.05)

print('Referência enviada')

##### Listas com os dados do ensaio #####

right_vel_list = []
left_vel_list = []
# right_var_list = []
# left_var_list = []

##### Laço #####

# Contador de amostras

sample = 1

while sample <= n_samples:
    try:
        # Tempo no início da iteração

        t0_iter = time.time()

        # Leitura das velocidades

        (v_right, v_left) = get_vel()

        # Leitura das variações dos encoders

        # (right_delta, left_delta) = get_encoder_deltas()

        # Apresentação dos valores

        print('Velocidades')
        print('Velocidade Direita: %f' % v_right)
        print('Velocidade Esquerda: %f' % v_left)
        print

        # print('Variações')
        # print('Variação Direita: %d' % right_delta)
        # print('Variação Esquerda: %d' % right_delta)
        # print

        # Atualização das listas com os dados

        right_vel_list.append(v_right)
        left_vel_list.append(v_left)
        # right_var_list.append(right_delta)
        # left_var_list.append(left_delta)

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

##### Finalização #####

# Parada dos motores

set_vel_ref(0.0, 0.0)

base.close()
print('Conexão com a base terminada.')
print

##### Gráficos dos resultados #####

# Lista com números das amostras

samples = range(1, n_samples + 1)

# Velocidades

plt.close('all')
plt.figure(1)

# Velocidade direita [RPM] | Amostra

plt.subplot(1, 2, 1)
plt.plot(samples, right_vel_list, 'b-')
plt.title('Velocidade Direita')
plt.xlabel('Amostra')
plt.ylabel('Velocidade (RPM)')
plt.grid('on')

# Velocidade esquerda [RPM] | Amostra

plt.subplot(1, 2, 2)
plt.plot(samples, left_vel_list, 'b-')
plt.title('Velocidade Esquerda')
plt.xlabel('Amostra')
plt.ylabel('Velocidade (RPM)')
plt.grid('on')

# Variações

# plt.figure(2)

# # Variação direita | Amostra

# plt.subplot(1, 2, 1)
# plt.plot(time_list, right_var_list, 'b-')
# plt.title('Variacao Direita')
# plt.xlabel('Amostra')
# plt.ylabel('Variacao')
# plt.grid('on')

# # Velocidade esquerda [RPM] | Amostra

# plt.subplot(1, 2, 2)
# plt.plot(time_list, left_var_list, 'b-')
# plt.title('Variacao Esquerda')
# plt.xlabel('Amostra')
# plt.ylabel('Variacao')
# plt.grid('on')

plt.show()

##### Médias de velocidades #####

first_sample = int(raw_input('Primeira amostra: '))

right_avg = sum(right_vel_list[first_sample:]) / len(right_vel_list[first_sample:])
left_avg = sum(left_vel_list[first_sample:]) / len(left_vel_list[first_sample:])

print('Médias de Velocidade')
print('Velocidade Direita: %f RPM' % right_avg)
print('Velocidade Esquerda: %f RPM' % left_avg)

###########################################################################################
###########################################################################################