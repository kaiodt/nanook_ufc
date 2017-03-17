#! /usr/bin/env python
# coding: utf-8

###########################################################################################
###########################################################################################
## Projeto: Nanook UFC
## Autor: Kaio Douglas Teófilo Rocha
## Email: kaiodtr@gmail.com
###########################################################################################
## Arquivo: Script de Teste de Leitura dos Encoders
## Revisão: 1 [08/03/2017]
###########################################################################################
###########################################################################################

###########################################################################################
# INSTRUÇÕES
###########################################################################################
# 1. Faça upload do código "Nanook_Encoder_Test" para a FRDM-K64F
# 2. Certifique-se de que os encdoers estão conectados à placa de interface
# 3. Encontre a porta serial da conexão. Pode-se utilizar o comando:
#    $ ls /dev/ttyACM*
# 4. Alterar a variável "serial_port" para o valor encontrado
# 5. Executar o script
# 6. Executar também o script 'test_motor_driver.py' (seguir instruções específicas)
# 7. Enviar comandos de velocidade para os motores e conferir as leituras dos encoders
###########################################################################################

import serial
import time
import matplotlib.pyplot as plt

##### Frequência de leitura #####

freq = 300
Ts = 1.0 / freq

##### Listas com as leituras #####

right_enc_list = []
left_enc_list = []

##### Porta serial #####

serial_port = '/dev/ttyACM0'

##### Conexão #####

try:

    print('Conectando-se aos encoders em %s.' % serial_port)

    encoders = serial.Serial(serial_port, 115200)

    time.sleep(1)

except serial.SerialException:

    print('Falha ao tentar conectar com os encoders em %s.' % serial_port)
    raise SystemExit

else:

    print('Conexão com encoders estabelecida!')

##### Reinicialização das leituras #####

encoders.write('r')
time.sleep(0.5)

##### Função de leitura #####

def read_encoder(encoders_serial, encoder_id):

    """Leitura de um encoder via serial.

    Argumentos:
        encoders_serial (Serial): Instância da conexão serial com os encoders
        encoder_id (str): Identificação do encoder ('d' = diereito, 'e' = esquerdo)

    Retorno:
        int: Leitura do encoder

    """

    # Envio da solicitação para a K64F

    encoders_serial.write(encoder_id)

    # Recebimento dos 4 bytes da leitura

    data = encoders_serial.read(4)

    # Recuperação do valor da leitura (união dos 4 bytes)

    reading = \
        ord(data[0]) | (ord(data[1])<<8) | (ord(data[2])<<16) | (ord(data[3])<<24)

    # Teste de valor negativo

    if reading >= 0x80000000:
        reading -= 0x100000000

    # Retorno da leitura

    return reading

##### Laço #####

while True:
    try:
        # Tempo no início da iteração
        
        t0_iter = time.time()

        # Leitura dos encoders (4 bytes cada)

        right_enc = read_encoder(encoders, 'd')
        left_enc = read_encoder(encoders, 'e')

        # Atualização das listas de leituras

        right_enc_list.append(right_enc)
        left_enc_list.append(left_enc)

        # Apresentação dos valores

        print('Direito: %d' % right_enc)
        print('Esquerdo: %d' % left_enc)
        print

        # Duração da iteração

        dt_iter = time.time() - t0_iter

        print('Tempo da Iteração: %.5f' % dt_iter)
        print

        # Aguardar até que o período seja concluído

        time.sleep(Ts - dt_iter)

    except KeyboardInterrupt:
        break

##### Finalização #####

encoders.close()
print('Conexão com encoders terminada.')

##### Gráficos dos resultados #####

plt.close('all')
plt.figure(1)

# Encoder direito

plt.subplot(1, 2, 1)
plt.plot(right_enc_list, 'b.')
plt.title('Encoder Direito')
plt.xlabel('Amostra')
plt.ylabel('Leitura')
plt.grid('on')

# Encoder esquerdo

plt.subplot(1, 2, 2)
plt.plot(left_enc_list, 'b.')
plt.title('Encoder Esquerdo')
plt.xlabel('Amostra')
plt.ylabel('Leitura')
plt.grid('on')

# Diferenças entre leituras consecutivas

right_diffs = \
    [(right_enc_list[i] - right_enc_list[i-1]) for i in range(1, len(right_enc_list))]
left_diffs = \
    [(left_enc_list[i] - left_enc_list[i-1]) for i in range(1, len(left_enc_list))]

# Gráficos

plt.figure(2)

# Diferenças no encoder direito

plt.subplot(1, 2, 1)
plt.plot(right_diffs)
plt.title('Diferencas no Encoder Direito')
plt.xlabel('Amostra')
plt.ylabel('Diferenca')
plt.grid('on')

# Encoder esquerdo

plt.subplot(1, 2, 2)
plt.plot(left_diffs)
plt.title('Diferencas no Encoder Esquerdo')
plt.xlabel('Amostra')
plt.ylabel('Diferenca')
plt.grid('on')

plt.show()

###########################################################################################
###########################################################################################