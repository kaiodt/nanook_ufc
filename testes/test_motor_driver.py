#! /usr/bin/env python
# coding: utf-8

###########################################################################################
###########################################################################################
## Projeto: Nanook UFC
## Autor: Kaio Douglas Teófilo Rocha
## Email: kaiodtr@gmail.com
###########################################################################################
## Arquivo: Script de Teste do Driver dos Motores
## Revisão: 1 [08/03/2017]
###########################################################################################
###########################################################################################

###########################################################################################
# INSTRUÇÕES
###########################################################################################
# 1. Conecte o pino S1 do driver ao pino de TX do conversor RS232-USB
# 2. Conecte os pinos de GND do driver e do conversor RS232-USB
# 3. Encontre a porta serial da conexão. Pode-se utilizar os comandos:
#    $ ls /dev/ttyACM*
#    $ ls /dev/ttyUSB*
# 4. Alterar a variável "serial_port" para o valor encontrado
# 5. Executar o script
# 6. Utilize o comando [e/d/a][velocidade] para enviar uma velocidade [-1,1] para o motor
#    esquerdo (e), direito (d), ou ambos (a)
# 7. Utilize o comando [q] para encerrar
###########################################################################################

import serial
import time

##### Porta serial #####

serial_port = '/dev/ttyUSB0'

##### Conexão #####

try:

    print('Conectando-se ao driver em %s.' % serial_port)

    driver = serial.Serial(serial_port, 9600)

    time.sleep(1)

except serial.SerialException:

    print('Falha ao tentar conectar com o driver em %s.' % serial_port)
    raise SystemExit

else:

    print('Conexão com driver estabelecida!')

##### Laço #####

while True:

    command = raw_input("Comando [e/d/a][velocidade] ('q' para terminar): ")

    try:

        if command == 'q':
            break

        command_vel = float(command[2:])

        if command[0] == 'd':

            # Conversão - Motor Direito
            command_byte = int((command_vel + 1) * 63 + 1)      # Valor entre 1 e 127

            driver.write([command_byte])

        elif command[0] == 'e':

            # Conversão - Motor Esquerdo

            command_byte = int((command_vel + 1) * 63 + 129)    # Valor entre 129 e 255

            driver.write([command_byte])

        elif command[0] == 'a':

            # Conversão - Motor Direito
            right_command_byte = int((command_vel + 1) * 63 + 1)    # Valor entre 1 e 127

            # Conversão - Motor Esquerdo
            left_command_byte = int((command_vel + 1) * 63 + 129)   # Valor entre 129 e 255

            driver.write([right_command_byte])
            driver.write([left_command_byte])

        else:

            raise

    except:
        
        print('Comando inválido!')

##### Finalização #####

driver.write([0])
driver.close()
print('Conexão com driver terminada.')

###########################################################################################
###########################################################################################