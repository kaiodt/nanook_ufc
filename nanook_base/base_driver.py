# coding: utf-8

###########################################################################################
###########################################################################################
## Projeto: Nanook UFC
## Autor: Kaio Douglas Teófilo Rocha
## Email: kaiodtr@gmail.com
###########################################################################################
## Arquivo: Driver de Controle e Comunicação com a Base (FRDM K64F)
## Revisão: 1 [26/02/2017]
###########################################################################################
###########################################################################################

import serial
import time

###########################################################################################
class NanookBase(object):
###########################################################################################

    #######################################################################################

    def __init__(self, serial_port='/dev/ttyACM0', baud_rate=115200):

        """Construtor

        Argumentos:
            serial_port (str): Porta serial da FRDM K64F
            baud_rate (int): Velocidade da conexão

        """

        self.serial_port = serial_port
        self.baud_rate = baud_rate

    #######################################################################################

    #######################################################################################
    
    def connect(self):

        """Conexão com a base (FRDM K64F) via porta serial."""

        try:

            print('[INFO] Conectando-se à base em %s.' % self.serial_port)

            # Criação do objeto Serial

            self.serial_base = \
                serial.Serial(self.serial_port, self.baud_rate, timeout = 1.0)

            time.sleep(1)
        
        except serial.SerialException:

            print('[ERROR] Falha ao tentar conectar com a base em %s.' % self.serial_port)
            raise SystemExit

        else:

            print("[INFO] Conexão com base estabelecida!")

    #######################################################################################

    #######################################################################################

    def disconnect(self):

        """Encerramento da conexão serial com a base (FRDM K64F)."""

        # Fechamento da porta serial

        self.serial_base.close()
        print('[INFO] Conexão com base terminada.')

    #######################################################################################

    #######################################################################################

    def set_vel_ref(self, v_ref, w_ref):

        """Definição de referência de velocidade (linear e angular).

        Argumentos:
            v_ref (float): Velocidade linear de referência [m/s]
            w_ref (float): Velocidade angular de referênca [rad/s]

        """

        # Preparação da string a ser enviada para a base
        # Formato: v[sig][v1].[v2][v3][v4][sig][w1].[w2][w3][w4]
        # Nota: Os pontos decimais não são enviados, por isso os valores são multiplicados
        # por 1000

        ref_str = 'v{:=+05.0f}{:=+05.0f}'.format(v_ref * 1000, w_ref * 1000)

        # Envio da string com a referência, um caractere por vez

        for char in ref_str:
            self.serial_base.write(char)

    #######################################################################################

    #######################################################################################

    def get_odometry(self):

        """Informações de odometria da base em relação ao referencial fixo.

        Retorno:
            tuple: (Posição no eixo x [m],
                    Posição no eixo y [m],
                    Orientação [rad],
                    Velocidade linear [m/s],
                    Velocidade angular [rad/s])

        """

        # Envio da solicitação de odometria para a base

        self.serial_base.write('o')

        # Recebimento dos 29 bytes contendo os dados de odometria
    
        odometry_str = self.serial_base.read(29)

        # Extração dos valores numéricos dos dados de odometria a partir da string recebida
        # Formato da string recebida:
        # [sig][x1][x2][x3].[x2][x5][x6]
        # [sig][y1][y2][y3].[y2][y5][y6]
        # [sig][t1].[t2][t3][t4]
        # [sig][v1].[v2][v3][v4]
        # [sig][w1].[w2][w3][w4]
        # Nota: Os pontos decimais não são recebidos

        # Extração da posição no eixo x [m]

        x = int(odometry_str[1]) * 100 + \
            int(odometry_str[2]) * 10 + \
            int(odometry_str[3]) + \
            int(odometry_str[4]) * 0.1 + \
            int(odometry_str[5]) * 0.01 + \
            int(odometry_str[6]) * 0.001

        # Teste de valor negativo

        if (odometry_str[0] == '-'):
            x *= -1

        # Extração da posição no eixo y [m]

        y = int(odometry_str[8]) * 100 + \
            int(odometry_str[9]) * 10 + \
            int(odometry_str[10]) + \
            int(odometry_str[11]) * 0.1 + \
            int(odometry_str[12]) * 0.01 + \
            int(odometry_str[13]) * 0.001

        # Teste de valor negativo

        if (odometry_str[7] == '-'):
            y *= -1

        # Extração da orientação [rad]

        theta = int(odometry_str[15]) + \
                int(odometry_str[16]) * 0.1 + \
                int(odometry_str[17]) * 0.01 + \
                int(odometry_str[18]) * 0.001

        # Teste de valor negativo

        if (odometry_str[14] == '-'):
            theta *= -1

        # Extração da velocidade linear [m/s]

        v = int(odometry_str[20]) + \
            int(odometry_str[21]) * 0.1 + \
            int(odometry_str[22]) * 0.01 + \
            int(odometry_str[23]) * 0.001

        # Teste de valor negativo

        if (odometry_str[19] == '-'):
            v *= -1

        # Extração da velocidade angular [rad/s]

        w = int(odometry_str[25]) + \
            int(odometry_str[26]) * 0.1 + \
            int(odometry_str[27]) * 0.01 + \
            int(odometry_str[28]) * 0.001

        # Teste de valor negativo

        if (odometry_str[24] == '-'):
            w *= -1

        # Retorno dos valores extraídos

        return (x, y, theta, v, w)

    #######################################################################################

    #######################################################################################

    def reset_base(self):

        """Reinicialização da base (parada dos motores e reinicialização de odometria)."""

        self.serial_base.write('r')     
        print("[INFO] Reiniciando a base...")
        time.sleep(0.5)

    #######################################################################################

###########################################################################################
###########################################################################################