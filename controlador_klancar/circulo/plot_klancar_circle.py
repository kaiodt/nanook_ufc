#! /usr/bin/env python
# coding: utf-8

###########################################################################################
###########################################################################################
## Projeto: Nanook UFC
## Autor: Kaio Douglas Teófilo Rocha
## Email: kaiodtr@gmail.com
###########################################################################################
## Arquivo: Plotter de Ensaios do Controlador de Klancar (Círculo)
## Revisão: 2 [08/04/2017]
###########################################################################################
###########################################################################################

from os.path import expanduser
from math import pi, sin, cos
import matplotlib.pyplot as plt

###########################################################################################
### INICIALIZAÇÃO
###########################################################################################

# Número do ensaio

ensaio = int(raw_input('Número do Ensaio: '))

# Arquivo com os dados

home = expanduser('~')
path = home + '/ros_catkin_ws/src/nanook_ufc/controlador_klancar'
path += '/circulo/resultados/ensaio_%d.txt' % ensaio

# Abertura do arquivo

data_file = open(path, 'r')

# Listas com os dados de cada variável coletada

samples = []            # Amostra
time_list = []          # Tempo [s]
x_ref_list = []         # Referência de posição no eixo x [m]  
y_ref_list = []         # Referência de posição no eixo y [m]
theta_ref_list = []     # Referência de orientação [rad]
v_ref_list = []         # Referência de velocidade linear da base [m/s]
w_ref_list = []         # Referência de velocidade angular da base [rad/s]
x_list = []             # Posição no eixo x [m]
y_list = []             # Posição no eixo y [m]
theta_list = []         # Orientação [rad]
v_list = []             # Velocidade linear da base [m/s]
w_list = []             # Velocidade angular da base [rad/s]
u_v_list = []           # Comando de velocidade linear da base [m/s]
u_w_list = []           # Comando de velocidade angulat da base [rad/s]
x_error_list = []       # Erro de posição no eixo x [m]
y_error_list = []       # Erro de posição no eixo y [m]
theta_error_list = []   # Erro de orientação [rad]


###########################################################################################
### LEITURA DO ARQUIVO
###########################################################################################

# Exibição das linhas com informações e armazenamento de linhas com dados (iniciadas com *)
# nas respectivas listas

for line in data_file:
    if '*' not in line:                     # Linha com informações do ensaio
        print line
    else:                                   # Linha com dados
        line = line.split()
        samples.append(int(line[1]))
        time_list.append(float(line[2]))
        x_ref_list.append(float(line[3]))
        y_ref_list.append(float(line[4]))
        theta_ref_list.append(float(line[5]))
        v_ref_list.append(float(line[6]))
        w_ref_list.append(float(line[7]))
        x_list.append(float(line[8]))
        y_list.append(float(line[9]))
        theta_list.append(float(line[10]))
        v_list.append(float(line[11]))
        w_list.append(float(line[12]))
        u_v_list.append(float(line[13]))
        u_w_list.append(float(line[14]))
        x_error_list.append(float(line[15]))
        y_error_list.append(float(line[16]))
        theta_error_list.append(float(line[17]))

# Fechamento do arquivo

data_file.close()

###########################################################################################
### GERAÇÃO DE GRÁFICOS
###########################################################################################

plt.close('all')

# Pose

plt.figure(1)

# Posição no eixo y [m] | Posição no eixo x [m]

plt.subplot(2, 2, 1)
plt.plot(x_ref_list, y_ref_list, 'r--', x_list, y_list, 'b-')
plt.title('Trajetoria XY')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.grid('on')

# Posição no eixo x [m] | Tempo [s]

plt.subplot(2, 2, 2)
plt.plot(time_list, x_ref_list, 'r--', time_list, x_list, 'b-')
plt.title('Posicao no Eixo X')
plt.xlabel('Tempo (s)')
plt.ylabel('x (m)')
plt.grid('on')

# Orientação [rad] | Tempo [s]

plt.subplot(2, 2, 3)
plt.plot(time_list, theta_ref_list, 'r--', time_list, theta_list, 'b-')
plt.title('Orientacao')
plt.xlabel('Tempo (s)')
plt.ylabel('theta (rad)')
plt.grid('on')

# Posição no eixo y [m] | Tempo [s]

plt.subplot(2, 2, 4)
plt.plot(time_list, y_ref_list, 'r--', time_list, y_list, 'b-')
plt.title('Posicao no Eixo Y')
plt.xlabel('Tempo (s)')
plt.ylabel('y (m)')
plt.grid('on')

# Velocidade

plt.figure(2)

# Velocidade linear [m/s] | Tempo [s]

plt.subplot(2, 2, 1)
plt.plot(time_list, v_ref_list, 'r--', time_list, v_list, 'b-')
plt.title('Velocidade Linear')
plt.xlabel('Tempo (s)')
plt.ylabel('v (m/s)')
plt.grid('on')

# Controle de velocidade linear [m/s] | Tempo [s]

plt.subplot(2, 2, 2)
plt.plot(time_list, u_v_list, 'b-')
plt.title('Controle Velocidade Linear')
plt.xlabel('Tempo (s)')
plt.ylabel('u_v (m/s)')
plt.grid('on')

# Velocidade angular [rad/s] | Tempo [s]

plt.subplot(2, 2, 3)
plt.plot(time_list, w_ref_list, 'r--', time_list, w_list, 'b-')
plt.title('Velocidade Angular')
plt.xlabel('Tempo (s)')
plt.ylabel('w (rad/s)')
plt.grid('on')

# Controle de velocidade angular [rad/s] | Tempo [s]

plt.subplot(2, 2, 4)
plt.plot(time_list, u_w_list, 'b-')
plt.title('Controle Velocidade Angular')
plt.xlabel('Tempo (s)')
plt.ylabel('u_w (rad/s)')
plt.grid('on')

plt.figure(3)

# Erro de posição no eixo x [m] | Tempo [s]

plt.subplot(2, 2, 1)
plt.plot(time_list, x_error_list, 'b-')
plt.title('Erro de Posicao no Eixo x')
plt.xlabel('Tempo (s)')
plt.ylabel('erro_x (m)')
plt.grid('on')

# Erro de posição no eixo y [m] | Tempo [s]

plt.subplot(2, 2, 2)
plt.plot(time_list, y_error_list, 'b-')
plt.title('Erro de Posicao no Eixo y')
plt.xlabel('Tempo (s)')
plt.ylabel('erro_y (m)')
plt.grid('on')

# Erro de orientação [rad] | Tempo [s]

plt.subplot(2, 2, 3)
plt.plot(time_list, theta_error_list, 'b-')
plt.title('Erro de Orientacao')
plt.xlabel('Tempo (s)')
plt.ylabel('erro_theta (rad)')
plt.grid('on')

# Mostrar gráficos

plt.show()

###########################################################################################
###########################################################################################