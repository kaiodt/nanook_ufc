#! /usr/bin/env python
# coding: utf-8

###########################################################################################
###########################################################################################
## Projeto: Nanook UFC
## Autor: Kaio Douglas Teófilo Rocha
## Email: kaiodtr@gmail.com
###########################################################################################
## Arquivo: Plotter de Ensaios do Controlador Embarcado da Base
## Revisão: 1 [03/03/2017]
###########################################################################################
###########################################################################################

from os.path import expanduser
import matplotlib.pyplot as plt 

###########################################################################################
### INICIALIZAÇÃO
###########################################################################################

# Número do ensaio

ensaio = int(raw_input('Número do Ensaio: '))

# Arquivo com os dados

home = expanduser('~')
path = home + '/ros_catkin_ws/src/nanook_ufc/ensaios'
path += '/controlador_base/resultados/ensaio_%d.txt' % ensaio

# Abertura do arquivo

data_file = open(path, 'r')

# Listas com os dados de cada variável coletada

samples = []        # Amostra
t = []              # Tempo [s]
left_ref = []       # Velocidade de referência esquerda [RPM]
left_vel = []       # Velocidade esquerda [RPM]
right_ref = []      # Velocidade de referência direita [RPM]
right_vel = []      # Velocidade direita [RPM]
x = []              # Posição no eixo x [m]
y = []              # Posição no eixo y [m]
theta = []          # Orientação [graus]
v = []              # Velocidade linear [m/s]
w = []              # Velocidade angular [rad/s]

###########################################################################################
### LEITURA DO ARQUIVO
###########################################################################################

# Exibição das linhas com informações e armazenamento de linhas com dados (iniciadas com #)
# nas respectivas listas

for line in data_file:
    if '#' not in line:                     # Linha com informações do ensaio
        print line
    else:                                   # Linha com dados
        line = line.split()
        samples.append(int(line[1]))
        t.append(float(line[2]))
        left_ref.append(float(line[3]))
        left_vel.append(float(line[4]))
        right_ref.append(float(line[5]))
        right_vel.append(float(line[6]))
        x.append(float(line[7]))
        y.append(float(line[8]))
        theta.append(float(line[9]))
        v.append(float(line[10]))
        w.append(float(line[11]))

# Fechamento do arquivo

data_file.close()

###########################################################################################
### GERAÇÃO DE GRÁFICOS
###########################################################################################

# Tratamento da lista de tempo (início em 0.0 s)

t0 = t[0]

for i in range(len(t)):
    t[i] -= t0

### Gráficos ###

plt.close('all')

# Pose

plt.figure(1)

# Posição no eixo x [m] | Tempo [s]

plt.subplot(2, 2, 1)
plt.plot(t, x, 'b-')
plt.title('x')
plt.xlabel('Tempo (s)')
plt.ylabel('x (m)')
plt.grid('on')

# Posição no eixo y [m] | Tempo [s]

plt.subplot(2, 2, 2)
plt.plot(t, y, 'b-')
plt.title('y')
plt.xlabel('Tempo (s)')
plt.ylabel('y (m)')
plt.grid('on')

# Orientação [graus] | Tempo [s]

plt.subplot(2, 2, 3)
plt.plot(t, theta, 'b-')
plt.title('theta')
plt.xlabel('Tempo (s)')
plt.ylabel('theta (graus)')
plt.grid('on')

# Posição no eixo y [m] | Posição no eixo x [m]

plt.subplot(2, 2, 4)
plt.plot(x, y, 'b-')
plt.title('Pos')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.grid('on')

# Velocidade

plt.figure(2)

# Velocidade esquerda [RPM] | Tempo [s]

plt.subplot(2, 2, 1)
plt.plot(t, left_ref, 'r--')
plt.hold(True)
plt.plot(t, left_vel, 'b-')
plt.title('Velocidade Esquerda')
plt.xlabel('Tempo (s)')
plt.ylabel('v_left (RPM)')
plt.grid('on')
plt.hold(False)

# Velocidade direita [RPM] | Tempo [s]

plt.subplot(2, 2, 2)
plt.plot(t, right_ref, 'r--')
plt.hold(True)
plt.plot(t, right_vel, 'b-')
plt.title('Velocidade Direita')
plt.xlabel('Tempo (s)')
plt.ylabel('v_right (RPM)')
plt.grid('on')
plt.hold(False)

# Velocidade linear [m/s] | Tempo [s]

plt.subplot(2, 2, 3)
plt.plot(t, v, 'b-')
plt.title('Velocidade Linear')
plt.xlabel('Tempo (s)')
plt.ylabel('v (m/s)')
plt.grid('on')

# Velocidade angular [rad/s] | Tempo [s]

plt.subplot(2, 2, 4)
plt.plot(t, w, 'b-')
plt.title('Velocidade Angular')
plt.xlabel('Tempo (s)')
plt.ylabel('w (rad/s)')
plt.grid('on')

# Mostrar gráficos

plt.show()

###########################################################################################
###########################################################################################