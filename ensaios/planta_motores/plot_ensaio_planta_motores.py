#! /usr/bin/env python
# coding: utf-8

###########################################################################################
###########################################################################################
## Projeto: Nanook UFC
## Autor: Kaio Douglas Teófilo Rocha
## Email: kaiodtr@gmail.com
###########################################################################################
## Arquivo: Plotter de Ensaios para Levantamento das Plantas dos Motores
## Revisão: 1 [17/03/2017]
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
path += '/planta_motores/resultados/ensaio_%d.txt' % ensaio

# Abertura do arquivo

data_file = open(path, 'r')

# Listas com os dados de cada variável coletada

samples = []        # Amostra
t = []              # Tempo [s]
right_vel_ref = []  # Velocidade de referência direita [RPM]
right_vel = []      # Velocidade direita [RPM]
left_vel_ref = []   # Velocidade de referência esquerda [RPM]
left_vel = []       # Velocidade esquerda [RPM]

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
        right_vel_ref.append(float(line[3]))
        right_vel.append(float(line[4]))        
        left_vel_ref.append(float(line[5]))
        left_vel.append(float(line[6]))

# Fechamento do arquivo

data_file.close()

###########################################################################################
### GERAÇÃO DE GRÁFICOS
###########################################################################################

plt.close('all')
plt.figure(1)

# Velocidade direita [RPM] | Tempo [s]

plt.subplot(1, 2, 1)
plt.plot(t, right_vel_ref, 'r--', t, right_vel, 'b-')
plt.title('Velocidade Direita')
plt.xlabel('Tempo (s)')
plt.ylabel('Velocidade (RPM)')
plt.grid('on')

# Velocidade esquerda [RPM] | Tempo [s]

plt.subplot(1, 2, 2)
plt.plot(t, left_vel_ref, 'r--', t, left_vel, 'b-')
plt.title('Velocidade Esquerda')
plt.xlabel('Tempo (s)')
plt.ylabel('Velocidade (RPM)')
plt.grid('on')

# Mostrar gráficos

plt.show()

###########################################################################################
###########################################################################################