%% IDENTIFICAÇÃO DAS PLANTAS DOS MOTORES

clc
clear
close all

% Leitura do arquivo com os dados do ensaio

data_file = fopen('ensaio.txt');
data = textscan(data_file, '%d %f %f %f %f %f');

% Vetores com os dados separados

samples = data{1};
time = data{2};
right_vel_ref = data{3};
right_vel = data{4};
left_vel_ref = data{5};
left_vel = data{6};

% Máximas velocidades lidas

right_max_vel = 30.30606;
left_max_vel = 29.87750;

% Dados da amostragem do ensaio de obtenção das plantas

f = 50;     % Frequência [Hz]
Ts = 1 / f; % Período [s]

% Criando objeto iddata a partir dos dados de saída e entrada

right_data = iddata(right_vel, right_vel_ref, Ts);
left_data = iddata(left_vel, left_vel_ref, Ts);

% Estimativa das funções de transferência em tempo contínuo

right_tf = tfest(right_data, 1);
left_tf = tfest(left_data, 1);

right_tf = tf(right_tf, 'measured');
left_tf = tf(left_tf, 'measured');

% Dados da amostragem na malha de velocidade (K64F)

f = 1000;   % Frequência [Hz]
Ts = 1 / f; % Período [s]

% Conversão das funções de transferência para tempo discreto

right_tf_d = c2d(right_tf, Ts);
left_tf_d = c2d(left_tf, Ts);

% Plots comparando as respostas original, estimada contínua e estimada
% discreta

figure(1);

% Motor direito
subplot(1, 2, 1);
plot(time, right_vel_ref, 'r--', time, right_vel, 'b-'); hold on;
step(right_vel_ref(1) * right_tf);
step(right_vel_ref(1) * right_tf_d);
title('Planta - Motor Direito')
xlabel('Tempo (s)')
ylabel('Velocidade (RPM)')
legend('Referência', 'Original', 'FT Contínua', 'FT Discreta')

% Motor esquerdo
subplot(1, 2, 2);
plot(time, left_vel_ref, 'r--', time, left_vel, 'b-'); hold on;
step(left_vel_ref(1) * left_tf);
step(left_vel_ref(1) * left_tf_d)
title('Planta - Motor Esquerdo')
xlabel('Tempo (s)')
ylabel('Velocidade (RPM)')
legend('Referência', 'Original', 'FT Contínua', 'FT Discreta')

%% PROJETO DOS CONTROLADORES

% TESTE

clc
clear all
close all

Ts = 1/1000;

right_tf = tf(30.5, [0.3702 1]);
right_tf_d = c2d(right_tf, Ts);

left_tf = tf(30.37, [0.378406 1]);
left_tf_d = c2d(left_tf, Ts);

% Características de resposta de malha fechada desejadas

t_rise = 0.18;  % Tempo de subida desejado
Mp = 0.001;      % Overshoot máximo desejado

% Frequência natural e amortecimento correspondetes

[Wn, ksi] = omega_dmp(t_rise, Mp);

% Função de transferência de malha fechada desejada (contínua)

D = tf(Wn^2, [1 2*Wn*ksi Wn^2]);

% Função de transferência de malha fechada desejada (discreta)

Dd = c2d(D, Ts);

% Coeficientes do polinômio característico desejado

[~, p] = tfdata(Dd, 'v');

% Coeficientes das funções de transferência de malha aberta

[right_b, right_a] = tfdata(right_tf_d, 'v');   % Motor direito
[left_b, left_a] = tfdata(left_tf_d, 'v');      % Motor esquerdo

% Parâmetros dos controladores

display('Parâmetros dos controladores:');

% Motor direito

display('Motor Direito:');
right_r0 = (p(2) - right_a(2) + 1) / right_b(2)
right_r1 = (p(3) + right_a(2)) / right_b(2)

% Motor esquerdo

display('Motor Esquerdo:');
left_r0 = (p(2) - left_a(2) + 1) / left_b(2)
left_r1 = (p(3) + left_a(2)) / left_b(2)

% Controladores

C_right = tf([right_r0 right_r1], [1 -1], Ts);
C_left = tf([left_r0 left_r1], [1 -1], Ts);

% Funções de transferência de malha fechada

G_right = feedback(C_right * right_tf_d, 1);
G_left = feedback(C_left * left_tf_d, 1);

% Plots comparando a resposta desejada com as respostas de malha aberta e
% fechada de cada motor

figure(2);

right_vel_ref = [30.5];
left_vel_ref = [30.37];

% Motor direito
subplot(1, 2, 1);
step(right_vel_ref(1) * Dd); hold on;
% step(right_vel_ref(1) * right_tf);
step(right_tf);
step(right_vel_ref(1) * G_right);
title('Funções de Transferência - Motor Direito')
xlabel('Tempo')
ylabel('Velocidade (RPM)')
legend('Desejada', 'Malha Aberta', 'Malha Fechada')

%plot(time, right_vel_ref, 'r--', time, right_vel, 'b-'); hold on;
%step(right_vel_ref(1) * right_tf);
%step(right_vel_ref(1) * right_tf_d);

% Motor esquerdo
subplot(1, 2, 2);
step(left_vel_ref(1) * Dd); hold on;
step(left_tf);
% step(left_vel_ref(1) * left_tf);
step(left_vel_ref(1) * G_left);
title('Funções de Transferência - Motor Esquerdo')
xlabel('Tempo')
ylabel('Velocidade (RPM)')
legend('Desejada', 'Malha Aberta', 'Malha Fechada')