% projeto_lqi.m - LQI do teste de controle, igual a tools/lqi/lqi_design.c.
%
% Pre-requisito: Ad (12x12) e Bd (12x4), o modelo DISCRETO do TCC (Ts = 10 ms)
% que alimenta o Simulink, no workspace. Ordem dos estados:
%   x y z phi theta psi vx vy vz p q r
%
% O resultado deve bater com App/Src/control/lqi_gains.c (gerado em C).

Ts = 0.01;
Q = diag([5 5 500 500 500 50 5 5 5 10 10 5]);   % pesos do TCC
R = 0.1 * eye(4);

% 1) Conferencia: com os pesos do TCC, dlqr deve dar o Kd exportado (Gain5).
Kd = dlqr(Ad, Bd, Q, R);
disp('Kd do TCC (deve ser o do Simulink):'); disp(Kd);

% 2) Subsistema de atitude e altura. No modelo do TCC, x, y, vx e vy nao
%    realimentam a atitude, e a integral de theta ja e (a menos de constante)
%    a velocidade vx: um integrador de theta no modelo completo seria redundante.
sub = [3 4 5 6 9 10 11 12];                % z phi theta psi vz p q r
Ar = Ad(sub, sub);
Br = Bd(sub, :);

% 3) Integradores de z, phi e theta: xi[k+1] = xi[k] + Ts * (z, phi, theta)[k]
Ci = zeros(3, 8);
Ci(1, 1) = 1;   % z
Ci(2, 2) = 1;   % phi
Ci(3, 3) = 1;   % theta
Aa = [Ar, zeros(8, 3); Ts * Ci, eye(3)];
Ba = [Br; zeros(3, 4)];
Qa = blkdiag(Q(sub, sub), diag([1000 10000 10000]));   % int_z, int_phi, int_theta

Ka = dlqr(Aa, Ba, Qa, R);

% 4) De volta aos indices do modelo completo (x, y, vx, vy ficam em zero).
Kx = zeros(4, 12);
Kx(:, sub) = Ka(:, 1:8);
Ki = Ka(:, 9:11);
% Referencia como desvio de estado: du = -Kx (xhat - x_ref) - Ki xi
Kfd = zeros(4, 4);
Kfd(:, 3) = Kx(:, 3);    % r_z
Kfd(:, 4) = Kx(:, 6);    % r_psi

disp('Kx (LQI):');  disp(Kx);
disp('Ki (LQI): colunas int_z int_phi int_theta'); disp(Ki);
disp('Kfd (LQI): colunas r_x r_y r_z r_psi'); disp(Kfd);

% Observacoes para o Simulink:
% - o Kalman (M, L) continua o do TCC;
% - o Kalman recebe so a parte do LQR (-Kx xhat + Kfd r), nao a dos
%   integradores: ela compensa uma perturbacao que o modelo nao tem;
% - os integradores usam as MEDIDAS (z do MTF, phi e theta do BNO), nao xhat;
% - anti-windup no firmware: correcao de cada integrador <= 250 rad/s por
%   motor, congela com motor saturado, e no ar o erro de angulo integrado e
%   limitado a +-0,08 rad.
