% Primeiro, a função Link é usada para criar cada elo. 
% Os parâmetros da função Link são exatamente os parâmetros
% de Denavit-Hartemberg do elo em questão, nesta ordem: 
% theta = ângulo da junta (rad)
% d = deslocamento do elo (m)
% a = comprimento do elo (m)
% alpha = torção do elo (rad)
% sigma = tipo de junta (0: rotativa ou 1: prismática)
clear all
close all
clc
startup_rvc;
%% Criação dos Links para o braço de 6-juntas
L(1) = Revolute('a', 0, 'alpha', pi/2, 'd', 0.125, 'qlim', [-2.79, 2.79]);
L(2) = Revolute('a', 0.21, 'alpha', 0, 'd', 0, 'offset', pi/2, 'qlim', [-2.09, 2.09]);
L(3) = Revolute('a', -0.075, 'alpha', -pi/2, 'd', 0, 'offset', -pi/2, 'qlim', [0.33, 2.79]);
L(4) = Revolute('a', 0, 'alpha', pi/2, 'd', 0.21, 'qlim', [-2.79, 2.79]);
L(5) = Revolute('a', 0, 'alpha', -pi/2, 'd', 0, 'qlim', [-2.09, 2.09]);
L(6) = Revolute('a', 0, 'alpha', 0, 'd', 0.07, 'qlim', [-6.28, 6.28]);


robot = SerialLink(L, 'name', 'Denso');
% Config inicial para o Denso
q = [0 0 -pi/2 0 0 0];

% Matriz de transformação por cinemática direta(Config Inicial)
Kd = robot.fkine(q);
% q()s gerados por cinemática inversa(Config Inicial)
Ki = robot.ikine(Kd);

%Ponto teste 1
t2 = transl(0.398, 0.0, 0.05);
t2(1:3,1:3) = roty(90);
%Ponto teste 2
t3 = transl(0.326,-0.230, -0.088);
t3(1:3,1:3) = roty(90);
%Cinematica inversa
Ki2 = robot.ikine(t2);
Ki3 = robot.ikine(t3);
robot.plot(Ki);
pause(3)
robot.plot(Ki2);
pause(3)
robot.plot(Ki);
pause(3)
robot.plot(Ki3);

%robot.teach()



%%
% Essas trajetórias serão enviadas para o denso como angulos por cinemática
% inversa
t = 7;
traj = jtraj(Ki, Ki2, t);
traj2 = jtraj(Ki2, Ki, t);

% Aqui vai entrar a coordenada que o usr inserir
traj3 = jtraj(Ki, Ki3, t);
%Kd
%Kd2
%traj
%Trajetória inicial=>cubo
for i = 1:1:7
    [traj(i,1) traj(i,2) traj(i,3) traj(i,4) traj(i,5) traj(i,6)]
    robot.plot([traj(i,1) traj(i,2) traj(i,3) traj(i,4) traj(i,5) traj(i,6)])
    pause(1);
end
%Trajetória cubo=>inicial
for i = 1:1:7
    [traj2(i,1) traj2(i,2) traj2(i,3) traj2(i,4) traj2(i,5) traj2(i,6)]
    robot.plot([traj2(i,1) traj2(i,2) traj2(i,3) traj2(i,4) traj2(i,5) traj2(i,6)])
    pause(1);
end
%Trajetória inicial=>destino
for i = 1:1:7
    [traj3(i,1) traj3(i,2) traj3(i,3) traj3(i,4) traj3(i,5) traj3(i,6)]
    robot.plot([traj3(i,1) traj3(i,2) traj3(i,3) traj3(i,4) traj3(i,5) traj3(i,6)])
    pause(1);
end
%Trajetória destino=>inicial
for i = 1:1:7
    [traj4(i,1) traj4(i,2) traj4(i,3) traj4(i,4) traj4(i,5) traj4(i,6)]
    robot.plot([traj4(i,1) traj4(i,2) traj4(i,3) traj4(i,4) traj4(i,5) traj4(i,6)])
    pause(1);
end

%%
setpoint = [0.1, 0.5, 0];

qnew = [0 0 -pi/2 0 0 0];

w = sqrt((setpoint(1).^2) + (setpoint(2).^2))

qnew(1) = atan2(setpoint(2),setpoint(1));
qnew(2) = atan2(setpoint(3), w)
qnew(3) = atan2(w, setpoint(3))

robot.plot(qnew);

%% 
% Forward and inverse kinematics

%qf = [0 pi/2 -pi/2 0 0 0];
syms q1 q2 q3 q4 q5 q6;
T = robot.fkine([q1 q2 q3 q4 q5 q6]);
%T_i = robot.ikine(T)
%T = robot.fkine(qf);
%J = robot.jacob0(qf);
%q_i = robot.ikine(T)

%% Find the transformation matrix of the last frame.
syms th6;
syms q1 q2 q3 q4 q5 q6;
%last_frame = rotz(th6)* transl(0 0 70)
T01 = transl(0, 0, 0.125) * rotx(pi/2) * rotz(q1)
%%F = transl(0, 0, 70)
