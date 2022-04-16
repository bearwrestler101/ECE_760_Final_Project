%% Script para validación numérica de la cinemática inversa

clc
close all, %clear variables

L(1)=Link('revolute','d',0.36,'a',0,'alpha',0,'offset',0);
L(2)=Link('revolute','d',0,'a',0,'alpha',-pi/2,'offset',0);
L(3)=Link('revolute','d',0.42,'a',0,'alpha',pi/2,'offset',0);
L(4)=Link('revolute','d',0,'a',0,'alpha',pi/2,'offset',0);
L(5)=Link('revolute','d',0.4,'a',0,'alpha',-pi/2,'offset',0);
L(6)=Link('revolute','d',0,'a',0,'alpha',-pi/2,'offset',0);
L(7)=Link('revolute','d',0.126,'a',0,'alpha',pi/2,'offset',0);

L(1).mdh=1;
L(2).mdh=1;
L(3).mdh=1;
L(4).mdh=1;
L(5).mdh=1;
L(6).mdh=1;
L(7).mdh=1;

R = SerialLink(L, 'name', 'KUKA iiwa 14 R820');


DH = [0 .360 0 -pi/2 0
    0 0 0 pi/2 0
    0 .420 0 pi/2, 0
    0 0 0 -pi/2 0
    0 .400 0 -pi/2 0
    0 0 0 pi/2 0
    0 .126 0 0 0];

R_SDH = SerialLink(DH, 'name', 'KUKA iiwa 14 R820 SDH');

q = [0, 0, 0, 0, 0, 0, 0];
workspace = [-2, 2, -2, 2, -2, 3];

R.qlim = deg2rad([-170, 170
                -120, 120
                -170, 170
                -120, 120
                -170, 170
                -120, 120
                -175, 175]);

a = deg2rad(-120);
b = deg2rad(120);
       
n = 5;
q = a + (b-a)*rand(7,1);

T = R.fkine(q);

T_list = zeros(n, 4, 4);

T_error = zeros(n, 1);
q_error = zeros(n, 1);

for i=1:n
    q = a + (b-a)*rand(7,1);
    T = R.fkine(q);
    q_inv = inverse_kinematics(q(3), T, R_SDH);


    for j=1:8
       T_error(i) = T_error(i) + sum(sum(abs( T - R.fkine(q_inv(:, j) ) ) ) );    
    end
    
    q_error(i) = sum(abs(alikeness(q, q_inv) - q));
    
end

