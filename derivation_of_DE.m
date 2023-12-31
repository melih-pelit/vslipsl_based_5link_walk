% ラグランジュ方程式導?oプ�?グラム
% ?Vたなモデルに基づくプ�?グラムを組むときに運動方程式を?oすのに使ってね
% 2015/02/20 林 映光
% function derivation_of_DE
clear all
close all

syms th1 th2 th3 th4 th5 th6 th7 thb x1 z1 lb real
syms dth1 dth2 dth3 dth4 dth5 dth6 dth7 dthb dx1 dz1 dlb real 
syms ddth1 ddth2 ddth3 ddth4 ddth5 ddth6 ddth7 ddthb ddx1 ddz1 ddlb real 
syms m1 m2 m3 m4 m5 mh mb real 
syms I1 I2 I3 I4 I5 Ib real                                   
syms a1 a2 a3 a4 a5 b1 b2 b3 b4 b5 l1 l2 l3 l4 l5 c1 real 
syms phi real 
syms tau1 tau2 tau3 tau4 tau5 tau6 tau7 real 
syms gravi real 
syms Ka Da K1 D1 K2 D2 real 
syms l0 alpha_d real 
                   
tic
%-------質点の?骭ｾ!!!-------------------------------------------------------------------------------
%[p(1)の質点の質量; p(2)の質点の質量; p(3)の質点の質量]?E?E?E
% モデルの質点をmassに格納する
mass = [mb;m1;m2;m1;m2]; 
%一般化?ﾀ標?E速度?E加速度を定義する
%一般化?ﾀ標ベクトル
q = [th1;th2;th3;th4;thb;x1;z1];
%一般化速度ベクトル
dq = [dth1;dth2;dth3;dth4;dthb;dx1;dz1];
%一般化加速度ベクトル
ddq = [ddth1;ddth2;ddth3;ddth4;ddthb;ddx1;ddz1];
%%%------------------------------------------------------------%%%%%%%%%%%%
%%%-----位置ベクトルの定義--------------------------------------%%%%%%%%%%%%

% 各リンクの同時変換?s列の定義
Eb = [rot_mat(2,thb) [x1;0;z1];
      zeros(1,3) 1];
E1 = Eb*[rot_mat(2,th1) zeros(3,1);zeros(1,3) 1];
E2 = E1*[rot_mat(2,th2) [0;0;l1];zeros(1,3) 1];
E3 = Eb*[rot_mat(2,th3) zeros(3,1);zeros(1,3) 1];
E4 = E3*[rot_mat(2,th4) [0;0;l1];zeros(1,3) 1];

% 各質量の?ﾀ標を抽?o
Pb = [eye(3) zeros(3,1)]*Eb*[0;0;lb/2;1]
P1 = [eye(3) zeros(3,1)]*E1*[0;0;l1/2;1]
P2 = [eye(3) zeros(3,1)]*E2*[0;0;l2/2;1]
P3 = [eye(3) zeros(3,1)]*E3*[0;0;l1/2;1]
P4 = [eye(3) zeros(3,1)]*E4*[0;0;l2/2;1]

Ps1 = [eye(3) zeros(3,1)]*E1*[0;0;l1;1]
Pw1 = [eye(3) zeros(3,1)]*E3*[0;0;l1;1]

% 各足?謔ﾌ位置?ﾀ標を抽?o
Pst_tip = [eye(3) zeros(3,1)]*E2*[0;0;l2;1]
Psw_tip = [eye(3) zeros(3,1)]*E4*[0;0;l2;1]

% 足?謔ﾌヤコビアンを計算
J_Pst_tip = jacobian(Pst_tip,q)
J_Psw_tip = jacobian(Psw_tip,q)
J_Pst_tip(2,:) = []
J_Psw_tip(2,:) = []

simplify(J_Pst_tip*dq)
dJ_Pst_tip = simplify([diff(J_Pst_tip,th1)*dq,diff(J_Pst_tip,th2)*dq,diff(J_Pst_tip,th3)*dq,diff(J_Pst_tip,th4)*dq,diff(J_Pst_tip,thb)*dq,diff(J_Pst_tip,x1)*dq,diff(J_Pst_tip,z1)*dq])

% massに格納した?�番に質点の位置をPに格納する
P(:,1) = Pb;
P(:,2) = P1; 
P(:,3) = P2; 
P(:,4) = P3; 
P(:,5) = P4; 

% 各リンクの姿?ｨを表現する回転?s列を抽?o
Rb = Eb(1:3,1:3);
R1 = E1(1:3,1:3);
R2 = E2(1:3,1:3);
R3 = E3(1:3,1:3);
R4 = E4(1:3,1:3);

% 回転?s列の微分を計算
for i = 1 : 3
    for j = 1 : 3
        dRb(i,j) = jacobian(Rb(i,j),q)*dq;
        dR1(i,j) = jacobian(R1(i,j),q)*dq;
        dR2(i,j) = jacobian(R2(i,j),q)*dq;
        dR3(i,j) = jacobian(R3(i,j),q)*dq;
        dR4(i,j) = jacobian(R4(i,j),q)*dq;
    end
end

% 各リンクの角速度を算?o
omega_b = simplify([0,1,0]*sks2vec(dRb*inv(Rb)))
omega_1 = simplify([0,1,0]*sks2vec(dR1*inv(R1)))
omega_2 = simplify([0,1,0]*sks2vec(dR2*inv(R2)))
omega_3 = simplify([0,1,0]*sks2vec(dR3*inv(R3)))
omega_4 = simplify([0,1,0]*sks2vec(dR4*inv(R4)))

%----------------------------------------------------------------
% %足?謌ﾊ置のヤコビアン

% CoM の算?o
% CoMx = (m1*P1(1) + m2*P2(1) + m3*P3(1) + m4*P4(1) + mh*P5(1) + m4*P6(1) + m3*P7(1) + m2*P8(1) + m1*P9(1) + m5*P10(1))/(m1+m2+m3+m4+mh+m4+m3+m2+m1+m5);
% CoMz = (m1*P1(3) + m2*P2(3) + m3*P3(3) + m4*P4(3) + mh*P5(3) + m4*P6(3) + m3*P7(3) + m2*P8(3) + m1*P9(3) + m5*P10(3))/(m1+m2+m3+m4+mh+m4+m3+m2+m1+m5);
% CoM = [CoMx;0;CoMz];
% %simplify([CoM(1,1);CoM(3,1)])
% JCoM = simplify(jacobian(CoM,q))
% dJCoM = simplify([diff(JCoM,th1)*dq,diff(JCoM,th2)*dq,diff(JCoM,th3)*dq,diff(JCoM,th4)*dq,diff(JCoM,th5)*dq,diff(JCoM,th6)*dq,diff(JCoM,th7)*dq,diff(JCoM,x1)*dq,diff(JCoM,z1)*dq]);
% dJCoM = [dJCoM(1,:);dJCoM(3,:)]
% dCoM= simplify(jacobian(CoM,q)*dq)
% CoMz;
% dCoMz= simplify(jacobian(CoMz,q)*dq);

%運動量を算?o
% Po1 = simplify(m1*jacobian(P1,q)*dq)
% Po2 = simplify(m2*jacobian(P2,q)*dq)
% Po3 = simplify(m3*jacobian(P3,q)*dq)
% Po4 = simplify(m4*jacobian(P4,q)*dq)
% Po5 = simplify(mh*jacobian(P5,q)*dq)
% Po6 = simplify(m4*jacobian(P6,q)*dq)
% Po7 = simplify(m3*jacobian(P7,q)*dq)
% Po8 = simplify(m2*jacobian(P8,q)*dq)
% Po9 = simplify(m1*jacobian(P9,q)*dq)
% Po10 = simplify(m5*jacobian(P10,q)*dq)

% 各質点の?d?S速度を計算
dP(:,1) = simplify(jacobian(Pb,q)*dq);
dP(:,2) = simplify(jacobian(P1,q)*dq);
dP(:,3) = simplify(jacobian(P2,q)*dq);
dP(:,4) = simplify(jacobian(P3,q)*dq);
dP(:,5) = simplify(jacobian(P4,q)*dq);

dP1 = dP(:,1)
dP2 = dP(:,2)
dP3 = dP(:,3)
dP4 = dP(:,4)
dP5 = dP(:,5)

% % 角運動量の計算
%  %第１?�
%  PG_z_1 = I1*dth1 + I2*dth1 + I3*dth2 + I4*dth3 + I4*dth4 + I3*dth5 + I2*dth6 + I1*dth6 + I5*dth7
%  
%  %第２?�
%  PG_z_2 = 0;
%  for i = 1:10
%      PG_z_2 = PG_z_2 + mass(i,1)*(P(3,i)*dP(1,i)-P(1,i)*dP(3,i));
%  end
%     PG_z_2 = simplify(PG_z_2)
%  
%  %第３?�
%  PG_z_3 = -sum(mass)*(CoM(3,1)*dCoM(1,1)-CoM(1,1)*dCoM(3,1))
%  
%  PG_z = PG_z_1 + PG_z_2 + PG_z_3;
 
% 角運動量の微分
%  dPG_z_1 = jacobian(PG_z_1,dq)*ddq;
%  dPG_z_2 = jacobian(PG_z_2,q)*dq + jacobian(PG_z_2,dq)*ddq;
%  dPG_z_3 = jacobian(PG_z_3,q)*dq + jacobian(PG_z_3,dq)*ddq;
 
% dPG_z = simplify(jacobian(PG_z,q)*dq + jacobian(PG_z,dq)*ddq)
% 
% 
%  JdPG_z = (simplify(jacobian(PG_z,q)))'
%  JddPG_z = (simplify(jacobian(PG_z,dq)))'
 
 %dPG_z = simplify(JdPG_z' * dq)
 

%-------- 慣?ｫによる運動エネルギ?[ -------------------------------------------------------------
%慣?ｫモ?[�?ントの運動エネルギ?[を代入する
KE_inertia = 0.5*(Ib*omega_b^2 + I1*omega_1^2 + I2*omega_2^2 + I3*omega_3^2 + I4*omega_4^2);
%-------- 弾?ｫエネルギ?[ -------------------------------------------------------------------------
PE_spring = 0;
%-------- 粘?ｫによる損失 -------------------------------------------------------------------------
DE = 0;
%--------入力はここまで?C後は自動的に算?oされる----------------------------------------------

%-------- 速度ベクトル算?o -----------------------------------------------------------------------
for i = 1:1:length(mass)
dPx(i,:)=jacobian(P(1,i),q)*dq;
dPz(i,:)=jacobian(P(3,i),q)*dq;
end
%-------- ?炎�化 ----------------------------------------------------------------------------------
KE_all = 0;
PE_all = 0;
%---------------------------------------------------------------------------------------------------
for i = 1:1:length(mass)
KE = 0.5*mass(i)*(dPx(i)^2+dPz(i)^2);
PE = mass(i)*gravi*(-P(1,i)*sin(phi)+P(3,i)*cos(phi));
KE_all = KE_all + KE;
PE_all = PE_all + PE;
end
%---------- エネルギ?[�?和 ------------------------------------------------------------------------
KE_all = simplify(KE_all +KE_inertia)
PE_all = simplify(PE_all +PE_spring)
%---------- ラグラジアン ---------------------------------------------------------------------------
Lag = simplify(KE_all - PE_all);
q_dq = [q;dq];
dq_ddq = [dq;ddq];
%---------- 偏微分や時間微分 ----------------------------------------------------------------------
for i = 1:1:length(q)
Ldq(i,:) = diff(Lag,dq(i,:));
Ldq_dt(i,:) = jacobian(Ldq(i,:),q_dq)*dq_ddq;
Lq(i,:) = diff(Lag,q(i,:));
DEdq(i,:) = diff(DE,dq(i,:));
tau(i,:)= Ldq_dt(i,:) - Lq(i,:) + DEdq(i,:);
end
%---------- 運動方程式の各?�を算?o ---------------------------------------------------------------
M = simplify(jacobian(tau,ddq));
G = simplify(collect(tau-subs(tau,gravi,0),gravi));
M_line = M*ddq;
C = simplify(tau-M_line-G);
%---------- 運動方程式の?ｬ分を表示 ---------------------------------------------------------------
fprintf('-----------inertia_matrix-------------\n\n')
for i=1:1:length(q)
    for j= 1:1:length(q)
        fprintf('M%d%d', i ,j)
        M(i,j)
    end
end
fprintf('-----------Coliori and centrifugal force matrix (vector)-------------\n\n')
for i=1:1:length(q)
    fprintf('C%d', i)
    C(i,:)
end
 fprintf('-----------gravitational_vector-----------\n\n')
for i=1:1:length(q)
    fprintf('G%d', i)
    G(i,:)
end
% save('calculation_result.txt','M','C','G','KE_all','PE_all')
toc;
% end

% 回転?s列を�?める関?�
function R = rot_mat(mode,theta)

    if mode == 1 % X軸周りの回転の?�?�
        
        R = [1 0 0;
             0 cos(theta) -sin(theta);
             0 sin(theta) cos(theta)];
        
    elseif mode == 2 % Y軸周りの回転の?�?�
        
        R = [cos(theta) 0 sin(theta);
             0 1 0;
             -sin(theta) 0 cos(theta)];
        
    elseif mode == 3 % Z軸周りの回転の?�?�
    
        R = [cos(theta) -sin(theta) 0;
             sin(theta) cos(theta) 0;
             0 0 1];
        
    end

end

% 歪対?ﾌ?s列からベクトルを?ｶ?ｬする操?�
function omega = sks2vec(R)

    omega = [-R(2,3);R(1,3);-R(1,2)];

end