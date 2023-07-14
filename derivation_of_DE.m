% ���O�����W����������?o�v�?�O����
% ?V���ȃ��f���Ɋ�Â��v�?�O������g�ނƂ��ɉ^����������?o���̂Ɏg���Ă�
% 2015/02/20 �� �f��
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
%-------���_��?錾!!!-------------------------------------------------------------------------------
%[p(1)�̎��_�̎���; p(2)�̎��_�̎���; p(3)�̎��_�̎���]?E?E?E
% ���f���̎��_��mass�Ɋi�[����
mass = [mb;m1;m2;m1;m2]; 
%��ʉ�?��W?E���x?E�����x���`����
%��ʉ�?��W�x�N�g��
q = [th1;th2;th3;th4;thb;x1;z1];
%��ʉ����x�x�N�g��
dq = [dth1;dth2;dth3;dth4;dthb;dx1;dz1];
%��ʉ������x�x�N�g��
ddq = [ddth1;ddth2;ddth3;ddth4;ddthb;ddx1;ddz1];
%%%------------------------------------------------------------%%%%%%%%%%%%
%%%-----�ʒu�x�N�g���̒�`--------------------------------------%%%%%%%%%%%%

% �e�����N�̓����ϊ�?s��̒�`
Eb = [rot_mat(2,thb) [x1;0;z1];
      zeros(1,3) 1];
E1 = Eb*[rot_mat(2,th1) zeros(3,1);zeros(1,3) 1];
E2 = E1*[rot_mat(2,th2) [0;0;l1];zeros(1,3) 1];
E3 = Eb*[rot_mat(2,th3) zeros(3,1);zeros(1,3) 1];
E4 = E3*[rot_mat(2,th4) [0;0;l1];zeros(1,3) 1];

% �e���ʂ�?��W��?o
Pb = [eye(3) zeros(3,1)]*Eb*[0;0;lb/2;1]
P1 = [eye(3) zeros(3,1)]*E1*[0;0;l1/2;1]
P2 = [eye(3) zeros(3,1)]*E2*[0;0;l2/2;1]
P3 = [eye(3) zeros(3,1)]*E3*[0;0;l1/2;1]
P4 = [eye(3) zeros(3,1)]*E4*[0;0;l2/2;1]

Ps1 = [eye(3) zeros(3,1)]*E1*[0;0;l1;1]
Pw1 = [eye(3) zeros(3,1)]*E3*[0;0;l1;1]

% �e��?�̈ʒu?��W��?o
Pst_tip = [eye(3) zeros(3,1)]*E2*[0;0;l2;1]
Psw_tip = [eye(3) zeros(3,1)]*E4*[0;0;l2;1]

% ��?�̃��R�r�A�����v�Z
J_Pst_tip = jacobian(Pst_tip,q)
J_Psw_tip = jacobian(Psw_tip,q)
J_Pst_tip(2,:) = []
J_Psw_tip(2,:) = []

simplify(J_Pst_tip*dq)
dJ_Pst_tip = simplify([diff(J_Pst_tip,th1)*dq,diff(J_Pst_tip,th2)*dq,diff(J_Pst_tip,th3)*dq,diff(J_Pst_tip,th4)*dq,diff(J_Pst_tip,thb)*dq,diff(J_Pst_tip,x1)*dq,diff(J_Pst_tip,z1)*dq])

% mass�Ɋi�[����?��ԂɎ��_�̈ʒu��P�Ɋi�[����
P(:,1) = Pb;
P(:,2) = P1; 
P(:,3) = P2; 
P(:,4) = P3; 
P(:,5) = P4; 

% �e�����N�̎p?���\�������]?s���?o
Rb = Eb(1:3,1:3);
R1 = E1(1:3,1:3);
R2 = E2(1:3,1:3);
R3 = E3(1:3,1:3);
R4 = E4(1:3,1:3);

% ��]?s��̔������v�Z
for i = 1 : 3
    for j = 1 : 3
        dRb(i,j) = jacobian(Rb(i,j),q)*dq;
        dR1(i,j) = jacobian(R1(i,j),q)*dq;
        dR2(i,j) = jacobian(R2(i,j),q)*dq;
        dR3(i,j) = jacobian(R3(i,j),q)*dq;
        dR4(i,j) = jacobian(R4(i,j),q)*dq;
    end
end

% �e�����N�̊p���x���Z?o
omega_b = simplify([0,1,0]*sks2vec(dRb*inv(Rb)))
omega_1 = simplify([0,1,0]*sks2vec(dR1*inv(R1)))
omega_2 = simplify([0,1,0]*sks2vec(dR2*inv(R2)))
omega_3 = simplify([0,1,0]*sks2vec(dR3*inv(R3)))
omega_4 = simplify([0,1,0]*sks2vec(dR4*inv(R4)))

%----------------------------------------------------------------
% %��?�ʒu�̃��R�r�A��

% CoM �̎Z?o
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

%�^���ʂ��Z?o
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

% �e���_��?d?S���x���v�Z
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

% % �p�^���ʂ̌v�Z
%  %��P?�
%  PG_z_1 = I1*dth1 + I2*dth1 + I3*dth2 + I4*dth3 + I4*dth4 + I3*dth5 + I2*dth6 + I1*dth6 + I5*dth7
%  
%  %��Q?�
%  PG_z_2 = 0;
%  for i = 1:10
%      PG_z_2 = PG_z_2 + mass(i,1)*(P(3,i)*dP(1,i)-P(1,i)*dP(3,i));
%  end
%     PG_z_2 = simplify(PG_z_2)
%  
%  %��R?�
%  PG_z_3 = -sum(mass)*(CoM(3,1)*dCoM(1,1)-CoM(1,1)*dCoM(3,1))
%  
%  PG_z = PG_z_1 + PG_z_2 + PG_z_3;
 
% �p�^���ʂ̔���
%  dPG_z_1 = jacobian(PG_z_1,dq)*ddq;
%  dPG_z_2 = jacobian(PG_z_2,q)*dq + jacobian(PG_z_2,dq)*ddq;
%  dPG_z_3 = jacobian(PG_z_3,q)*dq + jacobian(PG_z_3,dq)*ddq;
 
% dPG_z = simplify(jacobian(PG_z,q)*dq + jacobian(PG_z,dq)*ddq)
% 
% 
%  JdPG_z = (simplify(jacobian(PG_z,q)))'
%  JddPG_z = (simplify(jacobian(PG_z,dq)))'
 
 %dPG_z = simplify(JdPG_z' * dq)
 

%-------- ��?��ɂ��^���G�l���M?[ -------------------------------------------------------------
%��?���?[�?���g�̉^���G�l���M?[��������
KE_inertia = 0.5*(Ib*omega_b^2 + I1*omega_1^2 + I2*omega_2^2 + I3*omega_3^2 + I4*omega_4^2);
%-------- �e?��G�l���M?[ -------------------------------------------------------------------------
PE_spring = 0;
%-------- �S?��ɂ�鑹�� -------------------------------------------------------------------------
DE = 0;
%--------���͂͂����܂�?C��͎����I�ɎZ?o�����----------------------------------------------

%-------- ���x�x�N�g���Z?o -----------------------------------------------------------------------
for i = 1:1:length(mass)
dPx(i,:)=jacobian(P(1,i),q)*dq;
dPz(i,:)=jacobian(P(3,i),q)*dq;
end
%-------- ?����� ----------------------------------------------------------------------------------
KE_all = 0;
PE_all = 0;
%---------------------------------------------------------------------------------------------------
for i = 1:1:length(mass)
KE = 0.5*mass(i)*(dPx(i)^2+dPz(i)^2);
PE = mass(i)*gravi*(-P(1,i)*sin(phi)+P(3,i)*cos(phi));
KE_all = KE_all + KE;
PE_all = PE_all + PE;
end
%---------- �G�l���M?[�?�a ------------------------------------------------------------------------
KE_all = simplify(KE_all +KE_inertia)
PE_all = simplify(PE_all +PE_spring)
%---------- ���O���W�A�� ---------------------------------------------------------------------------
Lag = simplify(KE_all - PE_all);
q_dq = [q;dq];
dq_ddq = [dq;ddq];
%---------- �Δ����⎞�Ԕ��� ----------------------------------------------------------------------
for i = 1:1:length(q)
Ldq(i,:) = diff(Lag,dq(i,:));
Ldq_dt(i,:) = jacobian(Ldq(i,:),q_dq)*dq_ddq;
Lq(i,:) = diff(Lag,q(i,:));
DEdq(i,:) = diff(DE,dq(i,:));
tau(i,:)= Ldq_dt(i,:) - Lq(i,:) + DEdq(i,:);
end
%---------- �^���������̊e?����Z?o ---------------------------------------------------------------
M = simplify(jacobian(tau,ddq));
G = simplify(collect(tau-subs(tau,gravi,0),gravi));
M_line = M*ddq;
C = simplify(tau-M_line-G);
%---------- �^����������?�����\�� ---------------------------------------------------------------
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

% ��]?s����?�߂��?�
function R = rot_mat(mode,theta)

    if mode == 1 % X������̉�]��?�?�
        
        R = [1 0 0;
             0 cos(theta) -sin(theta);
             0 sin(theta) cos(theta)];
        
    elseif mode == 2 % Y������̉�]��?�?�
        
        R = [cos(theta) 0 sin(theta);
             0 1 0;
             -sin(theta) 0 cos(theta)];
        
    elseif mode == 3 % Z������̉�]��?�?�
    
        R = [cos(theta) -sin(theta) 0;
             sin(theta) cos(theta) 0;
             0 0 1];
        
    end

end

% �c��?�?s�񂩂�x�N�g����?�?����鑀?�
function omega = sks2vec(R)

    omega = [-R(2,3);R(1,3);-R(1,2)];

end