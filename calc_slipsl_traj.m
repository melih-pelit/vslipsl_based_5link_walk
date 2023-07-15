function dc = calc_slipsl_traj(dc)
% constant parameters
m_M = dc.const_param.m_M;
m_swLeg = dc.const_param.m_swLeg;
m_swFoot = dc.const_param.m_swFoot;
L_thigh = dc.const_param.L_thigh;
I_swLeg = dc.const_param.I_swLeg;
I_swFoot = dc.const_param.I_swFoot;
gravi = dc.const_param.gravi;

x_M = dc.simout_ss(1,:)';
y_M = dc.simout_ss(2,:)';
theta = dc.simout_ss(3,:)';
r = dc.simout_ss(4,:)';

dx_M = dc.simout_ss(5,:)';
dy_M = dc.simout_ss(6,:)';
dtheta = dc.simout_ss(7,:)';
dr = dc.simout_ss(8,:)';

dc.ss_traj.x_CoM_ss = (m_swLeg*(x_M + (L_thigh*cos(theta))/2) + m_M*x_M + m_swFoot*(x_M + cos(theta).*(L_thigh + r)))/(m_M + m_swLeg + m_swFoot);
dc.ss_traj.y_CoM_ss = (m_swFoot*(y_M + sin(theta).*(L_thigh + r)) + m_swLeg*(y_M + (L_thigh*sin(theta))/2) + m_M*y_M)/(m_M + m_swLeg + m_swFoot);

dc.ss_traj.x_sw = x_M + cos(theta).*(L_thigh + r);
dc.ss_traj.y_sw = y_M + sin(theta).*(L_thigh + r);

dc.ss_traj.dx_CoM_ss = dx_M + (dr.*m_swFoot.*cos(theta))/(m_M + m_swLeg + m_swFoot) - (dtheta.*sin(theta).*((L_thigh*m_swLeg)/2 + L_thigh*m_swFoot + m_swFoot*r))/(m_M + m_swLeg + m_swFoot);
dc.ss_traj.dy_CoM_ss = dy_M + (dr.*m_swFoot.*sin(theta))/(m_M + m_swLeg + m_swFoot) + (dtheta.*cos(theta).*((L_thigh*m_swLeg)/2 + L_thigh*m_swFoot + m_swFoot*r))/(m_M + m_swLeg + m_swFoot);

dc.ss_traj.dx_sw = dx_M + dr.*cos(theta) - dtheta.*sin(theta).*(L_thigh + r);
dc.ss_traj.dy_sw = dy_M + dr.*sin(theta) + dtheta.*cos(theta).*(L_thigh + r);
end