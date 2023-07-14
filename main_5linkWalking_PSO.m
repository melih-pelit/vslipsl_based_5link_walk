%% five_link_walking_sim
tic
clear all
close all

%% SLIP Data

% 2021.09.01: new SLIP-SL trajectory with low CoT (CoT = 0.1907)
load('SLIPSL_Data\dc_comp2021-09-01-16-06') % SLIPSLOpenOCL2021-09-01-15-36 (@C:\Matlab Workspace\SLIPSL_OpenOCL\202108111747 - SLIPSL_OpenOCL (True CoM))

dc = calc_slipsl_traj(dc); % function to calculate CoM traj. at SS phase (appends to dc)

slipsl.time_ss = dc.time_ss';
slipsl.x_ss = dc.ss_traj.x_CoM_ss;
slipsl.y_ss = dc.ss_traj.y_CoM_ss;
slipsl.dx_ss = dc.ss_traj.dx_CoM_ss;
slipsl.dy_ss = dc.ss_traj.dy_CoM_ss;

slipsl.time_ds = dc.time_ds';
slipsl.x_ds = dc.simout_ds(1,:)';
slipsl.y_ds = dc.simout_ds(2,:)';
slipsl.dx_ds = dc.simout_ds(3,:)';
slipsl.dy_ds = dc.simout_ds(4,:)';

swFootSLIPSL.x_CoM = slipsl.x_ss;
swFootSLIPSL.x = dc.ss_traj.x_sw;
swFootSLIPSL.z = dc.ss_traj.y_sw;
swFootSLIPSL.dx = dc.ss_traj.dx_sw;
swFootSLIPSL.dz = dc.ss_traj.dy_sw;

slipslParams = [
    dc.const_param.m_M; dc.const_param.m_swLeg; dc.const_param.m_swFoot; dc.const_param.L_thigh; dc.const_param.I_swLeg; dc.const_param.I_swFoot; dc.const_param.gravi;
    dc.col_param.k0_ss; dc.col_param.L0_ss; dc.col_param.k0_ds; dc.col_param.L0_ds;
    dc.col_param.k_swFoot; dc.col_param.k_swLeg; dc.col_param.theta0; dc.col_param.r0; dc.col_param.foot; dc.col_param.footPlus];

% 2021.09.01: new SLIP-SL trajectory with low CoT (CoT = 0.1907)
load('SLIPSL_Data\ref_star2021-09-03-14-20') % dc_comp2021-09-01-16-06

ref_star_ss = [ref_star.ss.x_M_star, ...
    ref_star.ss.y_M_star, ref_star.ss.x_swFoot_star, ref_star.ss.y_swFoot_star, ...
    ref_star.ss.del_y_M_star, ref_star.ss.del_x_swFoot_star, ref_star.ss.del_y_swFoot_star, ...
    ref_star.ss.del2_y_M_star, ref_star.ss.del2_x_swFoot_star, ref_star.ss.del2_y_swFoot_star, ...
    ref_star.ss.x_CoM_star];

ref_star_ds = [ref_star.ds.x_CoM_star, ...
    ref_star.ds.y_CoM_star, ref_star.ds.dx_CoM_star, ...
    ref_star.ds.del_y_CoM_star, ref_star.ds.del_dx_CoM_star, ...
    ref_star.ds.del2_y_CoM_star];

%%
sample_time = 0.001;

%% Model parameters

params.m1 = 4.75;
params.m2 = 5.25;

params.m5 = 60; % 5,1,1
params.m3 = params.m2;
params.m4 = params.m1;

params.l1 = 0.55;
params.l2 = 0.5;
params.l3 = params.l2;
params.l4 = params.l1;
params.l5 = 0.3;

params.g = 9.81;

params.I1 = params.m1*params.l1^2/12;
params.I2 = params.m2*params.l2^2/12;
params.I3 = params.I1;
params.I4 = params.I2;
params.I5 = params.m5*params.l5^2/12;

param = [params.m1; params.m2; params.m5; params.l1; params.l2; params.l5; params.g; params.I1; params.I2; params.I5];

%% Getting the ground reaction forces equation

slipslTraj_ss = [ref_star.dc.ss.time, ref_star.dc.ss.x_CoM', ref_star.dc.ss.y_CoM', ref_star.dc.ss.dx_CoM', ref_star.dc.ss.dy_CoM'];
slipslTraj_ds = [ref_star.dc.ds.time - ref_star.dc.ds.time(1), ref_star.dc.ds.x_CoM, ref_star.dc.ds.y_CoM, ref_star.dc.ds.dx_CoM, ref_star.dc.ds.dy_CoM];

swFootTraj = [ref_star.dc.ss.x_CoM', ref_star.dc.ss.x_sw', ref_star.dc.ss.y_sw', ref_star.dc.ss.dx_sw', ref_star.dc.ss.dy_sw']; % swing foot reference trajectory

%% Calculate Link Lengths
[q0, link_lengths] = calculate_optimal_link_lengths(dc, param)

%% Initial conditions
N = 1; % starting collocation point
% N = dc.N_ss + 1;
x_CoM_des = slipsl.x_ss(N);
z_CoM_des = slipsl.y_ss(N);
x_sw_des = swFootSLIPSL.x(N);
z_sw_des = swFootSLIPSL.z(N);
dx_CoM_des = slipsl.dx_ss(N);
dz_CoM_des = slipsl.dy_ss(N);
des_dx_sw = swFootSLIPSL.dx(N);
des_dz_sw = swFootSLIPSL.dz(N);
% [q0, dq0] = calculate_init_pos(x_CoM_des, z_CoM_des, x_sw_des, z_sw_des, dx_CoM_des, dz_CoM_des, des_dx_sw, des_dz_sw, param);
[q0, dq0] = calculate_init_pos_v2(x_CoM_des, z_CoM_des, x_sw_des, z_sw_des, dx_CoM_des, dz_CoM_des, des_dx_sw, des_dz_sw, param);
clear x_CoM_des z_CoM_des x_sw_des z_sw_des dx_CoM_des dz_CoM_des des_dx_sw des_dz_sw

X = [q0; dq0];

Initial_state = [q0;dq0];

%% init flag
close
% CoM = calculate_com(X, param, foot, ddq);
CoM = calculate_com(X, param, 0, [0;0;0;0;0]);
init_flag = [1; 0; -dc.col_param.footPlus; 0; CoM(1); CoM(2)];
clear CoM

%% VSLIPSL Controller gains and ref trajectories
% Controller Gains
gain_VSLIPSL.K_p = 200;
gain_VSLIPSL.K_d = 40;

gain_VSLIPSL.K_p_sw = 200;
gain_VSLIPSL.K_d_sw = 40;

gain_VSLIPSL.K_p_ds = 200;
gain_VSLIPSL.K_d_ds = 40;
gain_VSLIPSL.K_v_ds = 40; % 5 in visser 2012

gains_VSLIPSL = [gain_VSLIPSL.K_p; gain_VSLIPSL.K_d; gain_VSLIPSL.K_p_sw; gain_VSLIPSL.K_d_sw; gain_VSLIPSL.K_p_ds; gain_VSLIPSL.K_d_ds; gain_VSLIPSL.K_v_ds];

%%
Tf = 15; % simulation finish time [secs]

ptc = [835, 370, 444, 550, 375, 100, 60, 110, 310, 140];

gain.kP1 = ptc(1); % gain for total energy controller
gain.kP2 = ptc(2); % gain for total energy controller
gain.kP3 = ptc(3); % gain for total energy controller
gain.kP4 = ptc(4); % gain for total energy controller
gain.kP5 = ptc(5); % gain for total energy controller

gain.kD1 = ptc(6); % gain for total energy controller
gain.kD2 = ptc(7); % gain for total energy controller
gain.kD3 = ptc(8); % gain for total energy controller
gain.kD4 = ptc(9); % gain for total energy controller
gain.kD5 = ptc(10); % gain for total energy controller

gains = [gain.kP1, gain.kD1, gain.kP2, gain.kD2, gain.kP3, gain.kD3, gain.kP4, gain.kD4, gain.kP5, gain.kD5]; % gains vector

% % F_dist_x = 1:1:1000;
% F_dist_x = 10;
% % F_dist_y = 0;
%
% for i = 1:length(F_dist_x)
%     F_dist_y = 0;
%     % f_dist = [1; F_dist_x(i); F_dist_y]; % flag for activating or deactivating the dist forces and choosing their magnitudes [on/off; F_dist_x, F_dist_y]
%     f_dist = [1; F_dist_x(i); F_dist_y; 9.7; 9.8]; % [dist on/off, F_x_dist, F_y_dist, dist. start time [sec], dist. end time [sec]]
%
%     open_system('model_5LinkWalking')
%     sim('model_5LinkWalking')
%
%     x_ZMP_noImpact = calc_ZMP(time, simout, param, flag, inputTorque, params, sample_time, GRF_simulink);
%
%     if time(end) == 15 && max(x_ZMP_noImpact(5/sample_time:end)) <= 0.3001 && min(x_ZMP_noImpact(5/sample_time:end)) >= -0.1501
%         pass(i) = 1;
%     else
%         pass(i) = 0;
%     end
%     rec_pass(i,:) = [pass(i), F_dist_x(i), F_dist_y];
%
%     fprintf('F_x= %d, F_y = %d, pass= %d \n', F_dist_x(i), F_dist_y, pass(i));
%
%     if pass(i) == 0
%         display('FAIL')
%         break
%     end
% end

%%

% initial disturbance
F_x = 0;
F_y = 0;
i = 0;
k = 0; % counter for the number of lines to send
k_max = 360/30; % max number of lines, set the search degree theta (360/theta)(ex. theta = 180 searches for two lines)

n = 1; % counter for recording the results

figure()
hold on; grid on
xlabel('F_{x, dist.} [N]')
ylabel('F_{y, dist.} [N]')

while(1)
    
    f_dist = [1; F_x; F_y; 10.1; 10.2]; % [dist on/off, F_x_dist, F_y_dist, dist. start time [sec], dist. end time [sec]]
    
    open_system('model_5LinkWalking')
    sim('model_5LinkWalking')
    
    x_ZMP_noImpact = calc_ZMP(time, simout, param, flag, inputTorque, params, sample_time, GRF_simulink);
    
    if time(end) == 15 && max(x_ZMP_noImpact(5/sample_time:end)) <= 0.3001 && min(x_ZMP_noImpact(5/sample_time:end)) >= -0.1501
        pass = 1;
    else
        pass = 0;
    end
    rec_pass(n,:) = [pass, F_x, F_y];
    
    fprintf('F_x= %d, F_y = %d, pass= %d \n', F_x, F_y, pass);
    
    % Plotting the disturbance analysis 
    plot(F_x, F_y, 'k.', 'MarkerSize', 20)
    
    if pass==1
        % set the new disturbance force
        i = i + 1;
        F_x = cos((k*2*pi)/k_max)*i;
        F_y = sin((k*2*pi)/k_max)*i;
    else
        % if the search has found the fail point (pass == 0), on to the
        % next line
        k = k + 1;
        i = 1; % reset i
        F_x = cos((k*2*pi)/k_max)*i;
        F_y = sin((k*2*pi)/k_max)*i;
    end
    
    if k == k_max
        % search finished
        break
    end

    n = n + 1;
end

%% Record disturbance test results
f_record = 1;
if f_record == 1
    distResults.pass = rec_pass;
    distResults.f_dist; % [dist on/off, F_x_dist, F_y_dist, dist. start time [sec], dist. end time [sec]], forces should be ignored since they are just the last value
    filename = sprintf('distTestResults%s.mat', datestr(now,'yyyy-mm-dd-HH-MM'));
    subfolder = 'disturbance test results';
    save(fullfile(subfolder,filename),'distResults')
end

%% Stability Analysis
% figure_poincare(simout, param, flag(:,2), step_no, rt_VLO, time)

%% CoM acceleration comparison
%  trackingPlots_jointAngles(simout, flag, time, des_th)

%% Animation
f_animation = 0;
if f_animation == 1
    f_video = 1; % flag for recrding video
    f_pause = 1;
    frame_leap = 20;
    % animation(f_video, simout, sample_time, sw_ft_des, param, f_pause, frame_leap, flag, step_no, force, des_z_dz_dx)
    animation(f_video, simout, sample_time, sw_ft_des, param, f_pause, frame_leap, flag, step_no, force, des_traj, slipslParams, des_th)
end
%% Calculating ZMP

m_foot = 2;
x_foot_CoM = 0.1;

x_ZMP = (inputTorque(:,1) + x_foot_CoM*m_foot*params.g) ./ (GRF_simulink(:,2) + m_foot*params.g);

x_ZMP_noImpact = x_ZMP;
% removing the data at the impact
for i =1:length(time)
    if time(i)- flag(i,4) < sample_time && flag(i,1) == -1
        if x_ZMP_noImpact(i) > 0.3
            x_ZMP_noImpact(i) = 0.3;
        elseif x_ZMP_noImpact(i) < -0.15
            x_ZMP_noImpact(i) = -0.15;
        end
    end
end

figure()
% plot(time, medfiltX_ZMP)
plot(time, x_ZMP)
hold on
plot(time, x_ZMP_noImpact)

max(x_ZMP_noImpact)
min(x_ZMP_noImpact)

toc

%% encoder-decoder elapsed time

encoder_time_elapsed = encoder_time_elapsed(~isnan(encoder_time_elapsed));
mean_encoder__time = mean(encoder_time_elapsed);
max_encoder__time = max(encoder_time_elapsed);

mean_decoder_time = mean(decoder_time_elapsed);
max_decoder_time_elapsed = max(decoder_time_elapsed);


%%

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

function x_ZMP_noImpact = calc_ZMP(time, simout, param, flag, inputTorque, params, sample_time, GRF_simulink)

m_foot = 2;
x_foot_CoM = 0.1;

x_ZMP = (inputTorque(:,1) + x_foot_CoM*m_foot*params.g) ./ (GRF_simulink(:,2) + m_foot*params.g);

x_ZMP_noImpact = x_ZMP;
% removing the data at the impact
for i =1:length(time)
    if time(i)- flag(i,4) < 6*sample_time && flag(i,1) == -1
        if x_ZMP_noImpact(i) > 0.3
            x_ZMP_noImpact(i) = 0.3;
        elseif x_ZMP_noImpact(i) < -0.15
            x_ZMP_noImpact(i) = -0.15;
        end
    end
end

end