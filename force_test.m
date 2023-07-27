function rec_pass = force_test(x_direction, plus_direction, F_start, batch_size, force_step_size, enable_VSLIPSL_in_controller, dist_time)

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
% [q0, link_lengths] = calculate_optimal_link_lengths(dc, param);

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

%%

load_system('model_5LinkWalking')
last_idx = 1;
if plus_direction
    sign = 1;
else
    sign = -1;
end


while true
    i_start = last_idx;
    i_end = last_idx + batch_size - 1;
    for i = i_start:i_end
        if x_direction
            F_x = F_start(1) + sign*(i-1)*force_step_size;
            F_y = F_start(2);
        else
            F_x = F_start(1);
            F_y = F_start(2) + sign*(i-1)*force_step_size;
        end

        rec_pass(i,:) = [NaN, F_x, F_y]; % [pass/fail, F_x, F_y, time_end]
    
        in(i) = Simulink.SimulationInput('model_5LinkWalking');
        in(i) = in(i).setVariable('f_dist', [1; F_x; F_y; dist_time(1); dist_time(2)]);
        in(i) = in(i).setVariable('enable_VSLIPSL_in_controller', enable_VSLIPSL_in_controller);
        
        % constant workspace variables
        in(i) = in(i).setVariable('Tf', Tf);
        in(i) = in(i).setVariable('sample_time', sample_time);
        in(i) = in(i).setVariable('Initial_state', Initial_state);
        in(i) = in(i).setVariable('init_flag', init_flag);
        in(i) = in(i).setVariable('gains', gains);
        in(i) = in(i).setVariable('gains_VSLIPSL', gains_VSLIPSL);
        in(i) = in(i).setVariable('param', param);
    
        % reference trajectories
        in(i) = in(i).setVariable('slipslTraj_ss', slipslTraj_ss);
        in(i) = in(i).setVariable('slipslTraj_ds', slipslTraj_ds);
        in(i) = in(i).setVariable('swFootTraj', swFootTraj);
    
        in(i) = in(i).setVariable('slipslParams', slipslParams);
        in(i) = in(i).setVariable('ref_star_ss', ref_star_ss);
        in(i) = in(i).setVariable('ref_star_ds', ref_star_ds); 
    end
    
    out(i_start:i_end) = parsim(in(i_start:i_end), ...
            'ShowSimulationManager', 'off', ...
            'TransferBaseWorkspaceVariables','on');
    
    %% Determine pass or fail
    for i = i_start:i_end
        x_ZMP_noImpact = calc_ZMP( ...
            out(i).time, ...
            out(i).simout, ...
            param, ...
            out(i).flag, ...
            out(i).inputTorque, ...
            params, ...
            sample_time, ...
            out(i).GRF_simulink);
        
        if out(i).time(end) == 15 && max(x_ZMP_noImpact(5/sample_time:end)) <= 0.3001 && min(x_ZMP_noImpact(5/sample_time:end)) >= -0.1501
            pass = 1;
        else
            pass = 0;
        end
        rec_pass(i,1) = pass;
        fprintf('F_x= %d, F_y = %d, pass= %d \n', rec_pass(i,2), rec_pass(i,3), pass);
    
        if ~pass
            rec_pass(isnan(rec_pass(:,1)),:) = [];  % remove NaN rows
            return
        end
    end

    last_idx = last_idx + batch_size ;
end

end  % force_test()