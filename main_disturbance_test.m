%% five_link_walking_sim
tic
clear all
close all

%%
dist_test.date_time = datestr(now,'yyyy-mm-dd-HH-MM');

%%
dist_test.enable_VSLIPSL_in_controller = true;
%% Do a force test in a certain direction until the fail point has been found
F_start = [0; 0];
dist_test.batch_size = 8;
dist_test.force_step_size = 10;  % [N]

%%
x_direction = true;
plus_direction = true;
current_test_record_plus = force_test( ...
    x_direction, ...
    plus_direction, ...
    F_start, ...
    dist_test.batch_size, ...
    dist_test.force_step_size, ...
    dist_test.enable_VSLIPSL_in_controller);

%%
plus_direction = false;
current_test_record_minus = force_test( ...
    x_direction, ...
    plus_direction, ...
    F_start, ...
    dist_test.batch_size, ...
    dist_test.force_step_size, ...
    dist_test.enable_VSLIPSL_in_controller);

%%
dist_test.F_x_starts = current_test_record_minus(end-1,2):dist_test.force_step_size:current_test_record_plus(end-1,2);
dist_test.results = [];
x_direction = false;

%%

for i = 1:length(dist_test.F_x_starts)
    
    F_start = [dist_test.F_x_starts(i), 0];

    plus_direction = true;
    dist_test.results = [dist_test.results; force_test( ...
        x_direction, ...
        plus_direction, ...
        F_start, ...
        dist_test.batch_size, ...
        dist_test.force_step_size, ...
        dist_test.enable_VSLIPSL_in_controller)];

    plus_direction = false;
    dist_test.results = [dist_test.results; force_test( ...
        x_direction, ...
        plus_direction, ...
        F_start, ...
        dist_test.batch_size, ...
        dist_test.force_step_size, ...
        dist_test.enable_VSLIPSL_in_controller)];

    save_disturbance_test_result(dist_test)
end
%% plotting
figure()
hold on
for i = 1:length(dist_test.results)
    if dist_test.results(i,1)
        plot(dist_test.results(i,2), dist_test.results(i,3), 'k.', 'MarkerSize', 20);
    else
        plot(dist_test.results(i,2), dist_test.results(i,3), 'ko', 'MarkerSize', 8);
    end
end
%%
save_disturbance_test_result(dist_test)
%%
function save_disturbance_test_result(dist_test)
    subfolder = "disturbance_test_results";
    filename = sprintf('disturbance_test_result_%s.mat', dist_test.date_time);
    save(fullfile(subfolder,filename),'dist_test')
end

