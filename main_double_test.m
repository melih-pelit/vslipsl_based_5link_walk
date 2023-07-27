%% five_link_walking_sim
tic
clear all
close all

%%
load("disturbance_test_results\disturbance_test_result_2023-07-20-00-51.mat")
dist_test.batch_size = 8;
dist_test.force_step_size = 20;
x_direction = false;

%%

for i = 1:4:length(dist_test.F_x_starts)
    
    F_start = [dist_test.F_x_starts(i), 0];

    plus_direction = true;
    dist_test.results = [dist_test.results; force_test( ...
        x_direction, ...
        plus_direction, ...
        F_start, ...
        dist_test.batch_size, ...
        dist_test.force_step_size, ...
        dist_test.enable_VSLIPSL_in_controller, ...
        dist_test.time)];

    plus_direction = false;
    dist_test.results = [dist_test.results; force_test( ...
        x_direction, ...
        plus_direction, ...
        F_start, ...
        dist_test.batch_size, ...
        dist_test.force_step_size, ...
        dist_test.enable_VSLIPSL_in_controller, ...
        dist_test.time)];

    save_disturbance_test_result(dist_test)
end

toc

%%

tic
clear all
close all
%%

load("disturbance_test_results\disturbance_test_result_2023-07-20-01-23.mat")
dist_test.batch_size = 8;
dist_test.force_step_size = 20;
x_direction = false;

dist_test.F_x_starts = -60:15:170;
dist_test.date_time = datestr(now,'yyyy-mm-dd-HH-MM');
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
        dist_test.enable_VSLIPSL_in_controller, ...
        dist_test.time)];

    plus_direction = false;
    dist_test.results = [dist_test.results; force_test( ...
        x_direction, ...
        plus_direction, ...
        F_start, ...
        dist_test.batch_size, ...
        dist_test.force_step_size, ...
        dist_test.enable_VSLIPSL_in_controller, ...
        dist_test.time)];

    save_disturbance_test_result(dist_test)
end

toc
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
clear all
close all

dist_test_nom = load("disturbance_test_results\disturbance_test_result_2023-07-20-00-51.mat");
dist_test_vslipsl = load("disturbance_test_results\disturbance_test_result_2023-07-20-04-35.mat");

results = {dist_test_nom, dist_test_vslipsl};

for j = 1:length(results)

    figure()
    hold on
    for i = 1:length(results{j}.dist_test.results)
        if results{j}.dist_test.results(i,1)
            plot(results{j}.dist_test.results(i,2), results{j}.dist_test.results(i,3), 'k.', 'MarkerSize', 20);
        else
            % plot(dist_test.results(i,2), dist_test.results(i,3), 'ko', 'MarkerSize', 8);
        end
    end
    xlim([-100, 200])
    ylim([-1000,600])
    
end

%% Polygon visualization TODO
figure()
pgon = polyshape([-24 -24 12 12 0],[-440 260 0 -540 400])
plot(pgon)
%%
save_disturbance_test_result(dist_test)

%%
function save_disturbance_test_result(dist_test)
    subfolder = "disturbance_test_results";
    filename = sprintf('disturbance_test_result_%s.mat', dist_test.date_time);
    save(fullfile(subfolder,filename),'dist_test')
end

