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

results_to_be_plotted = {dist_test_nom.dist_test.results, dist_test_vslipsl.dist_test.results};
print = true;
save_names = {"nominal", "vslipsl"};

for i = 1:length(results_to_be_plotted)
    % detect the edge points
    results = results_to_be_plotted(i);
    results = results{1};
    pass_idx = find(results(:,1)==1);
    pass_results = results(pass_idx,:);
    
    mins_x = [];
    maxs_x = [];
    
    mins_y = [];
    maxs_y = [];
    
    for j = 1:length(pass_results)
        % 
        curr_F_x = pass_results(j,2);
    
        if ismember(curr_F_x, maxs_x)
            continue
        end
    
        curr_idx = find(pass_results(:,2)==curr_F_x);
        curr_F_ys = pass_results(curr_idx,3);
    
        mins_x(end+1) = curr_F_x;
        maxs_x(end+1) = curr_F_x;
        
        mins_y(end+1) = min(curr_F_ys);
        maxs_y(end+1) = max(curr_F_ys);
    end
    
    polygon_x = [mins_x, fliplr(maxs_x)];
    polygon_y = [mins_y, fliplr(maxs_y)];
    
    figure()
    pgon = polyshape(polygon_x,polygon_y);
    plot(pgon)
    xlim([-100, 200])
    ylim([-1000, 800])
    xlabel("$F_x$ [N]", 'Interpreter','latex')
    ylabel("$F_y$ [N]", 'Interpreter','latex')

    if print
        addpath("C:\Matlab Workspace\altmany-export_fig-d8b9f4a")
        set(gcf, 'color', 'none');
        set(gca, 'color', 'none');
        export_fig("figures/dist_res_"+ save_names{i} + ".png", '-m3')
    end

end

%%
function save_disturbance_test_result(dist_test)
    subfolder = "disturbance_test_results";
    filename = sprintf('disturbance_test_result_%s.mat', dist_test.date_time);
    save(fullfile(subfolder,filename),'dist_test')
end

