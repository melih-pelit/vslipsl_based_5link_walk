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