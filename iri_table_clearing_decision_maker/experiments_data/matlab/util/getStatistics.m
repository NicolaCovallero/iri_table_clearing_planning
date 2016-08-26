function getStatistics( data )
% Print some usefull statistic
% Average length of the real plan
disp('------ STATISTICS -----------')
% AVERAGE LENGTH EXECUTED PLANS
% ---- LENGTH EXECUTED PLAN
labelsIndices
length_plans =[];
for e = 1:1:length(data)
    length_plan = 0;
    for i=1:1:size(data{e},1)
        if (~strcmp(data{e}{i,ACTION},'no_plan') && str2num(data{e}{i,ACTION_IK_FEASIBLE}))
            length_plan = length_plan + 1;
        end
    end
    length_plans = [length_plans length_plan];
end
str = sprintf('LENGTH EXECUTED PLAN:     Mean: %d  +/-    Std: %d',mean(length_plans),std(length_plans));
disp(str)

%----------------------------------------------


end

