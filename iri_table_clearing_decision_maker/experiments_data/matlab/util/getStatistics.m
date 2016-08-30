function getStatistics( data )
% Print some usefull statistic
% Average length of the real plan
disp('------ STATISTICS -------------------------------')

labelsIndices

% AVERAGE LENGTH EXECUTED PLANS
% ---- LENGTH EXECUTED PLAN
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
str = sprintf('LENGTH EXECUTED PLAN:         Mean: %d  +/-    Std: %d',mean(length_plans),std(length_plans));
disp(str)
% ----------------------------------------------

%% ---- PERCEPTION TIME (FILTERING + SEGMENTATION)
perception_time = [];
for e = 1:1:length(data)
    for i = 1:1:size(data{e},1)
        perception_time = [perception_time; (str2num(data{e}{i,FILTERING_TIME}) + str2num(data{e}{i,SEG_TIME}))];
    end
end
str = sprintf('PERCEPTION TIME:                      Mean: %d  +/-    Std: %d',mean(perception_time),std(perception_time));
disp(str)
% ----------------------------------------------

%% ---- STATE GENERATION TIME (FILTERING + SEGMENTATION)
state_generation_time = [];
state_generation_time_ = []; % this is used to compute the general time for the planning susbsytem considreing only the good cases
for e = 1:1:length(data)
    for i = 1:1:size(data{e},1)
        if(str2num(data{e}{i,IK_TIME}) > 0)
            state_generation_time_ = [state_generation_time_ ; str2num(data{e}{i,PREDICATES_TIME}) ];
        end
        state_generation_time = [state_generation_time; str2num(data{e}{i,PREDICATES_TIME}) ];
    end
end
str = sprintf('STATE GENERATION TIME:       Mean: %d  +/-    Std: %d',mean(state_generation_time),std(state_generation_time));
disp(str)
% ----------------------------------------------

%% ---- PLANNING TIME PLANNING TIME
planning_time = [];
planning_time_ = []; % this is used to compute the general time for the planning susbsytem considreing only the good cases
for e = 1:1:length(data)
    for i = 1:1:size(data{e},1)
         if(str2num(data{e}{i,IK_TIME}) > 0)
            planning_time_ = [planning_time_ ; str2num(data{e}{i,PLANNING_TIME}) ];
        end
        planning_time = [planning_time; str2num(data{e}{i,PLANNING_TIME}) ];
    end
end
str = sprintf('PLANNING TIME:                           Mean: %d  +/-    Std: %d',mean(planning_time),std(planning_time));
disp(str)
% ----------------------------------------------

%% ---- IK TIME 
ik_time = [];
ik_time_ = [];
for e = 1:1:length(data)
    for i = 1:1:size(data{e},1)
        if(str2num(data{e}{i,IK_TIME}) > 0)
            ik_time_ = [ik_time_ ; str2num(data{e}{i,IK_TIME}) ];
        end
        ik_time = [ik_time ; str2num(data{e}{i,IK_TIME}) ]; % this one will be used in the whole calculus at the bototm of the file
    end
end
str = sprintf('IK TIME:                                           Mean: %d  +/-    Std: %d',mean(ik_time_ ),std(ik_time_ ));
disp(str)
% ----------------------------------------------

%% ---- ACTION EXECUTION TIME 
action_execution_time = [];
for e = 1:1:length(data)
    for i = 1:1:size(data{e},1)
        % consider only the times when it took more than 1 sec
        if(str2num(data{e}{i,ACTION_EXECUTION_TIME}) > 1)
            action_execution_time = [action_execution_time ; str2num(data{e}{i,ACTION_EXECUTION_TIME}) ];
        end
      end
end
str = sprintf('ACTION EXECUTION TIME:        Mean: %d  +/-    Std: %d',mean(action_execution_time),std(action_execution_time ));
disp(str)

%%
planning_subsytem_time = [state_generation_time + planning_time + ik_time];
disp('-------------------')
str = sprintf('PLANNING SUBSYSTEM TIME:        Mean: %d  +/-    Std: %d',mean(planning_subsytem_time),std(planning_subsytem_time ));
disp(str)
disp('The planning subsystem includes. state generation  + planning + inverse kinematic. This is calculated taking into account all the cases (wrong segmentation, so no plan and ik = 0 (this affects in the average!))')

planning_subsytem_time_ = [state_generation_time_ + planning_time_ + ik_time_];
disp('-------------------')
str = sprintf('PLANNING SUBSYSTEM TIME:        Mean: %d  +/-    Std: %d',mean(planning_subsytem_time_),std(planning_subsytem_time_ ));
disp(str)
disp('The planning subsystem includes. state generation  + planning + inverse kinematic  (only for the good cases, when a plan exists)')


disp('TODO: when we backtrack the time should be added to the time of the previous iteration for a fair computation of the mean time of the system')
planning_subsystem_time_fair = [];
for e = 1:1:length(data)
    for i = 1:1:size(data{e},1)
        planning_subsystem_time_fair;
         % if the action is not executed and the IK is comptued (there is a plan)
         if i > 1
             if(str2num(data{e}{i,ACTION_EXECUTION_TIME}) < 1 && str2num(data{e}{i,IK_TIME}) > 0 && str2num(data{e}{i,PREDICATES_TIME}) == str2num(data{e}{i,PREDICATES_TIME}) ) 
                planning_subsystem_time_fair(i-1) = planning_subsystem_time_fair(i-1)  + str2num(data{e}{i,PREDICATES_TIME}) + str2num(data{e}{i,IK_TIME});
             else
                 % if there exists a solution
                 if(str2num(data{e}{i,IK_TIME}) > 0)
                    planning_subsystem_time_fair = [planning_subsystem_time_fair; str2num(data{e}{i,PLANNING_TIME}) + str2num(data{e}{i,PREDICATES_TIME}) + str2num(data{e}{i,IK_TIME}) ];
                 end
             end
          else
             % if there exists a solution
             if(str2num(data{e}{i,IK_TIME}) > 0)
                planning_subsystem_time_fair = [planning_subsystem_time_fair; str2num(data{e}{i,PLANNING_TIME}) + str2num(data{e}{i,PREDICATES_TIME}) + str2num(data{e}{i,IK_TIME}) ];
             end
         end
    end
end
disp('-------------------')
str = sprintf('FAIR PLANNING SUBSYSTEM TIME:        Mean: %d  +/-    Std: %d',mean(planning_subsystem_time_fair),std(planning_subsystem_time_fair ));
disp(str)
disp('The planning subsystem includes. state generation  + planning + inverse kinematic  (only for the good cases, when a plan exists and the backtracking is neglected)')

end


