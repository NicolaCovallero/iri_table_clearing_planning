function elapsed_times = getStatistics( data )
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
    perception_time_ = 0;
    for i = 1:1:size(data{e},1)
        if (~strcmp(data{e}{i,ACTION},'no_plan') )
            %perception_time = [perception_time; (str2num(data{e}{i,FILTERING_TIME}) + str2num(data{e}{i,SEG_TIME}))];
            perception_time_ = perception_time_ +  (str2num(data{e}{i,FILTERING_TIME}) + str2num(data{e}{i,SEG_TIME}));
        end
    end
    perception_time = [perception_time perception_time_ ];
end
str = sprintf('PERCEPTION TIME(Filtering + segmentation):                      Mean: %d  +/-    Std: %d',mean(perception_time),std(perception_time));
disp(str)
% ----------------------------------------------

%% ---- 'PERCEPTION SUBSYSTEM  TIME (FILTERING + SEGMENTATION + STATE)
perception_time = [];
for e = 1:1:length(data)
    perception_time_ = 0;
    for i = 1:1:size(data{e},1)
        if (~strcmp(data{e}{i,ACTION},'no_plan') )
            perception_time_ = perception_time_ +  (str2num(data{e}{i,FILTERING_TIME}) + str2num(data{e}{i,SEG_TIME}) + str2num(data{e}{i,PREDICATES_TIME}));
        end
    end
    perception_time = [perception_time perception_time_ ];
end
str = sprintf('PERCEPTION SUBSYSTEM TIME (Filter+seg+state gen):       Mean: %d  +/-    Std: %d',mean(perception_time),std(perception_time));
disp(str)
% ----------------------------------------------

%% ---- PLANNING TIME PLANNING TIME
% planning_time = [];
% planning_time_ = []; % this is used to compute the general time for the planning susbsytem considreing only the good cases
% for e = 1:1:length(data)
%     for i = 1:1:size(data{e},1)
%          if(str2num(data{e}{i,IK_TIME}) > 0)
%             planning_time_ = [planning_time_ ; str2num(data{e}{i,PLANNING_TIME}) ];
%         end
%         planning_time = [planning_time; str2num(data{e}{i,PLANNING_TIME}) ];
%     end
% end
% str = sprintf('PLANNING TIME:                           Mean: %d  +/-    Std: %d',mean(planning_time),std(planning_time));
% disp(str)
% % ----------------------------------------------

%% ---- IK TIME 
% ik_time = [];
% ik_time_ = [];
% for e = 1:1:length(data)
%     for i = 1:1:size(data{e},1)
%         if(str2num(data{e}{i,IK_TIME}) > 0)
%             ik_time_ = [ik_time_ ; str2num(data{e}{i,IK_TIME}) ];
%         end
%         ik_time = [ik_time ; str2num(data{e}{i,IK_TIME}) ]; % this one will be used in the whole calculus at the bototm of the file
%     end
% end
% str = sprintf('IK TIME:                                           Mean: %d  +/-    Std: %d',mean(ik_time_ ),std(ik_time_ ));
% disp(str)
% % ----------------------------------------------

%% ---- ACTION EXECUTION TIME 
action_execution_time = [];
for e = 1:1:length(data)
    action_execution_time_ = 0;
    for i = 1:1:size(data{e},1)
        % consider only the times when it took more than 1 sec
        if(str2num(data{e}{i,ACTION_EXECUTION_TIME}) > 1)
            %action_execution_time = [action_execution_time ; str2num(data{e}{i,ACTION_EXECUTION_TIME}) ];
            action_execution_time_ = action_execution_time_ + str2num(data{e}{i,ACTION_EXECUTION_TIME}) ;
        end
    end
    action_execution_time = [action_execution_time action_execution_time_];
end
str = sprintf('ACTION EXECUTION TIME:        Mean: %d  +/-    Std: %d',mean(action_execution_time),std(action_execution_time ));
disp(str)

%%
% planning_subsytem_time = [state_generation_time + planning_time + ik_time];
% disp('-------------------')
% str = sprintf('PLANNING SUBSYSTEM TIME:        Mean: %d  +/-    Std: %d',mean(planning_subsytem_time),std(planning_subsytem_time ));
% disp(str)
% disp('The planning subsystem includes. state generation  + planning + inverse kinematic. This is calculated taking into account all the cases (wrong segmentation, so no plan and ik = 0 (this affects in the average!))')
% 
% planning_subsytem_time_ = [state_generation_time_ + planning_time_ + ik_time_];
% disp('-------------------')
% str = sprintf('PLANNING SUBSYSTEM TIME:        Mean: %d  +/-    Std: %d',mean(planning_subsytem_time_),std(planning_subsytem_time_ ));
% disp(str)
% disp('The planning subsystem includes. state generation  + planning + inverse kinematic  (only for the good cases, when a plan exists)')
% disp('Time to obtain an action and evaluate  ')

% planning_subsystem_time_fair = [];
% for e = 1:1:length(data)
%     for i = 1:1:size(data{e},1)
%         planning_subsystem_time_fair;
%          % if the action is not executed and the IK is comptued (there is a plan)
%          if i > 1
%              if(str2num(data{e}{i,ACTION_EXECUTION_TIME}) < 1 && str2num(data{e}{i,IK_TIME}) > 0 && str2num(data{e}{i,PREDICATES_TIME}) == str2num(data{e}{i - 1,PREDICATES_TIME}) ) 
%                 planning_subsystem_time_fair(end-1) = planning_subsystem_time_fair(end-1)  + str2num(data{e}{i,PLANNING_TIME}) + str2num(data{e}{i,IK_TIME});
%              else
%                  % if there exists a solution
%                  if(str2num(data{e}{i,IK_TIME}) > 0)
%                     planning_subsystem_time_fair = [planning_subsystem_time_fair; str2num(data{e}{i,PLANNING_TIME}) + str2num(data{e}{i,PREDICATES_TIME}) + str2num(data{e}{i,IK_TIME}) ];
%                  end
%              end
%           else
%              % if there exists a solution
%              if(str2num(data{e}{i,IK_TIME}) > 0)
%                 planning_subsystem_time_fair = [planning_subsystem_time_fair; str2num(data{e}{i,PLANNING_TIME}) + str2num(data{e}{i,PREDICATES_TIME}) + str2num(data{e}{i,IK_TIME}) ];
%              end
%          end
%     end
% end
% disp('-------------------')
% str = sprintf('FAIR PLANNING SUBSYSTEM TIME:        Mean: %d  +/-    Std: %d',mean(planning_subsystem_time_fair),std(planning_subsystem_time_fair ));
% disp(str)
% disp('The planning subsystem includes. state generation  + planning + inverse kinematic  (only for the good cases, when a plan exists and the backtracking is considered)')
% disp('Here the backtracking is considered. This time refers from when we have as input the segmented objects and the output a feasible action')

planning_subsystem_time_fairs = [];
for e = 1:1:length(data)
    planning_subsystem_time_fair = [];
    for i = 1:1:size(data{e},1)
         % if the action is not executed and the IK is comptued (there is a plan)
         if size(planning_subsystem_time_fair,1) > 1
             if(str2num(data{e}{i,ACTION_EXECUTION_TIME}) < 1 && str2num(data{e}{i,IK_TIME}) > 0 && str2num(data{e}{i,PREDICATES_TIME}) == str2num(data{e}{i-1,PREDICATES_TIME}) ) %if backtrack is done
                  i
                  planning_subsystem_time_fair(end-1)
                 planning_subsystem_time_fair(end-1) = planning_subsystem_time_fair(end-1)  + str2num(data{e}{i,PLANNING_TIME}) + str2num(data{e}{i,IK_TIME});
             else
                 % if there exists a solution
                 if(str2num(data{e}{i,IK_TIME}) > 0)
                    planning_subsystem_time_fair = [planning_subsystem_time_fair; str2num(data{e}{i,PLANNING_TIME})  + str2num(data{e}{i,IK_TIME}) ];
                 end
             end
          else
             % if there exists a solution
             if(str2num(data{e}{i,IK_TIME}) > 0)
                planning_subsystem_time_fair = [planning_subsystem_time_fair; str2num(data{e}{i,PLANNING_TIME}) + str2num(data{e}{i,IK_TIME}) ];
             end
         end
    end
    planning_subsystem_time_fairs = [planning_subsystem_time_fairs sum(planning_subsystem_time_fair)];
end
disp('-------------------')
str = sprintf('FAIR PLANNING SUBSYSTEM TIME(No state generation):        Mean: %d  +/-    Std: %d',mean(planning_subsystem_time_fairs),std(planning_subsystem_time_fairs ));
disp(str)
disp('The planning subsystem includes.: planning + inverse kinematic  (only for the good cases, when a plan exists and the backtracking is considered)')
disp('Here the backtracking is considered. This time refers from when we have as input the state and the output a feasible action')

%% TOTAL TIME (Average from when the point cloud is captured until a feasible decision (an action that can be executed) is done)
total_times =[];
for e = 1:1:length(data)
    total_time = [];
    for i = 1:1:size(data{e},1)
         % if the action is not executed and the IK is comptued (there is a plan)
         if size(total_time,1) > 1
             if(str2num(data{e}{i,ACTION_EXECUTION_TIME}) < 1 && str2num(data{e}{i,IK_TIME}) > 0 && str2num(data{e}{i,PREDICATES_TIME}) == str2num(data{e}{i-1,PREDICATES_TIME}) ) %if backtrack is done
                total_time(end-1) = total_time(end-1)  + str2num(data{e}{i,PLANNING_TIME}) + str2num(data{e}{i,IK_TIME});
             else
                 % if there exists a solution
                 if(str2num(data{e}{i,IK_TIME}) > 0)
                    total_time = [total_time; str2num(data{e}{i,ACTION_EXECUTION_TIME})+str2num(data{e}{i,SEG_TIME})+str2num(data{e}{i,FILTERING_TIME})+  str2num(data{e}{i,PREDICATES_TIME}) + str2num(data{e}{i,PLANNING_TIME})  + str2num(data{e}{i,IK_TIME}) ];
                 end
             end
          else
             % if there exists a solution
             if(str2num(data{e}{i,IK_TIME}) > 0)
                total_time = [total_time; str2num(data{e}{i,ACTION_EXECUTION_TIME})+str2num(data{e}{i,SEG_TIME})+str2num(data{e}{i,FILTERING_TIME})+str2num(data{e}{i,PREDICATES_TIME}) + str2num(data{e}{i,PLANNING_TIME}) + str2num(data{e}{i,IK_TIME}) ];
             end
         end
    end
    total_times = [total_times sum(total_time)];
end
disp('-------------------')
disp(total_times)
str = sprintf('TOTAL:        Mean: %d  +/-    Std: %d',mean(total_times),std(total_times ));
disp(str)
disp('TOTAL is the time to execute an action, from when a point clous is received.')
disp(' So this takes into account filtering, segmentation, state generation, planning, IK, eventual backtrackings and action execution. This is done considering only the cases in which a plan exists.')

elapsed_times{1} = [length_plans];
elapsed_times{2} = [perception_time];
elapsed_times{3} = [planning_subsystem_time_fairs];
elapsed_times{4} = [action_execution_time];
elapsed_times{5} = [total_times];

end


