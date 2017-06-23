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
length(data)
for e = 1:1:length(data)
    perception_time_ = 0;
    for i = 1:1:size(data{e},1)
        if (~strcmp(data{e}{i,ACTION},'no_plan') ) % consider only the cases the segmentation was fine and lead to a plan
            if perception_time_ > 0
                %i
                if(~((str2num(data{e}{i - 1,ACTION_EXECUTION_TIME}) < 1 && str2num(data{e}{i,IK_TIME}) > 0 && str2num(data{e}{i,PREDICATES_TIME}) == str2num(data{e}{i-1,PREDICATES_TIME})))) %do not consider backtracking (when backtracked is done in the file the eprception times are saved)
                    if(str2num(data{e}{i,IK_TIME}) > 0) % consider only the feasible plan
                        perception_time_ = perception_time_ +  (str2num(data{e}{i,FILTERING_TIME}) + str2num(data{e}{i,SEG_TIME}) + str2num(data{e}{i,PREDICATES_TIME}));                
                    end
                else
                    disp('backtrack')
                end
            else
                if(str2num(data{e}{i,IK_TIME}) > 0)
                perception_time_ = perception_time_ +  (str2num(data{e}{i,FILTERING_TIME}) + str2num(data{e}{i,SEG_TIME}) + str2num(data{e}{i,PREDICATES_TIME}));
                end
                end
        end
    end
    perception_time = [perception_time perception_time_ ];
end
disp('perception_time')
disp(perception_time)

str = sprintf('PERCEPTION SUBSYSTEM TIME (Filter+seg+state gen):       Mean: %d  +/-    Std: %d',mean(perception_time),std(perception_time));
disp(str)
% ----------------------------------------------
%% ---- 'STATE GENERATION TIME (FILTERING + SEGMENTATION + STATE)
state_generation_times = [];
for e = 1:1:length(data)
    state_generation_time = 0;
    for i = 1:1:size(data{e},1)
        if (~strcmp(data{e}{i,ACTION},'no_plan') ) % consider only the cases the segmentation was fine and lead to a plan
            if state_generation_time > 0
                %i
                if(~((str2num(data{e}{i - 1,ACTION_EXECUTION_TIME}) < 1 && str2num(data{e}{i,IK_TIME}) > 0 && str2num(data{e}{i,PREDICATES_TIME}) == str2num(data{e}{i-1,PREDICATES_TIME})))) %do not consider backtracking (when backtracked is done in the file the eprception times are saved)
                    if(str2num(data{e}{i,IK_TIME}) > 0)
                        state_generation_time = state_generation_time +  (str2num(data{e}{i,FILTERING_TIME}) + str2num(data{e}{i,SEG_TIME}) + str2num(data{e}{i,PREDICATES_TIME}));
                    end
                else
                    %disp('backtrack')
                end
            else
                if(str2num(data{e}{i,IK_TIME}) > 0)
                    state_generation_time = state_generation_time +  (str2num(data{e}{i,FILTERING_TIME}) + str2num(data{e}{i,SEG_TIME}) + str2num(data{e}{i,PREDICATES_TIME}));
                end
            end
        end
    end
    state_generation_times = [perception_time perception_time_ ];
end
% disp(perception_time)

str = sprintf('STATE GENERATION:       Mean: %d  +/-    Std: %d',mean(state_generation_times),std(state_generation_times));
disp(str)
% ----------------------------------------------

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
% disp(action_execution_time)

str = sprintf('ACTION EXECUTION TIME:        Mean: %d  +/-    Std: %d',mean(action_execution_time),std(action_execution_time ));
disp(str)
%% PLANNING TIME
planning_subsystem_time_fairs = [];
for e = 1:1:length(data)
    planning_subsystem_time_fair = [];
    for i = 1:1:size(data{e},1)
         % if the action is not executed and the IK is comptued (there is a plan)
         if size(planning_subsystem_time_fair,1) > 1
             if(str2num(data{e}{i - 1,ACTION_EXECUTION_TIME}) < 1 && str2num(data{e}{i,IK_TIME}) > 0 && str2num(data{e}{i,PREDICATES_TIME}) == str2num(data{e}{i-1,PREDICATES_TIME})) %if backtrack is done
                  i;
                  planning_subsystem_time_fair(end);
                 planning_subsystem_time_fair(end) = planning_subsystem_time_fair(end)  + str2num(data{e}{i,PLANNING_TIME}) + str2num(data{e}{i,IK_TIME});
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
% disp(planning_subsystem_time_fairs)
str = sprintf('FAIR PLANNING SUBSYSTEM TIME(No state generation):        Mean: %d  +/-    Std: %d',mean(planning_subsystem_time_fairs),std(planning_subsystem_time_fairs ));
disp(str)
disp('The planning subsystem includes.: planning + inverse kinematic  (only for the good cases, when a plan exists and the backtracking is considered)')
disp('Here the backtracking is considered. This time refers from when we have as input the state and the output a feasible action')

%% decision time
decision_time_ = [];
n_backtracks = [];
for e = 1:1:length(data)
%     disp('data:')
%     data{e}
    n_backtracking = 0;
    decision_time = [];
    for i = 1:1:size(data{e},1) % for the "e" experiment
         % if the action is not executed and the IK is comptued (there is a plan)
         if length(decision_time) > 1
             if(str2num(data{e}{i - 1,ACTION_EXECUTION_TIME}) < 1 && str2num(data{e}{i,IK_TIME}) > 0 && str2num(data{e}{i,PREDICATES_TIME}) == str2num(data{e}{i-1,PREDICATES_TIME})) %if backtrack is done
                 disp('bactracking detected')
                 n_backtracking = n_backtracking + 1;
                 decision_time(end) = decision_time(end)  + str2num(data{e}{i,PLANNING_TIME}) + str2num(data{e}{i,IK_TIME});
             else
                 % if there exists a solution
                 if(str2num(data{e}{i,IK_TIME}) > 0)
                    decision_time = [decision_time, str2num(data{e}{i,FILTERING_TIME}) + str2num(data{e}{i,SEG_TIME})   + str2num(data{e}{i,PREDICATES_TIME}) + str2num(data{e}{i,PLANNING_TIME})  + str2num(data{e}{i,IK_TIME}) ];
                 end
             end
         else
              
             % if there exists a solution
             if(str2num(data{e}{i,IK_TIME}) > 0)
                    decision_time = [decision_time, str2num(data{e}{i,FILTERING_TIME}) + str2num(data{e}{i,SEG_TIME})   + str2num(data{e}{i,PREDICATES_TIME}) + str2num(data{e}{i,PLANNING_TIME})  + str2num(data{e}{i,IK_TIME}) ];
             end
             
         end
    end
    decision_time_ = [decision_time_ , decision_time];
    n_backtracks = [n_backtracks n_backtracking];
end
disp('-------------------')
%disp(decision_time_)
str = sprintf('DECISION TIME:        Mean: %d  +/-    Std: %d',mean(decision_time_),std(decision_time_ ));
disp(str)
str = sprintf('Detected %d backtracking', n_backtracking);
disp(str)


%% TOTAL TIME (Average from when the point cloud is captured until a feasible decision (an action that can be executed) is done)
total_times =[];
%     for e = 1:1:length(data)
%         total_time = [];
%         for i = 1:1:size(data{e},1)
%             if (~strcmp(data{e}{i,ACTION},'no_plan') )
%              % if the action is not executed and the IK is comptued (there is a plan)
%              if size(total_time,1) > 1
%                  if(str2num(data{e}{i - 1,ACTION_EXECUTION_TIME}) < 1 && str2num(data{e}{i,IK_TIME}) > 0 && str2num(data{e}{i,PREDICATES_TIME}) == str2num(data{e}{i-1,PREDICATES_TIME})) %if backtrack is done
%                     total_time(end) = total_time(end)  + str2num(data{e}{i,PLANNING_TIME}) + str2num(data{e}{i,IK_TIME});
%                  else
%                      % if there exists a solution
%                      if(str2num(data{e}{i,IK_TIME}) > 0)
%                         total_time = [total_time; str2num(data{e}{i,ACTION_EXECUTION_TIME})+str2num(data{e}{i,SEG_TIME})+str2num(data{e}{i,FILTERING_TIME})+  str2num(data{e}{i,PREDICATES_TIME}) + str2num(data{e}{i,PLANNING_TIME})  + str2num(data{e}{i,IK_TIME}) ];
%                      end
%                  end
%               else
%                  % if there exists a solution
%                  if(str2num(data{e}{i,IK_TIME}) > 0)
%                     total_time = [total_time; str2num(data{e}{i,ACTION_EXECUTION_TIME})+str2num(data{e}{i,SEG_TIME})+str2num(data{e}{i,FILTERING_TIME})+str2num(data{e}{i,PREDICATES_TIME}) + str2num(data{e}{i,PLANNING_TIME}) + str2num(data{e}{i,IK_TIME}) ];
%                  end
%              end
%             end
%         end
%         total_times = [total_times sum(total_time)];
%     end
total_times = [planning_subsystem_time_fairs + action_execution_time + perception_time];
disp('-------------------')
% disp(total_times)
str = sprintf('TOTAL:        Mean: %d  +/-    Std: %d',mean(total_times),std(total_times ));
disp(str)
disp('TOTAL is the time to execute an action, from when a point clous is received.')
disp(' So this takes into account filtering, segmentation, state generation, planning, IK, eventual backtrackings and action execution. This is done considering only the cases in which a plan exists.')

elapsed_times{1} = [length_plans];
elapsed_times{2} = [perception_time];
elapsed_times{3} = [planning_subsystem_time_fairs];
elapsed_times{4} = [action_execution_time];
elapsed_times{5} = [total_times];
elapsed_times{6} = [decision_time_];
elapsed_times{7} = [n_backtracks];

end


