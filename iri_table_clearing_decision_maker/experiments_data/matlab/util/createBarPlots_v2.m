function createBarPlots_v2( data, exp_number, neglect, experiment_name)
% create bar plots of the experiment indexed by "exp_number", 
% and create the bar plots of the experiment, if "neglect" is set to true
% This version shows 2 bar plots:
% 1) The time are divided in: perception module, planning subsystem,
% execution system (see the paper)
% 2) The different stages of the planning system are shown

data = data{exp_number};
labelsIndices
% fix the data, for the case the IK is not feasible, we set to 0 all the
% time of the filtreing and this stuff, the code did not take this into
% account
for i = 1:1:size(data,1) -1 
    if(strcmp(data{i,ACTION_IK_FEASIBLE},'0'))
        data{i+1,FILTERING_TIME} = '0';
        data{i+1,SEG_TIME} = '0';
        data{i+1,PREDICATES_TIME} = '0';
        %data{i+1,PLANNING_TIME} = '0';
        data{i+1,ON_PREDICATES_TIME} = '0';
        data{i+1,BLOCK_PREDICATES_TIME} = '0';
        data{i+1,BLOCK_GRASP_PREDICATES_TIME} = '0';
        data{i+1,OBJECT_COLLISIONS_TIME} = '0';
        data{i+1,EE_COLLISIONS_TIME} = '0';
        data{i+1,AVERAGE_OBJECTS_COLLISION_TIME} = '0';
        data{i+1,AVERAGE_EE_COLLISIONS_TIME} = '0';
    end
end

%%
i = 1;
data_1 = [str2num(data{i,N_OBJECTS}), (str2num(data{i,FILTERING_TIME}) + str2num(data{i,SEG_TIME})), (str2num(data{i,PREDICATES_TIME}) + str2num(data{i,PLANNING_TIME}) + str2num(data{i,IK_TIME})), str2num(data{i,ACTION_EXECUTION_TIME})];
for i=2:size(data,1)
      data_1 = [data_1; str2num(data{i,N_OBJECTS}), (str2num(data{i,FILTERING_TIME}) + str2num(data{i,SEG_TIME})), (str2num(data{i,PREDICATES_TIME}) + str2num(data{i,PLANNING_TIME}) + str2num(data{i,IK_TIME})), str2num(data{i,ACTION_EXECUTION_TIME})];
end


%remove all the iterations in which is fault of the segmenation
data_tmp = data_1;
if (neglect)
    disp(neglect)
    data_tmp  = []
    for i = 1:1:size(data_1,1)
        if(data_1(i,4) ~= 0)
            data_tmp =[data_tmp; data_1(i,:)];
        end
    end
end
n_objs = data_tmp(:,1)
data_ = data_tmp(:,2:end)
figure()
grid on
bar(data_,'stacked')
xlabel('<Iteration - Number of segmented objects>','FontSize',15)%if you save with .eps if will see the right size
ylabel('Time [seconds]','FontSize',15)
legend('Perception','Planning','Execution')
set(gca,'YGrid','on')
title(experiment_name)

% get the x tick label
for i = 1:length(n_objs)
    str{i} = sprintf('%d - %d',i , n_objs(i)) ;
end

set(gca,'XTickLabel',str);
%set(gca, 'FontSize', 40)
i = 1;
data_2 = [str2num(data{i,N_OBJECTS}),  str2num(data{i,PREDICATES_TIME}) , str2num(data{i,PLANNING_TIME}) , str2num(data{i,IK_TIME})];
for i=2:size(data,1)
      data_2 = [data_2; str2num(data{i,N_OBJECTS}),  str2num(data{i,PREDICATES_TIME}) , str2num(data{i,PLANNING_TIME}) , str2num(data{i,IK_TIME})];
end
%remove all the iterations in which is fault of the segmenation
data_tmp = data_2;
if (neglect)
    disp(neglect)
    data_tmp  = []
    for i = 1:1:size(data_2,1)
        if(data_1(i,4) ~= 0)
            data_tmp =[data_tmp; data_2(i,:)];
        end
    end
end

figure()
grid on

bar(data_tmp(:,2:end),'stacked')
xlabel('<Iteration - Number of segmented objects>','FontSize',15)%if you save with .eps if will see the right size
ylabel('Time [seconds]','FontSize',15)
set(gca,'YGrid','on')
title(strcat(experiment_name,'  Planning Subsystem Elapsed Times'))
% 
% P=findobj(gca,'type','patch');
% C=['b','r','g']; % make a colors list 
% for n=1:length(P) 
% set(P(n),'facecolor',C(n));
% end
legend('State Generation','Planning','IK')


end

