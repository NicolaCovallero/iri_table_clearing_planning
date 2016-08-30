function createBarPlot( data, exp_number, neglect, experiment_name)
% create bar plots of the experiment indexed by "exp_number", 
% and create the bar plots of the experiment, if "neglect" is set to true
% the bar plot shows only the iterations for which it founded a plan.

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
data_ = [str2num(data{i,N_OBJECTS}), str2num(data{i,FILTERING_TIME}), str2num(data{i,SEG_TIME}), str2num(data{i,PREDICATES_TIME}), str2num(data{i,PLANNING_TIME}), str2num(data{i,IK_TIME}), str2num(data{i,ACTION_EXECUTION_TIME})];
for i=2:size(data,1)
      data_ = [data_; str2num(data{i,N_OBJECTS}), str2num(data{i,FILTERING_TIME}), str2num(data{i,SEG_TIME}), str2num(data{i,PREDICATES_TIME}), str2num(data{i,PLANNING_TIME}), str2num(data{i,IK_TIME}), str2num(data{i,ACTION_EXECUTION_TIME})];
end
%remove all the iterations in which is fault of the segmenation
data_tmp = data_;
if (neglect)
    disp(neglect)
    data_tmp  = []
    for i = 1:1:size(data_,1)
        if(data_(i,6) ~= 0)
            data_tmp =[data_tmp; data_(i,:)];
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
legend('Filterting','Segmentation','States generation','Planning','IK','Action execution')
set(gca,'YGrid','on')
title(experiment_name)

% get the x tick label
for i = 1:length(n_objs)
    str{i} = sprintf('%d - %d',i , n_objs(i)) ;
end

set(gca,'XTickLabel',str);
%set(gca, 'FontSize', 40)
data_ = data_(:,1:end-1);
figure()
grid on

bar(data_,'stacked')
xlabel('<Iteration - Number of segmented objects>','FontSize',15)%if you save with .eps if will see the right size
ylabel('Time [seconds]','FontSize',15)
legend('Filterting','Segmentation','States generation','Planning','IK')
set(gca,'XTickLabel',str);
set(gca,'YGrid','on')
title(experiment_name)

end

