function [data, original_plan, plans] = readExpData( experiment_name )
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here

fileID = fopen(strcat(experiment_name,'/data.txt'));
tline = fgets(fileID)
new_experiment = false;
if(strcmp(tline(1:5),'--exp'))
   new_experiment = true;
else
    str = sprintf('ERROR! the format is not correct: %s',experiment_name);
    error(str)
end
r = 1; % row counter
exit = false;
data = []

tline = fgets(fileID)
if(strcmp(tline(1:5),'plan:'))
    original_plan = readOriginalPlan(tline)
end

plans = []; % this is a cell array that contains a plan for each possible iteration, for each frame the plan could be different from the original one

while(~exit)
    tline = fgets(fileID)
    if (length(tline) >= 17)
        if (strcmp(tline(1:17),'-- Table Cleared!'))
            exit = true;
            break;
        end  
    end
    r = r + 1
    
    label = getNamespace(tline);
     if(strcmp(label,'data'))
         data = [data; readData(tline)];
     elseif (~strcmp(label,'labels'))
         plan_tmp = strsplit(tline(1:end))
          for i = 1:1:length(plan_tmp)
               plan_tmp{i} = plan_tmp{i}(2:end);
          end
          plan_tmp = plan_tmp(~cellfun('isempty',plan_tmp)) %remove empty elements     
          
          plans{length(plans) + 1} =  plan_tmp;
          
     end  
   
end


end

