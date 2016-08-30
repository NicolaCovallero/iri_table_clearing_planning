% Nicola Covallero
% Parser for the data of experiments for the project: https://bitbucket.org/NicolaCov/iri_table_clearing_planning
% data parser: format v1.0

addpath(genpath('../'));
addpath(genpath('utils/'));

clear all;
close all
init_exp_process;
%experiments = {'3objects_exp1','3objects_exp2','3objects_exp5'};
%experiments = {'7-29-9-44_simple3_2','7-29-10-18_simple3_1_real'};
experiments = {'exp3_1_real','exp3_2_real','exp3_3_real'}
orignal_plan = [];
plans= [];

for i = 1:1:length(experiments)
    [data{i}, orignal_plan{i}, plans{i}] = readExpData(experiments{i});
end


% createBarPlots(data,1,0,experiments{1})
% createBarPlots(data,2,1,experiments{2})
% createBarPlots(data,3,1,experiments{3})
% createBarPlots_v2(data,1,0,experiments{1})
% createBarPlots_v2(data,2,0,experiments{2})
% createBarPlots_v2(data,3,0,experiments{3})

getStatistics(data);
