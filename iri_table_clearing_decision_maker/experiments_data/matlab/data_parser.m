% Nicola Covallero
% Parser for the data of experiments for the project: https://bitbucket.org/NicolaCov/iri_table_clearing_planning
% data parser: format v1.0

addpath(genpath('../'));
addpath(genpath('utils/'));

clear all;
close all
init_exp_process;
%experiments = {'3objects_exp1','3objects_exp2','3objects_exp5'};
experiments = {'simple3_1_newLib','simple3_1'};

orignal_plan = [];
plans= [];

for i = 1:1:length(experiments)
    [data{i}, orignal_plan{i}, plans{i}] = readExpData(experiments{i});
end

getStatistics(data);

createBarPlots(data,1,0)
createBarPlots(data,2,0)
