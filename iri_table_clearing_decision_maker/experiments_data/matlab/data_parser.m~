% Nicola Covallero
% Parser for the data of experiments for the project: https://bitbucket.org/NicolaCov/iri_table_clearing_planning
% data parser: format v1.0

addpath(genpath('../'));
addpath(genpath('utils/'));

clear all;
init_exp_process;
experiments = {'3objects_exp5','3objects_exp5'};

orignal_plan = [];
plans= [];

for i = 1:1:length(experiments)
    [data{i}, orignal_plan{i}, plans{i}] = readExpData(experiments{1});
end

getStatistics(data);
