% Nicola Covallero
% Parser for the data of experiments for the project: https://bitbucket.org/NicolaCov/iri_table_clearing_planning
% data parser: format v1.0

addpath(genpath('../'));
addpath(genpath('utils/'));

clear all;
close all
init_exp_process;
experiments_{1} = {'exp3_1_real','exp3_2_real','exp3_3_real'}
experiments_{2} = {'exp4_1_real','exp4_2_real','exp4_4_real'}
experiments_{3} = {'exp5_1_real','exp5_2_real','exp5_3_real'}
experiments_{4} = {'exp6_1_real','exp6_3_real','exp6_4_real'}
experiments_{5} = {'exp7_2_real','exp7_3_real','exp7_4_real'}



for h = 1:1:5
    experiments = experiments_{h};
    orignal_plan = [];
    plans= [];
    for i = 1:1:length(experiments)
        str = strcat('Reading eperiment: ',experiments{i});
        disp(str)
        [data{i}, orignal_plan{i}, plans{i}] = readExpData(experiments{i});
    end
    disp('computing statistics...')
    elapsed_times{h} = getStatistics(data);
end

%% DISP DATA
clc
format hex
disp('----------------------------- STATISTICS ----------------------------- ')
disp('n objects:     3            4             5            6            7')
str = sprintf('n actions:   %0.4d   +/- %0.4d    |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d     ',mean(elapsed_times{1}{1}),std(elapsed_times{1}{1}),mean(elapsed_times{2}{1}),std(elapsed_times{2}{1}),mean(elapsed_times{3}{1}), std(elapsed_times{3}{1}),mean(elapsed_times{4}{1}),std(elapsed_times{4}{1}),mean(elapsed_times{5}{1}),std(elapsed_times{5}{1}));
disp(str)
str = sprintf('perception:   %0.4d   +/- %0.4d    |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d     ',mean(elapsed_times{1}{2}),std(elapsed_times{1}{2}),mean(elapsed_times{2}{2}),std(elapsed_times{2}{2}),mean(elapsed_times{3}{2}), std(elapsed_times{3}{2}),mean(elapsed_times{4}{2}),std(elapsed_times{4}{2}),mean(elapsed_times{5}{2}),std(elapsed_times{5}{2}));
disp(str)
str = sprintf('planning:   %0.4d   +/- %0.4d    |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d     ',mean(elapsed_times{1}{3}),std(elapsed_times{1}{3}),mean(elapsed_times{2}{3}),std(elapsed_times{2}{3}),mean(elapsed_times{3}{3}), std(elapsed_times{3}{3}),mean(elapsed_times{4}{3}),std(elapsed_times{4}{3}),mean(elapsed_times{5}{3}),std(elapsed_times{5}{3}));
disp(str)
str = sprintf('execution:   %0.4d   +/- %0.4d    |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d     ',mean(elapsed_times{1}{4}),std(elapsed_times{1}{4}),mean(elapsed_times{2}{4}),std(elapsed_times{2}{4}),mean(elapsed_times{3}{4}), std(elapsed_times{3}{4}),mean(elapsed_times{4}{4}),std(elapsed_times{4}{4}),mean(elapsed_times{5}{4}),std(elapsed_times{5}{4}));
disp(str)
str = sprintf('total:   %0.4d   +/- %0.4d    |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d     ',mean(elapsed_times{1}{5}),std(elapsed_times{1}{5}),mean(elapsed_times{2}{5}),std(elapsed_times{2}{5}),mean(elapsed_times{3}{5}), std(elapsed_times{3}{5}),mean(elapsed_times{4}{5}),std(elapsed_times{4}{5}),mean(elapsed_times{5}{5}),std(elapsed_times{5}{5}));
disp(str)