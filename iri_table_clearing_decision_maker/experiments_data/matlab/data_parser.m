% Nicola Covallero
% Parser for the data of experiments for the project: https://bitbucket.org/NicolaCov/iri_table_clearing_planning
% data parser: format v1.0

addpath(genpath('../'));
addpath(genpath('utils/'));

clear all;
close all
format short
init_exp_process;
experiments_{1} = {'exp3_1_real','exp3_2_real','exp3_3_real'};
experiments_{2} = {'exp4_1_real','exp4_2_real','exp4_4_real'};
experiments_{3} = {'exp5_1_real','exp5_2_real','exp5_3_real'};
experiments_{4} = {'exp6_1_real','exp6_3_real','exp6_4_real'};
experiments_{5} = {'exp7_2_real','exp7_3_real','exp7_4_real'};

for h = 1:1:size(experiments_,2)
    experiments = experiments_{h};
    orignal_plan = [];
    plans= [];
    for i = 1:1:length(experiments)
        str = strcat('Reading eperiment: ',experiments{i});
        disp(str)
        % there is bug to read the plans since the first action is not saved
         % correctly in data.txt
        [data{h}{i}, orignal_plan{h}{i}, plans{h}{i}] = readExpData(experiments{i});
    end
    disp('computing statistics...')
    elapsed_times{h} = getStatistics(data{h});
    disp('--------------------')
    disp(' ') % new line
end

%% Create a bar plot of a specific experiment
% Show the elapsed times for each iteration, this function creates 3 bar plots:
% 1) Elapsed times for the perception, planning and execution sybsystems
% 2) Elapsed times fot state generation, planning (get a plan), IK of
% first action
% 3) Elapsed times for planning (get a plan), IK of first action
% Note: the backtracking is considered as an iteration, we can notice when
% a backtracking is done since the perception time is 0.

% series_of_exp; % number of the series of experiments
% n_exp; % number of the experiment of the series
% example: series_of_exp = 1; n_exp = 2; there will be shown the elapsed
% times of the experiment 'exp3_2_real'
% createBarPlots_v2(data{series_of_exp},n_exp);

close all
series_of_exp = 5;
n_exp = 1;
createBarPlots_v2(data{series_of_exp},n_exp);

%% DISPLAY DATA
clc
format short


disp('----------------------------- STATISTICS ----------------------------- ')
disp('n objects:     3            4             5            6            7')
i = 1;
str = sprintf('n actions:   %0.4d   +/- %0.4d    |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d     ',mean(elapsed_times{1}{i}),std(elapsed_times{1}{i}),mean(elapsed_times{2}{i}),std(elapsed_times{2}{i}),mean(elapsed_times{3}{i}), std(elapsed_times{3}{i}),mean(elapsed_times{4}{i}),std(elapsed_times{4}{i}),mean(elapsed_times{5}{i}),std(elapsed_times{5}{i}));
disp(str)
disp(' ') % new line

i= 2;
str = sprintf('perception [s]:   %0.4d   +/- %0.4d    |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d     ',mean(elapsed_times{1}{i}),std(elapsed_times{1}{i}),mean(elapsed_times{2}{i}),std(elapsed_times{2}{i}),mean(elapsed_times{3}{i}), std(elapsed_times{3}{i}),mean(elapsed_times{4}{i}),std(elapsed_times{4}{i}),mean(elapsed_times{5}{i}),std(elapsed_times{5}{i}));
disp(str)
disp(' ') % new line

i = 3;
str = sprintf('planning [s]:   %0.4d   +/- %0.4d    |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d     ',mean(elapsed_times{1}{i}),std(elapsed_times{1}{i}),mean(elapsed_times{2}{i}),std(elapsed_times{2}{i}),mean(elapsed_times{3}{i}), std(elapsed_times{3}{i}),mean(elapsed_times{4}{i}),std(elapsed_times{4}{i}),mean(elapsed_times{5}{i}),std(elapsed_times{5}{i}));
disp(str)
disp(' ') % new line

i = 4;
str = sprintf('execution [s]:   %0.4d   +/- %0.4d    |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d     ',mean(elapsed_times{1}{i}),std(elapsed_times{1}{i}),mean(elapsed_times{2}{i}),std(elapsed_times{2}{i}),mean(elapsed_times{3}{i}), std(elapsed_times{3}{i}),mean(elapsed_times{4}{i}),std(elapsed_times{4}{i}),mean(elapsed_times{5}{i}),std(elapsed_times{5}{i}));
disp(str)
disp(' ') % new line

i= 5;
str = sprintf('total [s]:   %0.4d   +/- %0.4d    |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d     ',mean(elapsed_times{1}{i}),std(elapsed_times{1}{i}),mean(elapsed_times{2}{i}),std(elapsed_times{2}{i}),mean(elapsed_times{3}{i}), std(elapsed_times{3}{i}),mean(elapsed_times{4}{i}),std(elapsed_times{4}{i}),mean(elapsed_times{5}{i}),std(elapsed_times{5}{i}));
disp(str)
disp(' ') % new line

i = 6;
str = sprintf('decision making [s]:   %0.4d   +/- %0.4d    |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d     ',mean(elapsed_times{1}{i}),std(elapsed_times{1}{i}),mean(elapsed_times{2}{i}),std(elapsed_times{2}{i}),mean(elapsed_times{3}{i}), std(elapsed_times{3}{i}),mean(elapsed_times{4}{i}),std(elapsed_times{4}{i}),mean(elapsed_times{5}{i}),std(elapsed_times{5}{i}));
disp(str)
disp(' ') % new line

i = 7;
str = sprintf('n backtrackings:   %0.4d   +/- %0.4d    |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d   |   %0.4d   +/- %0.4d     ',mean(elapsed_times{1}{i}),std(elapsed_times{1}{i}),mean(elapsed_times{2}{i}),std(elapsed_times{2}{i}),mean(elapsed_times{3}{i}), std(elapsed_times{3}{i}),mean(elapsed_times{4}{i}),std(elapsed_times{4}{i}),mean(elapsed_times{5}{i}),std(elapsed_times{5}{i}));
disp(str)