%% Import data from text file
% Script for importing data from the following text file:
%
%    filename: C:\Users\Zuhair\Documents\UTEXAS\Spring 2023\ASBR\HW3-PA1\HW3-PA1\pa1-debug-a-calbody.txt
%
% Auto-generated by MATLAB on 19-Apr-2023 21:55:55

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 3);

% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["VarName1", "VarName2", "VarName3"];
opts.VariableTypes = ["double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
pa1debugacalbody = readtable("HW3-PA1\pa1-debug-a-calbody.txt", opts);

%% Convert to output type
pa1debugacalbody = table2array(pa1debugacalbody);

%% Clear temporary variables
clear opts