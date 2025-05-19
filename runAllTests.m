% Script to run all tests from the "xUintTests" directory. 

clear
clc

% Add current folder to path - this is needed for pre R2019a releases that
% don't support Matlab Projects
addpath(pwd)

% Create and run the test suite
testSuite = matlab.unittest.TestSuite.fromFolder(...
    'xUnitTests', ...
    'IncludingSubfolders',true);
testRes = run(testSuite);
disp(testRes)