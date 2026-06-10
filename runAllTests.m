function testRes = runAllTests(doPlot)
%RUNALLTESTS    Run all tests from the "xUintTests" directory. 
%   TESTRES = RUNALLTESTS() runs all tests from the "xUnitTests" directory
%   and returns the results TESTRES.
% 

if nargin < 1
    doPlot = true;
end

% Get the root folder of the project - this should also work when the
% project is referenced in another project.
path2RootFolder = mfilename('fullpath');
path2RootFolder = path2RootFolder(1:end-numel(mfilename()));

% Add current folder to path - this is needed for pre R2019a releases that
% don't support Matlab Projects
addpath([path2RootFolder, 'src'])
addpath([path2RootFolder, 'xUnitTests' filesep() 'testdata'])

% Create and run the test suite
if verLessThan('matlab', '9.5')
    % fromFolder() requires R2018b or newer
    testSuite = matlab.unittest.TestSuite.fromFolder(...
        [path2RootFolder 'xUnitTests'], ...
        'IncludingSubfolders',true);
else
    p = matlab.unittest.parameters.Parameter.fromData('DoPlot',{doPlot});
    testSuite = matlab.unittest.TestSuite.fromFolder(...
        [path2RootFolder 'xUnitTests'], ...
        'IncludingSubfolders',true, ...
        'ExternalParameters', p);
end

str = ' Running "Path2D" tests ... ';
printTopRule('=', numel(str))
fprintf('<strong>%s</strong>\n', str);
printTopRule('=', numel(str))

testRes = run(testSuite);

close all

end%fcn


function printTopRule(symbol, n)
fprintf('%s\n', repmat(symbol, 1, n));
end%fcn
