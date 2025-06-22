function testRes = runAllTests()
%RUNALLTESTS    Run all tests from the "xUintTests" directory. 
%   TESTRES = RUNALLTESTS() runs all tests from the "xUnitTests" directory
%   and returns the results TESTRES.
% 
 
% Get the root folder of the project - this should also work when the
% project is referenced in another project.
path2RootFolder = mfilename('fullpath');
path2RootFolder = path2RootFolder(1:end-numel(mfilename()));

% Add current folder to path - this is needed for pre R2019a releases that
% don't support Matlab Projects
addpath(path2RootFolder)

% Create and run the test suite
testSuite = matlab.unittest.TestSuite.fromFolder(...
    [path2RootFolder 'xUnitTests'], ...
    'IncludingSubfolders',true);

str = ' Running "Path2D" tests ... ';
printTopRule('=', numel(str))
fprintf('<strong>%s</strong>\n', str);
printTopRule('=', numel(str))

testRes = run(testSuite);

end%fcn


function printTopRule(symbol, n)
fprintf('%s\n', repmat(symbol, 1, n));
end%fcn
