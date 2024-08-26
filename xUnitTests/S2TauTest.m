classdef S2TauTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        PathObj = struct(...
            'PolygonPathEmpty',PolygonPath(), ...
            'PolygonPathZeroLength',PolygonPath(1, 1, 0, 0), ...
            'PolygonPathNonEmpty',PolygonPath.xy2Path(0:10, zeros(1,11)))

        s = struct(...
            'emptyColS',zeros(0,1), ...
            'emptyRowS',zeros(1,0), ...
            'scalarS',1.1, ...
            'vectorS',linspace(-1,10,24), ...
            'ndS',ones(10,2,3))
    end
    
    
    
    methods (Test)
        function testReturnSize(testCase, PathObj, s)
            [tau,idx] = PathObj.s2tau(s);
            
            verifySize(testCase, tau, size(s));
            verifySize(testCase, idx, size(s));
            verifyClass(testCase, idx, 'uint32');
        end%fcn
    end
    
end%class
