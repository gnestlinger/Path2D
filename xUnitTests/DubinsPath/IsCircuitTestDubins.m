classdef IsCircuitTestDubins < matlab.unittest.TestCase
    
    properties (TestParameter)
    end
    
    methods (Test)
        
        function testCircuitTrue(testCase)
            R = 2;
            d = 3;
            dub = DubinsPath([0 0 0], [1 0 1 0], [R*pi d R*pi d], R);
            verifyTrue(testCase, dub.IsCircuit);
        end%fcn
        
        function testCircuitFalse(testCase)
            R = 2;
            d = 3;
            dub = DubinsPath([0 0 0], [1 0 1 0], [R*pi d R*pi d*0.99], R);
            verifyFalse(testCase, dub.IsCircuit);
        end%fcn
        
    end
    
end%class
