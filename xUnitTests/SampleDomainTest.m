classdef SampleDomainTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        obj = struct(...
            'PolygonPathEmpty', PolygonPath(), ...
            'PolygonPathNonEmpty', PolygonPath.xy2Path(1:10, 1:10));
        uintx = {'uint8','uint16','uint32','uint64'};
        float = {'single', 'double'};
    end
    
    
    methods (Test)
        function testUint8(testCase, obj, uintx)
            
            N = cast(10, uintx);
            tau = obj.sampleDomain(N);
            testCase.verifySize(tau, [N,1]);
            
        end%fcn
        
        function testFloat(testCase, obj, float)
            
            if isempty(obj)
                % TODO: decide what should be returned in this case
                return
            end
            dtau = cast(1.5, float);
            tau = obj.sampleDomain(dtau);
            flag = (unique(diff(tau), 'stable') == dtau);
            testCase.verifyTrue(numel(flag) < 3);
            testCase.verifyTrue(flag(1));
            
        end%fcn
    end
    
end%class
