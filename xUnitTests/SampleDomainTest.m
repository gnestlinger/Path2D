classdef SampleDomainTest < matlab.unittest.TestCase
    
    properties (TestParameter)
        obj = struct(...
            'PolygonPathEmpty', PolygonPath(), ...
            'PolygonPathNonEmpty', PolygonPath.xy2Path(1:10, 1:10), ...
            'SplinePathEmpty', SplinePath(), ...
            'SplinePathNonEmpty', SplinePath([0 1 2], reshape([1 0; 0 0; 1 1; 1 0], 2,2,2)));
        uintx = {'uint8','uint16','uint32','uint64'};
        float = {'single', 'double'};
    end
    
    
    methods (Test)
        function testUint8(testCase, obj, uintx)
            
            N = cast(10, uintx);
            if isempty(obj)
                testCase.verifyError(@() obj.sampleDomain(N), ...
                    'PATH2D:sampleDomain:PathMustBeNonempty');
                return
            end
            
            tau = obj.sampleDomain(N);
            flag = isempty(obj);
            testCase.verifySize(tau, [N*double(~flag),1]);
            
        end%fcn
        
        function testFloat(testCase, obj, float)
            
            dtau = cast(1.5, float);
            if isempty(obj)
                testCase.verifyError(@() obj.sampleDomain(dtau), ...
                    'PATH2D:sampleDomain:PathMustBeNonempty');
                return
            end
            
            tau = obj.sampleDomain(dtau);
            flag = (unique(diff(tau), 'stable') == dtau);
            testCase.verifyTrue(numel(flag) < 3);
            testCase.verifyTrue(isempty(flag) || flag(1));
            
        end%fcn
    end
    
end%class
