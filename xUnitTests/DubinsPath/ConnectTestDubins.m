classdef ConnectTestDubins < matlab.unittest.TestCase
    
    properties (TestParameter)       
        ConfigsRight = {...
            {[0 0 0],    [0 -2 -pi], 1};
            {[1 1 pi/2], [3 1 -pi/2], 1};
            {[1 1 pi/2], [5 1 -pi/2], 2};
            };
        
        ConfigsLeft = {...
            {[0 0 0],    [0 2 pi], 1};
            {[3 1 pi/2], [1 1 3*pi/2], 1};
            {[3 1 pi/2], [-1 1 3*pi/2], 2};
            };
        
        ConfigsXSY = {...
            {[2 3 pi/4], [-2 3 -pi/4], 2, 'LSL'};
            {[2 3 pi/4], [-2 3 +pi/4], 1, 'LSR'};
            {[-2 3 pi/4], [2 3 +pi/4], 1, 'RSL'};
            {[-2 3 pi/4], [2 3 -pi/4], 1, 'RSR'};
            };
        
        ConfigsLRL = {...
            {[1 2 pi/2], [5 2 -pi/2], 3}}
        
        ConfigsRLR = {...
            {[5 2 pi/2], [1 2 -pi/2], 3}}
    end
    
    methods (Test, ParameterCombination='sequential')
        function testRightTurn(testCase, ConfigsRight)

            C0 = ConfigsRight{1};
            C1 = ConfigsRight{2};
            R = ConfigsRight{3};
            dub = DubinsPath.connect(C0, C1, R);
%             [tau0,tau1] = dub.domain();
%             [x,y,~,h] = dub.eval([tau0 tau1]);
            [x,y,~,h] = dub.eval();
            

%             checkAgainstMatlabImpl(dub, C0, C1, R);
            
            % Check terminal points
            testCase.verifyEqual([dub.InitialPos' dub.InitialAng], C0)
            testCase.verifyEqual([x(1) y(1) h(1)], C0);
            testCase.verifyEqual([x(end) y(end) h(end)], C1, 'AbsTol',1e-15);
            
            % Check segment types/lengths
            testCase.verifyEqual(dub.SegmentTypes, [dub.LEFT dub.RIGHT dub.LEFT]);
            S = abs(C1(end)-C0(end))*R;
            testCase.verifyEqual(dub.SegmentLengths, [0 S 0]);
            
            testCase.verifyFalse(dub.IsCircuit)
        end%fcn
        
        function testLeftTurn(testCase, ConfigsLeft)

            C0 = ConfigsLeft{1};
            C1 = ConfigsLeft{2};
            R = ConfigsLeft{3};
            dub = DubinsPath.connect(C0, C1, R);
%             [tau0,tau1] = dub.domain();
%             [x,y,~,h] = dub.eval([tau0 tau1]);
            [x,y,~,h] = dub.eval();
            
            % Check against Matlab implementation
%             checkAgainstMatlabImpl(dub, C0, C1, R);
            
            % Check terminal points
            testCase.verifyEqual([dub.InitialPos' dub.InitialAng], C0)
            testCase.verifyEqual([x(1) y(1) h(1)], C0);
            testCase.verifyEqual([x(end) y(end) h(end)], C1, 'AbsTol',1e-15);
            
            % Check segment types/lengths
            testCase.verifyEqual(dub.SegmentTypes, [dub.LEFT dub.RIGHT dub.LEFT]);
            S = abs(C1(end)-C0(end))*R;
            testCase.verifyEqual(dub.SegmentLengths, [S/2 0 S/2]);
            
            testCase.verifyFalse(dub.IsCircuit)
        end%fcn
        
        function testXSY(testCase, ConfigsXSY)

            C0 = ConfigsXSY{1};
            C1 = ConfigsXSY{2};
            R = ConfigsXSY{3};
            T = ConfigsXSY{4};
            dub = DubinsPath.connect(C0, C1, R);
%             [tau0,tau1] = dub.domain();
%             [x,y,~,h] = dub.eval([tau0 tau1]);
            [x,y,~,h] = dub.eval();
            
            % Check against Matlab implementation
%             ml = checkAgainstMatlabImpl(dub, C0, C1, R);
            
            % Check terminal points
            testCase.verifyEqual([dub.InitialPos' dub.InitialAng], C0)
            testCase.verifyEqual([x(1) y(1) h(1)], C0);
            testCase.verifyEqual([x(end) y(end)], C1(1:2), 'AbsTol',1e-15);
            testCase.verifyEqual(mod2Pi(h(end)), mod2Pi(C1(end)), 'AbsTol',1e-15);
            
            % Check segment types/lengths
            testCase.verifyEqual(dub.convertSegmentType2Char(), T);
%             testCase.verifyEqual(dub.SegmentLengths, [0 0 0]);
            
            testCase.verifyFalse(dub.IsCircuit)
        end%fcn
        
        function testLRL(testCase, ConfigsLRL)

            C0 = ConfigsLRL{1};
            C1 = ConfigsLRL{2};
            R = ConfigsLRL{3};
            dub = DubinsPath.connect(C0, C1, R);
%             [tau0,tau1] = dub.domain();
%             [x,y,~,h] = dub.eval([tau0 tau1]);
            [x,y,~,h] = dub.eval();
            
            % Check against Matlab implementation
%             ml = checkAgainstMatlabImpl(dub, C0, C1, R);
            
            % Check terminal points
            testCase.verifyEqual([dub.InitialPos' dub.InitialAng], C0)
            testCase.verifyEqual([x(1) y(1) h(1)], C0);
            testCase.verifyEqual([x(end) y(end)], C1(1:2), 'AbsTol',1e-15);
            testCase.verifyEqual(mod2Pi(h(end)), mod2Pi(C1(end)), 'AbsTol',1e-15);
            
            % Check segment types/lengths
            testCase.verifyEqual(dub.convertSegmentType2Char(), 'LRL');
            
            testCase.verifyFalse(dub.IsCircuit)
        end%fcn
        
        function testRLR(testCase, ConfigsRLR)

            C0 = ConfigsRLR{1};
            C1 = ConfigsRLR{2};
            R = ConfigsRLR{3};
            dub = DubinsPath.connect(C0, C1, R);
%             [tau0,tau1] = dub.domain();
%             [x,y,~,h] = dub.eval([tau0 tau1]);
            [x,y,~,h] = dub.eval();
            
            % Check against Matlab implementation
%             ml = checkAgainstMatlabImpl(dub, C0, C1, R);
            
            % Check terminal points
            testCase.verifyEqual([dub.InitialPos' dub.InitialAng], C0)
            testCase.verifyEqual([x(1) y(1) h(1)], C0);
            testCase.verifyEqual([x(end) y(end)], C1(1:2), 'AbsTol',1e-15);
            testCase.verifyEqual(mod2Pi(h(end)), mod2Pi(C1(end)), 'AbsTol',1e-15);
            
            % Check segment types/lengths
            testCase.verifyEqual(dub.convertSegmentType2Char(), 'RLR');
            
            testCase.verifyFalse(dub.IsCircuit)
        end%fcn
        
    end
    
end%class


function ds = checkAgainstMatlabImpl(dub, C0, C1, R)
% Check against Matlab implementation 
%   Requires R2019b!

dc = dubinsConnection('MinTurningRadius',R);
ds = dc.connect(C0, C1);
ds = ds{1};
ds.show();

if ~isempty(dub)
    hold on
    dub.plot('MarkerIndices',1, 'Marker','o')
    hold off
end

end%fcn

function val = mod2Pi(val)
val = mod(val, 2*pi);
end%fcn
