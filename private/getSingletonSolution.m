function [dMin,QMin,tauMin] = getSingletonSolution(Q, x, y, tau)
% Get the closest singleton solution.
%   

% The distance from each point of Q to (x,y)
d = hypot(Q(:,1) - x, Q(:,2) - y);

% The distance and the closest point
[dMin,idxMin] = min(d);
QMin = Q(idxMin,:);
tauMin = tau(idxMin);

% Now, we need to check if there is a different point in Q that has the
% same minimum distance
hasMinDist = (d == dMin);
hasMinDist(idxMin) = false; % Do not check the potential solution
flag = isSingleton(QMin, Q(hasMinDist,:));
if ~flag
    dMin(:) = NaN;
    QMin(:) = NaN;
    tauMin(:) = NaN;
end

end%fcn


function flag = isSingleton(QRef, QTest)

for i = 1:size(QTest, 1)
    Qi = QTest(i,:);
    if ~isequal(QRef, Qi)
        flag = false;
        return
    end
end%for
flag = true;

end%fcn
