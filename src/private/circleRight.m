function [x,y,c] = circleRight(r, head)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% We can avoid calculating phi = head + pi/2 by using the identities
%   cos(x + pi/2) = -sin(x) and
%   sin(x + pi/2) = cos(x)
x = r*-sin(head);
y = r*cos(head);
c = -1/r*ones(size(x));

end%fcn
