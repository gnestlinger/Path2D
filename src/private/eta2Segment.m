function coefs = eta2Segment(eta_, Xa, Xb)
%ETA2SEGMENT    Coefficients for a G2 continuous eta² spline segment.
%   COEFS = ETA2SEGMENT(ETA, XA, XB)
% 
%   References:
%    A. Piazzi and C. Guarino Lo Bianco, "Quintic G²-splines for trajectory
%    planning of autonomous vehicles," Proceedings of the IEEE Intelligent
%    Vehicles Symposium 2000, Dearborn, MI, USA, 2000, pp. 198-203, doi:
%    10.1109/IVS.2000.898341.


if numel(eta_) == 2
    % Assume the user wants a symmetric curve
    eta = [eta_(1); eta_(1); eta_(2); -eta_(2)];
else
    eta = eta_(:);
end
            
xA = Xa(1);
yA = Xa(2);
kA = Xa(4);
cosA = cos(Xa(3));
sinA = sin(Xa(3));

xB = Xb(1);
yB = Xb(2);
kB = Xb(4);
cosB = cos(Xb(3));
sinB = sin(Xb(3));

x0 = xA;
x1 = eta(1)*cosA;
x2 = 0.5*(eta(3)*cosA - eta(1)^2*kA*sinA);
x3 = 10*(xB - xA) - (6*eta(1) + 1.5*eta(3))*cosA ...
    - (4*eta(2) - 0.5*eta(4))*cosB + 1.5*eta(1)^2*kA*sinA ...
    - 0.5*eta(2)^2*kB*sinB;
x4 = -15*(xB - xA) + (8*eta(1) + 1.5*eta(3))*cosA ...
    + (7*eta(2) - eta(4))*cosB - 1.5*eta(1)^2*kA*sinA ...
    + eta(2)^2*kB*sinB;
x5 = 6*(xB - xA) - (3*eta(1) + 0.5*eta(3))*cosA...
    - (3*eta(2) - 0.5*eta(4))*cosB + 0.5*eta(1)^2*kA*sinA...
    - 0.5*eta(2)^2*kB*sinB;

y0 = yA;
y1 = eta(1)*sinA;
y2 = 0.5*(eta(3)*sinA + eta(1)^2*kA*cosA);
y3 = 10*(yB - yA) - (6*eta(1) + 1.5*eta(3))*sinA ...
    - (4*eta(2) - 0.5*eta(4))*sinB - 1.5*eta(1)^2*kA*cosA ...
    + 0.5*eta(2)^2*kB*cosB;
y4 = -15*(yB - yA) + (8*eta(1) + 1.5*eta(3))*sinA ...
    + (7*eta(2) - eta(4))*sinB + 1.5*eta(1)^2*kA*cosA ...
    - eta(2)^2*kB*cosB;
y5 = 6*(yB - yA) - (3*eta(1) + 0.5*eta(3))*sinA...
    - (3*eta(2) - 0.5*eta(4))*sinB - 0.5*eta(1)^2*kA*cosA...
    + 0.5*eta(2)^2*kB*cosB;

coefs = [x5 x4 x3 x2 x1 x0; y5 y4 y3 y2 y1 y0];

end%fcn
