function R = rotmat2D(phi) %#codegen
%ROTMAT2D	Rotation matrix in 2D.
%	R = ROTMAT2D(PHI) returns the rotation matrix R for a rotation of PHI
%	in radians. For a 2-by-N matrix XY of N vectors [x; y], utilizing
%	vectorization, the rotation is then performed by R(PHI)*XY.
%	
%	Notice: 
%	 (1) R'(p) = R(-p)
%	 (2) inv(R(p)) = R'(p) = R(-p)
% 

% Subject: Computational Geometry.
% $Author: georgnestlinger $
% $LastChangedDate: 2021-04-02 10:27:46 +0200 (Fr., 02 Apr 2021) $
% $Revision: 1867 $

R = [...
	cos(phi), -sin(phi); 
	sin(phi),  cos(phi)];

end%fcn
