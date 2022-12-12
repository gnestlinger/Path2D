function r = realroots(c)
%REALROOTS  Return the roots with zero-valued imaginary part.
%   R = REALROOTS(C)
% 
%   See also ROOTS.

rc = roots(c);
r = real(rc(imag(rc) == 0));

end%fcn
