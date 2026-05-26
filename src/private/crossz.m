function z = crossz(u, v)
%CROSSZ		Vectorized z-component of cross product.
z = u(:,1).*v(:,2) - u(:,2).*v(:,1);
end%fcn
