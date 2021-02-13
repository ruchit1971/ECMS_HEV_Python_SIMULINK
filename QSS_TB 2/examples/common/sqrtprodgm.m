function z = sqrtprodgm(x, y)
%Elementwise square root of a product, using the standard geo_mean function
%in CVX.
%   z=sqrtprodgm(x,y), where x is a vector and y is a vector or scalar,
%   computes sqrt(x*y). If y or any of the elements of x are negative, then
%   z=-Inf. 
%
%   Disciplined convex programming information:
%       sqrtprodgm is concave and nondecreasing; therefore, when used
%       in CVX specifications, its argument must be concave.

N1=length(x); N2=length(y);
N=max(N1,N2);
if N1==N && N2==N
    z=geo_mean([x(:) y(:)],2);
elseif N1==N && N2==1
    z=geo_mean([x(:) y*ones(length(x),1)],2);
elseif N1==1 && N2==N
    z=geo_mean([x*ones(length(x),1) y(:)],2);
else
    error('X and Y must have the same length, or one of them should be scalar!');
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-01.