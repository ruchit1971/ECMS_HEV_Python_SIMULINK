function z = quadlinsum(x,y)
% Sum of quadratic over linear operations. 
%   z=quadlinsum(x,y), where x is a vector and y is a scalar CVX variable,
%   computes the operation z=sum(x.^2/y).

sx=size(x); 
if ~isvector(x) || ~isscalar(y)
    error('quadlinsum accepts only a vector as a first argument and a scalar as a second argument!');
end
if isscalar(x)
    N=1;
else
    N=max(sx);
    x=x(:); % make it a column vector
end

cvx_begin
    epigraph variable z;
    cone = lorentz(N+1, 1);
    cvx_optpnt.x = cone.x(1:N);
    temp = cone.x(N+1);
    cvx_optpnt.y = cone.y + temp;
    cvx_optpnt.z = cone.y - temp;
    {x, y, z} == cvxtuple(cvx_optpnt);
cvx_end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-01.