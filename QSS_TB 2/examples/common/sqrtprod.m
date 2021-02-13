function z = sqrtprod(x,y)
% Elementwise square root of a product. 
%   z=sqrtprod(x,y), where x and y are real vector or scalar CVX variables,
%   computes the elementwise operation z=sqrt(x.*y).

sx=size(x); 
sy=size(y);
if ~isvector(x) || ~isvector(y)
    error('quadlin accepts only scalar and vector arguments!');
elseif ~isscalar(x) && ~isscalar(y) && any(sx~=sy)
    error('Input vectors must be of equal size!');
end
if isscalar(y)
    sz=sx;
    x=x(:);
else
    sz=sy;
    x=x(:); y=y(:); % make them column vectors
end
N=max(sz);  

cvx_begin
    hypograph variable z(N,1);
    norms([2*z, y-x],2,2) <= y+x;
cvx_end
z=reshape(z,sz);





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2014-01.