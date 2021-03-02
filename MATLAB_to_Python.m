function [u] = MATLAB_to_Python(x)
    u = 0;
    coder.extrinsic('py.parallelhybrid_ECMS.parallelhybrid') % Python functions have to be run extrinsically, meaning no C code generated
    u = py.parallelhybrid_ECMS.parallelhybrid(x);
    u  = cell(u);
    u =  cell2mat(u)';
    u = double(u);
end


