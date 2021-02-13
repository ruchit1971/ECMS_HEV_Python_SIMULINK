function [gearratio] = gearbox(u);

switch u(1)
case 0
   gearratio = 0;
case 1
   gearratio = u(2);
case 2
   gearratio = u(3);
case 3
   gearratio = u(4);
case 4
   gearratio = u(5);
case 5
   gearratio = u(6);
otherwise
   disp('no valid input to gearbox');
end

gearratio = gearratio*u(7);