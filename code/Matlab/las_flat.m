function [x,y] = las_flat(las,odo, maxRange)
% function [x,y] = las_flat(las,odo, maxRange)
%
%  las -- T r1 r2 r3 ... r361
%  odo -- [x,y,a] -- location of the sensor
%  maxRange -- no return value

%

  a = linspace(-pi/2, pi/2, 361);
  n = size(las,1);
  
  np = 0;

  for i = 1:n
    R = las(i,2:362);
    ii = find(R < maxRange);
    
    xx = R(ii).*cos(odo(i,3) + a(ii)) + odo(i,1);
    yy = R(ii).*sin(odo(i,3) + a(ii)) + odo(i,2);
    
    k = length(ii);
    
    x(np+1:np+k) = xx;
    y(np+1:np+k) = yy;
    
    np = np + k;
  end


