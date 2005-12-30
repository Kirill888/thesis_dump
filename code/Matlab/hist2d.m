function [h,x_axis,y_axis] = hist2d(xx, n, w)
% function h = hist2d(xx)
%   compute histogram in 2d space
%
  
  if nargin < 2
    n = [100,100];
  elseif length(n) == 1
    n = [n ,n];
  end
  
  if nargin < 3
    w = ones(size(xx,1),1);
  end
  
  
  h = zeros(n(2),n(1));
  
  xx_min = min(xx);
    

  xx(:,1) = xx(:,1) - xx_min(1);
  xx(:,2) = xx(:,2) - xx_min(2);
  
  dx = 1.0000001*max(xx(:,1));
  dy = 1.0000001*max(xx(:,2));
  
  nx = size(xx,1);
  
  for i = 1:nx
    col = floor(xx(i,1)/dx*n(1)) + 1;
    row = floor(xx(i,2)/dy*n(2)) + 1;
    
%    disp(sprintf('%02d, %02d',row,col));
    
    h(row,col) = h(row,col) + w(i);
  end
  
  if nargout > 1
    x_axis = linspace(xx_min(1), xx_min(1)+dx,n(1));
    y_axis = linspace(xx_min(2), xx_min(2)+dy,n(2));
  end
  
  
 
    
  