function display_grid_data(fname)
% function display_grid_data(fname)
%
%
%

  run(fname);
  
  if ~exist('odo', 'var') || ~exist('las','var')
    disp('Bad file');
  end
  
  nscans = size(odo,1);
  
  xx = []; rxx = [];
  yy = []; ryy = [];
  for i = 1:nscans
    [x,y] = scan2points(odo(i,:), las(i,:));
    xx = [xx,x]; yy = [yy,y];
    
    rxx = [rxx, ones(size(x))*odo(i,1)];
    ryy = [ryy, ones(size(y))*odo(i,2)];
  end

  h = line([rxx;xx], [ryy; yy], 'Color','g','LineStyle',':');
  hold on; axis equal
  plot(xx,yy,'g.');
  plot_robot(odo,'r');

function [xx,yy] = scan2points(odo, las)
  a = linspace(-pi/2,pi/2, 361);
  a = a + odo(3);
  
  ca = cos(a); sa = sin(a);

  xx = ca.*las + odo(1);
  yy = sa.*las + odo(2);

