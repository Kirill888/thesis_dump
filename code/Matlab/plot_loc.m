function plot_loc(odo,step)
  n = size(odo,1);
  
  for i = 1:step:n
    o = row2odo(odo(i,:));
    plot_robot(o,'g.-',0.001);
    hold on
  end
  
  axis equal

function [odo,w] = row2odo(row)
  np = size(row,2)/4;
  
  odo = zeros(np,3);
  w   = zeros(np,3);
  
  for i = 0:np-1
    odo(i+1,:) = row(1 + i*4: 3 + i*4);
    w(i+1)     = row(4 + i*4);
  end
  