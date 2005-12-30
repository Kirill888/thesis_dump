function [xy,h] = collect_points(point_marker,axis_range,np)
%function [xy,h] = collect_points(point_marker,axis_range,np)
  xy = zeros(0,2);
  h = [];
  i = 0;
  b = 1;
  hold on

  if nargin < 2
    axis_range = [];
  elseif length(axis_range) == 1
      axis_range = [-axis_range,+axis_range,-axis_range,+axis_range]
  end

  if ~isempty(axis_range)
    grid on
    axis(axis_range)
  end

  [x,y,b] = ginput(1);

  if nargin < 3
    np = Inf;
  end

  while (length(b) == 1) & (b ~= 2) 
     i = i + 1;
     h(i) = plot(x,y,point_marker);

     if ~isempty(axis_range)
        axis(axis_range)
     end

     xy(i,:) = [x,y];

     if i >= np
       break;
     end
     [x,y,b] = ginput(1);
  end





