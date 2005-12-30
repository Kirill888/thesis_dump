function [xy0] = pmerge(xy)

  xy_base = xy(1,1:2);

  x = xy(:,1) - xy_base(1);
  y = xy(:,2) - xy_base(2);

  xy0 = [x, y];
