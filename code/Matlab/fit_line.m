function [l_abc, err] = fit_line(x,y)

  dx = sum(abs(diff(x)));
  dy = sum(abs(diff(y)));
  
  if dx > dy
    l = polyfit(x,y,1);
    s = 1./sqrt(1 + l(1)*l(1));
    l_abc = [l(1)*s, -s, l(2)*s];
  else
    l = polyfit(y,x,1);
    s = 1./sqrt(1 + l(1)*l(1));
    l_abc = [-s, l(1)*s, l(2)*s];
  end

  if nargout > 1
    err = x.*l_abc(1) + y.*l_abc(2) + l_abc(3);
  end
