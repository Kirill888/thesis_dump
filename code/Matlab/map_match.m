function [d] = map_match(m1, m2, doPlot)
%function [d] = map_match(m1, m2)
  
  n = size(m1,1);
  d = zeros(n,2);
  
  if nargin < 3
    doPlot =0;
  end
  
  if doPlot
    figure
    mplot(m1,'ro','r-');
    hold on
    mplot(m2,'go','g-');
  end
  
  for i = 1:n
    a = m1(i,1:2)';
    b = m2(i,1:2)';
    Paa = [ m1(i, 3:4); m1(i, 5:6) ];
    Pbb = [ m2(i, 3:4); m2(i, 5:6) ];
  
    [c, Pcc] = cov_intersect(a, Paa, b, Pbb, 0.5);
    
    d(i,1) = mahalanobis2(a', Paa, c');
    d(i,2) = mahalanobis2(b', Pbb, c');

    if doPlot
      mplot([c' Pcc(1,1:2) Pcc(2,1:2)],'mo','m-');
      text(c(1),c(2), num2str(d(i,:)));
    end

  end
  
  if doPlot
    axis equal
  end