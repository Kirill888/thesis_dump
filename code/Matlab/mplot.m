function [hmap,htext] =  mplot(m, center_style, ellipse_style,plot_opts)
%function hmap =  mplot(m, center_style, ellipse_style, plot_opts)

  
  x = m(:,1);
  y = m(:,2);

  n = size(m,1);

  p = zeros(2,2,n);

  plot_text = 0;

  if nargin > 3
    if plot_opts ~= 0
      plot_text = 1;
    end
  end
  

  for i = 1:n
    p(:,:,i) = [m(i,3:4); m(i,5:6)];

    if plot_text
      s(i,:) = sprintf('%02d',i);
    end

  end

  if ~isempty(ellipse_style)
    [ex,ey] = ellipse([x,y],p);

    if nargout > 0
      hmap = plot(x,y,center_style,ex',ey',ellipse_style);
    else
      plot(x,y,center_style,ex',ey',ellipse_style);
    end
  else
    if nargout > 0
      hmap = plot(x,y,center_style);
    else
      plot(x,y,center_style);
    end
  end
  
  if plot_text
    if nargout > 1
       htext = text(x,y,s);
    else
       text(x,y,s);
    end
  end
