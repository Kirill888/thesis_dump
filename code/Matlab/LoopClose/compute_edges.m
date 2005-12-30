function [r,Srr, a, Saa] = compute_edges(m)
% function  [r,Srr, a, Saa] = compute_edges(m)

  n = size(m,1);
  ne = n*(n-1)/2;
  
  if ne <= 0
    e = zeros(0,6);
    return
  end
  
  from = [];
  to   = [];
  
  for i = 1:n
    i2 = (i+1):n;
    i1 = i2*0 + i;
    
    from = [from,i1];
    to   = [to, i2];
  end

  %Compute length and angles  
  
  dx = m(to,1) - m(from,1);
  dy = m(to,2) - m(from,2);
  dx2 = dx.*dx;  dy2 = dy.*dy; dxdy = dx.*dy;
  
  r2 = dx2 + dy2;
  r  = sqrt(r2);
  a  = atan2(dy,dx);
  
  dx_r  = dx./r;
  dy_r  = dy./r;
  dx_r2 = dx./r2;
  dy_r2 = dy./r2;
  
  %Compute jacobians
  ja = zeros(ne, 2*n);
  jr = zeros(ne, 2*n);
%  disp(sprintf('Edge[%d] = %d=>%d\n',[1:ne;from;to]));
    
  
  for i = 1:ne
    f = from(i); t = to(i);
    jr(i,2*f-1) = -dx_r(i);  % dR(i)/dx(from)
    jr(i,2*f  ) = -dy_r(i);  % dR(i)/dy(from)
    jr(i,2*t-1) =  dx_r(i);  % dR(i)/dx(to)
    jr(i,2*t  ) =  dy_r(i);  % dR(i)/dy(to)

    ja(i,2*f-1) =  dy_r2(i); % dA(i)/dx(from)
    ja(i,2*f  ) = -dx_r2(i); % dA(i)/dy(from)
    ja(i,2*t-1) = -dy_r2(i); % dA(i)/dx(to)
    ja(i,2*t  ) =  dx_r2(i); % dA(i)/dy(to)
    
  end

  %Compute merged covariance of the input
  Cov = zeros(2*n);
  
  for i = 1:n
    Cov([2*i-1,2*i], [2*i-1,2*i]) =  [m(i,[3,4]); m(i,[5,6])];
  end
  
  Srr = jr*Cov*jr';
  Saa = ja*Cov*ja';
 
  
  
