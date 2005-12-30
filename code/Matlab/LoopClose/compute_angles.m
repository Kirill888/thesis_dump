function [xya,cov,jf] = compute_angles(m)
% function [xya,cov,jf] = compute_angles(m)
%
%


 n = size(m,1);
 n2 = 2*n;

 mx = m(:,1);
 my = m(:,2);
 
 x = sum(mx)/n;
 y = sum(my)/n;
 
 a = zeros(1,n);
 
 jf = zeros(2+n,n2);
 
 jf(1,[1:2:n2]) = 1/n;
 jf(2,[2:2:n2]) = 1/n;
 
 m_cov = zeros(n2);
 
 for i = 1:n
    dx = mx(i) - x;
    dy = my(i) - y;
    
    a(i) = atan2(dy,dx);
    
    %Init Jacobian
    r2 = dx*dx + dy*dy;
    
    jx =  dy/(n*r2);
    jy = -dx/(n*r2);
    
    jf(i+2,1:2:end) = jx;
    jf(i+2,2:2:end) = jy;
    
    jf(i+2,(i-1)*2 + 1) = -(n-1)*jx;
    jf(i+2,(i-1)*2 + 2) = -(n-1)*jy;
    
    %Init input cov
    ii = (i-1)*2 + 1;
    m_cov([ii,ii+1], [ii,ii+1]) = [m(i,3:4); m(i,5:6)];
 end
 
 cov = jf*m_cov*jf';
 xya = [x,y,a];
 
 %Compute cov directly, and compare
 if 0
 cov2 = zeros(2+n);
 
 for i = 0:(n-1)
   Col = jf(:, 2*i + [1:2]);
   k = [m(i+1,3:4); m(i+1,5:6)];
   
   cov2 = cov2 + Col*k*Col';
 end
 
 cov2 - cov %seems to work!!!
 end
