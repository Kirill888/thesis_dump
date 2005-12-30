function tst5(m1,s1,m2,s2)

  dx = 4*max(s1,s2);
  x = linspace(m1-dx,m1+dx,200);

  y1 = gauss(x,m1,s1);
  y2 = gauss(x,m2,s2);
  
  a1 = gauss2a(m1,s1);
  a2 = gauss2a(m2,s2);
  
  figure
  subplot(2,1,1);
  hold on
  
  plot(x,y1,'r.-', x,y2,'b.-');

  for i = 1:length(x)
    a2 = gauss2a(x(i),s2);
    v(i) = volume(a1+a2);
    
    dd = m1 - x(i);
    v2(i) = dd*dd/s1;
    v3(i) = dd*dd/s2;
    v4(i) = dd*dd/(s1+s2);
    
    v5(i) = log_vol(a1+a2);
    v6(i) = 0.5*dd*dd/(s1*s1+s2*s2);
    v7(i) = log(vol2(m1,s1*s1,x(i),s2*s2));
  end
  
  plot(x,v,'g.-');
  
  %v5 = v5 - max(v5);
  
  subplot(2,1,2);
  plot(x,-log(v),'m.-'    ...
       ,x, sqrt(v2),'g.-' ...
       ,x, sqrt(v3),'r.-' ...
       ,x, sqrt(v4),'b.-' ...
       ,x, -v5,'cs-'      ...
       ,x, v6,'co-'       ...
       ,x, -v7,'go-'      ...
  );


  sum(v)*(x(2)-x(1))
  
function y = gauss(x, m,s)
  
  y = 1/(s*sqrt(2*pi))*exp(-(x - m).*(x - m)/(2*s*s));
  
function a = gauss2a(m,s)
  
  a(1) = 1/(2*s*s);
  a(2) = m/(s*s);
  a(3) = -log(s*sqrt(2*pi)) - m*m/(2*s*s);


function v = volume(a)
  v = sqrt(pi/a(1))*exp(a(2)*a(2)/(4*a(1))   +a(3));
  
function v = log_vol(a)
  v =  - 0.5*log(a(1)) + a(2)*a(2)/(4*a(1)) + a(3) + 0.5*log(pi);
  











