function xyr = align_maps(m1,m2,x0)
%function xyr = align_maps(m1,m2,x0)
%

%
  opts = optimset( 'MaxFunEvals',1000 ...
		  ,'TolCon'     ,0.1  ...
                  ,'TolFun'     ,0.1  ...
                  ,'TolX'       ,0.1 ...
		  );

  xyr = fminsearch(@aux1,x0,opts,m1,m2);
  
function w = aux1(x, m1, m2)

  w = weight(m1,x,m2);
  
  w = -sum(w);
  
 % disp(sprintf('[%.2f %.2f %.2f] %.10f',x(1),x(2), x(3)*180/pi, w));
    
function w = weight(map1, odo, map2)

  map2 = translate_obs(map2,odo);
  nmap = size(map1,1);
  w = 0;
  
  for i = 1:nmap
    w = w + log_volume(gauss2a(map1(i,:)), gauss2a(map2(i,:)));
  end
  
  w = exp(w);

function lv = log_volume(a1, a2)
  a = a1 + a2;

  lv = 2*log(pi)*(a(6)-a(4)*a(4)/4/a(1) + ...
		  (a(5)-a(3)*a(4)/2/a(1))^2 ...
	         /(a(3)*a(3)/a(1) - 4*a(2))) ...
      /log(-a(1)*(a(3)*a(3)/4/a(1) - a(2)));
