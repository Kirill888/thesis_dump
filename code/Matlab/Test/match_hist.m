function v = match_hist(x,y,a,Paa,Pbb, fun)
%  match_hist(x,y,a,Paa,Pbb, @fun) 
%   
  
  nx = length(x);
  ny = length(y);
  
  v = zeros(ny,nx);
  
  for i = 1:nx
    for j = 1:ny
      v(j,i) = feval(fun,a,Paa,[x(i) y(j)], Pbb);
    end
  end
  
  
