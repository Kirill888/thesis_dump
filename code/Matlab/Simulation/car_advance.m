function odo = car_advance(u, odo0, L)
%  function odo = car_advance(u, odo0[, L = 3])
%
%    u  = [T Vc Steer]
%  odo0 = [x,y,a]
%    L -- Distance between rear and front wheels

%

  if nargin < 3
    L = 3;
  end
  
  
  DT = diff(u(:,1));
  n  = size(u,1) - 1; % the last control is not applied

  odo = zeros(n+1,3);
  odo(1,:) = odo0;

  x = odo(1,1); y = odo(1,2); a = odo(1,3);
  
  for i = 1:n
    Vc = u(i,2);
    steer = u(i,3);
    
    x = x + DT(i)*Vc*cos(a);
    y = y + DT(i)*Vc*sin(a);
    a = a + DT(i)*Vc*tan(steer)/L;


    odo(i+1,:) = [x,y,a];
  end



