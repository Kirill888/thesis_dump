function [Vt,Vr,Dir,dt, odo] = tst_odo(odo0)

  n = size(odo0,1);
  dt = diff(odo0(:,1));
  
  X = odo0(:,2);
  Y = odo0(:,3);
  A = odo0(:,4);
  
  dA = diff(A);
  dX = diff(X);
  dY = diff(Y);

  ind = find(dt > 0);
  
  Vr = zeros(n-1,1);
  Vt = Vr; Dir = Vr;
  
  Vr(ind)  = dA(ind)./dt(ind);
  Vt(ind)  = sqrt(dX(ind).*dX(ind) + dY(ind).*dY(ind))./dt(ind);
  Dir(ind) = angle_diff(atan2(dY(ind),dX(ind)),A(ind));
  
  a   = -0.577*pi/180;
  Vr  = Vr + a*Vt;
  %Dir(ind) = Dir(ind) + a*Vt(ind)./dt(ind);

  ind = find(dt == 0);
  Vr(ind) = 0; Vt(ind) = 0; Dir(ind) = 0;
  
  odo = applyControlInput(odo0(1,2:4),Vt,Vr,Dir,dt);
 
function odo = applyControlInput(startPoint,Vt,Vr,Dir,dt)
  n = length(Vt);

  odo = zeros(n+1,3);
  odo(1,:) = startPoint;
  
  da = dt.*Vr;
  
  for i = 1:n
    dx = dt(i).*Vt(i).*cos(Dir(i) + odo(i,3));
    dy = dt(i).*Vt(i).*sin(Dir(i) + odo(i,3));
    odo(i+1,:) = odo(i,:) + [dx,dy,da(i)];
  end
  

