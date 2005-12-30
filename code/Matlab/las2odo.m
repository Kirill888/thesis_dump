function odo = las2odo(las_odo, LAS_R)

  if nargin < 2
     LAS_R = 0.263;
  end

  n = size(las_odo,1);
  odo = zeros(n,3);

  a = las_odo(:,3) + pi/2;
  odo(:,1) = las_odo(:,1) - LAS_R*cos(a);
  odo(:,2) = las_odo(:,2) - LAS_R*sin(a);
  odo(:,3) = las_odo(:,3);
