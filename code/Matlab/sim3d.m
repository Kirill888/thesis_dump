function [obs0, obs] = sim3d(odo, map)
%function obs = sim3d(odo)
  nodo = size(odo,1);
  nmap = size(map,1);
  
  obs = zeros(nodo*nmap,3);
  obs0 = zeros(nodo*nmap,5);
  obs0(:,2) = nmap;

  Sr = 0.1/3;
  Sa = 5*pi/180/3;
  Sb = 5*pi/180/3;
  
  for i = 1:nodo
    ind = ((i-1)*nmap+1):1:i*nmap;
    obs0(ind,3:5) = get_obs(map,odo(i,2:4)) + get_obs_noise(Sr,Sa,Sb,nmap);
    
    obs0(ind,1) = odo(i,1)*ones(nmap,1);
    
    obs(ind,:)  = obs_to_x(obs0(ind,3:5),odo(i,2:4));
  end
  

function z = get_obs(x,robot);
  dx = x(:,1) - robot(1);
  dy = x(:,2) - robot(2);
  dz = x(:,3);
  
  z(:,1) = sqrt(dx.*dx + dy.*dy + dz.*dz);
  z(:,2) = angle_diff(atan2(dy,dx),robot(3));
  z(:,3) = asin(dz./z(:,1));
  
function z_noise = get_obs_noise(Sr,Sa,Sb, n)
  z_noise = zeros(n,3);

  z_noise(:,1) = randn(n,1)*Sr;
  z_noise(:,2) = randn(n,1)*Sa;
  z_noise(:,3) = randn(n,1)*Sb;

  
function [x] = obs_to_x(z, robot)
  a = z(:,2) + robot(3);
  b = z(:,3);
  ca = cos(a);
  sa = sin(a);
  cb = cos(b);
  sb = sin(b);
  R  = z(:,1);

  x(:,1) = robot(1) + R.*cb.*ca;
  x(:,2) = robot(2) + R.*cb.*sa;
  x(:,3) =            R.*sb;



