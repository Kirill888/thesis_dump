function [x,Sxx] = tst_kalman(x,Sxx,z,Szz,robot)

if nargin == 0
  [x,Sxx] = run_test;
  return
end

if 0
disp(...
    sprintf('range=%+.7e; bearing=%+.7e;\nrobot.set(%+.7e, %+.7e, %+.7e);', ... 
	 z(1),z(2),robot(1),robot(2),robot(3)));
     disp('ekf_update(&x,range,bearing,Szz,&robot);');
     
end

  dx = x(1) - robot(1);
  dy = x(2) - robot(2);
  
  r2 = dx*dx + dy*dy;
  r  = sqrt(r2);
  
  H = [  dx/r  , dy/r;
	-dy/r2 , dx/r2 ];
  
  Ht = H';
  
  K = Sxx*Ht*inv(H*Sxx*Ht + Szz);
  
  dz(1,1) = z(1) - r;
  dz(2,1) = angle_diff(z(2), atan2(dy,dx) - robot(3));
  
  x = x + K*dz;
  Sxx = (eye(2) - K*H)*Sxx;
  
  md2 = dz'*inv(Szz)*dz;
  disp(sprintf('dr = %+.3f, da = %+.2f, md2 = %+.3f',dz(1),dz(2)*180/pi, md2));
    
function [x,Sxx] = run_test

  landmark = [3, -1];
  
  Sr = 0.1/3;
  Sa = 5*pi/180/3;
  Szz = [Sr*Sr 0; 0 Sa*Sa];
  
  disp(sprintf('double Srr = %.8e; double Saa = %.8e;',Sr*Sr,Sa*Sa));
  rx = 0:0.1:4;
  ry = 0.5*sin(2*rx);
  ra = atan(cos(2*rx));

  robot = [rx', ry', ra'];
  
  z0 = get_obs(landmark, robot(1,:)) + get_obs_noise(Sr,Sa);
  x = obs_to_x(z0,robot(1,:));

  if 0
    disp('  RobotPose robot; double range, bearing;');
disp(...
    sprintf('range=%+.7e; bearing=%+.7e;\nrobot.set(%+.7e, %+.7e, %+.7e);', ... 
	 z0(1),z0(2),robot(1,1),robot(1,2),robot(1,3)));
     disp('//Init observation');
     disp('  Landmark2dObs obs0(range, Srr, bearing, Saa);');
     disp('  Gaussian2d x(obs0.toCartesian(&robot));');
end

  
  
  ca = cos(z0(2) + robot(3));
  sa = sin(z0(2) + robot(3));
  T  = [ca, -z0(1)*sa;
	sa,  z0(1)*ca];
  
  Sxx = T*Szz*T';
  
  figure
  hold on
  mplot([x' Sxx(1,1:2) Sxx(2,1:2)],'ro','r-');
  axis equal

  
  plot(rx,ry,'.-',landmark(1),landmark(2),'rs');
  
  for i = 2:length(rx);
    z = get_obs(landmark,robot(i,:)) + get_obs_noise(Sr,Sa);
    [x,Sxx] = tst_kalman(x,Sxx,z,Szz,robot(i,:));
    
    %Plot the observation
    x_ = obs_to_x(z,robot(i,:));
    plot(x_(1),x_(2),'g+',x(1),x(2),'b.');
  
    mplot([x' Sxx(1,1:2) Sxx(2,1:2)],'b.','b:');

    
  end
  
  mplot([x' Sxx(1,1:2) Sxx(2,1:2)],'bs','b-');
  
function z = get_obs(x,robot);
  dx = x(1) - robot(1);
  dy = x(2) - robot(2);
  
  z(1,1) = sqrt(dx*dx + dy*dy);
  z(2,1) = angle_diff(atan2(dy,dx),robot(3));
  
function z_noise = get_obs_noise(Sr,Sa)
  z_noise(1,1) = randn(1)*Sr;
  z_noise(2,1) = randn(1)*Sa;
  
function x = obs_to_x(z,robot)
  x(1,1) = robot(1) + z(1)*cos(z(2) + robot(3));
  x(2,1) = robot(2) + z(1)*sin(z(2) + robot(3));
  
  