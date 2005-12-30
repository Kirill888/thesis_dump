function [x,Sxx,odo,z_all] = tst_kalman3d(x,Sxx,z,Szz,robot)
%function [x,Sxx,odo,z_all] = tst_kalman3d(x,Sxx,z,Szz,robot)
%

%

if nargin == 0
  [x,Sxx,odo,z_all] = run_test;
  return
end

  dx = x(1) - robot(1);
  dy = x(2) - robot(2);
  dz = x(3);
  
  r2 = dx*dx + dy*dy;
  R2 = r2 + dz*dz;
  R = sqrt(R2);
  R_inv  = 1/R;
  r2_inv = 1/r2;
  r  = sqrt(r2);
  R2r_inv = 1/(R2*r);
  
  
  H = [  dx*R_inv     ,  dy*R_inv     , dz*R_inv  ;
	-dy*r2_inv    ,  dx*r2_inv    , 0;
	-dz*dx*R2r_inv, -dz*dy*R2r_inv, r2*R2r_inv]; 
  
  Ht = H';
  
  K = Sxx*Ht*inv(H*Sxx*Ht + Szz);
  
  DZ(1,1) = z(1) - R;
  DZ(2,1) = angle_diff(z(2), atan2(dy,dx) - robot(3));
  DZ(3,1) = angle_diff(z(3), asin(dz*R_inv));
  
  x = x + K*DZ;
  Sxx = (eye(3) - K*H)*Sxx;
  
  md2 = DZ'*inv(Szz)*DZ;
%disp(sprintf('dr = %+.3f, da = %+.2f, md2 = %+.3f',dz(1),dz(2)*180/pi, md2));
    
function [x,Sxx,odo,z_all] = run_test

  landmark = [3, -2, 1];
  
  Sr = 0.1/3;
  Sa = 5*pi/180/3;
  Sb = 5*pi/180/3;
  
  Szz = [Sr*Sr 0   0; 
	 0   Sa*Sa 0;
	 0     0   Sb*Sb];
  
  disp(sprintf('double Srr = %.8e; double Saa = %.8e; double Sbb = %.8e;' ...
	       ,Sr*Sr,Sa*Sa,Sb*Sb));
  
  rx = 0:0.1:4;
  ry = 0.5*sin(2*rx);
  ra = atan(cos(2*rx));

  robot = [rx', ry', ra'];
  
  z0 = get_obs(landmark, robot(1,:)) + get_obs_noise(Sr,Sa,Sb);
  [x,Sxx] = obs_to_x(z0,Szz,robot(1,:));

  z_all = z0';
  
  figure
  hold on
  mplot([x(1:2)' Sxx(1,1:2) Sxx(2,1:2)],'ro','r-');
  axis equal

  
  plot(rx,ry,'.-',landmark(1),landmark(2),'rs');
  
  for i = 2:length(rx);
    z = get_obs(landmark,robot(i,:)) + get_obs_noise(Sr,Sa,Sb);
    [x,Sxx] = tst_kalman3d(x,Sxx,z,Szz,robot(i,:));
  
    z_all = [z_all; z'];
    
    %Plot the observation
    x_ = obs_to_x(z,[],robot(i,:));
    plot(x_(1),x_(2),'g+',x(1),x(2),'b.');
  
    mplot([x(1:2)' Sxx(1,1:2) Sxx(2,1:2)],'b.','b:');
   
  end
  
  mplot([x(1:2)' Sxx(1,1:2) Sxx(2,1:2)],'bs','b-');
  odo = robot;
  
function z = get_obs(x,robot);
  dx = x(1) - robot(1);
  dy = x(2) - robot(2);
  dz = x(3);
  
  z(1,1) = sqrt(dx*dx + dy*dy + dz*dz);
  z(2,1) = angle_diff(atan2(dy,dx),robot(3));
  z(3,1) = asin(dz/z(1,1));
  
function z_noise = get_obs_noise(Sr,Sa,Sb)
  z_noise(1,1) = randn(1)*Sr;
  z_noise(2,1) = randn(1)*Sa;
  z_noise(3,1) = randn(1)*Sb;
  
function [x,Sxx] = obs_to_x(z,Szz,robot)
  a = z(2) + robot(3);
  b = z(3);
  ca = cos(a);
  sa = sin(a);
  cb = cos(b);
  sb = sin(b);
  R  = z(1);

  x(1,1) = robot(1) + R*cb*ca;
  x(2,1) = robot(2) + R*cb*sa;
  x(3,1) =            R*sb;
  

 %[    cos(b)*cos(a), -R*cos(b)*sin(a), -R*sin(b)*cos(a)]
 %[    cos(b)*sin(a),  R*cos(b)*cos(a), -R*sin(b)*sin(a)]
 %[           sin(b),                0,         R*cos(b)]
 
  if nargout > 1
     F = [cb*ca, -R*cb*sa, -R*sb*ca;
	  cb*sa,  R*cb*ca, -R*sb*sa;
	  sb   ,    0    ,  R*cb];
     Sxx = F*Szz*F';
  end
  
  
  