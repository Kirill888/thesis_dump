function [odo,obs] = add_noise(odo0,obs0)
   odo = odo0;
   
   V = odo(:,2);
   steer = odo(:,3);
   sigma_s = (5*pi/180)/3;
   
   ii = find(V ~= 0);
   sigma_v = (0.1*V(ii)+ 0.1)/3;
   noise = randn(size(V(ii)));
   V(ii) = V(ii) + noise.*sigma_v;
   
   noise = randn(size(steer));
   steer = steer + noise.*sigma_s;
   
   odo(:,2) = V;
   odo(:,3) = steer;

   if nargin > 1 && nargout > 1
     obs = obs0;
     r = obs0(:,3);
     a = obs0(:,4);
     d = obs0(:,5);
     
     sigma_r = (0.05*r + 0.5)/2;
     sigma_a = (1*pi/180)/2;
     sigma_d = (0.1)/2;
     
     r = r + randn(size(r)).*sigma_r;
     a = a + randn(size(a)).*sigma_a;
     d = d + randn(size(d)).*sigma_d;
     
     obs(:,3) = r;
     obs(:,4) = a;
     obs(:,5) = d;
   end
   