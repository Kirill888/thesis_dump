function [obs] = obs3d(obs0,odo0)
%function h = plot_obs3d(obs,odo)

if 0
  odo = odo0(obs0(:,13),:);
else
  odo = odo0;
end


[x,y,z] = polar2cartesian(obs0(:,1:3), odo);

%h = plot3(x,y,z,'r.');

obs = [x,y,z];


function [x,y,z] = polar2cartesian(obs0,odo)
  R = obs0(:,1);
  a = obs0(:,2) + odo(:,3);
  b = obs0(:,3);
  
  sb = sin(b);
  cb = cos(b);
  sa = sin(a);
  ca = cos(a);
  
  x = odo(:,1) + R.*cb.*ca;
  y = odo(:,2) + R.*cb.*sa;
  z = R.*sb;