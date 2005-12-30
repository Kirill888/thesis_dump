function [ll]  = backProject(xy)
% function [ll]  = backProject(xy)

global C1 C2

F1 = 737.5912;
F2 = 711.0105;
CX1 = 314.2927;
CX2 = 333.1539;

[u1,h1] = projectToImagePlane(xy,C1);
[u2,h2] = projectToImagePlane(xy,C2);

sxx = xy(:,3);
syy = xy(:,6);
sxy = xy(:,4);

h11 = h1(:,1);
h12 = h1(:,2);
h21 = h2(:,1);
h22 = h2(:,2);

s_uu1 = (h11.*sxx+h12.*sxy).*h11+(h11.*sxy+h12.*syy).*h12;
s_uu2 = (h21.*sxx+h22.*sxy).*h21+(h21.*sxy+h22.*syy).*h22;
s_u12 = (h11.*sxx+h12.*sxy).*h21+(h11.*sxy+h12.*syy).*h22;

ll = [CX1 - u1*F1, CX2 - u2*F2, s_uu1*F1*F1, s_u12*F1*F2, s_u12*F1*F2, ...
     s_uu2*F2*F2];

function [u,h] = projectToImagePlane(xy,pose)
  ca = cos(pose(3));
  sa = sin(pose(3));
  x  = xy(:,1) - pose(1)/1000;
  y  = xy(:,2) - pose(2)/1000;
  
  tmp =  x.*ca + y.*sa;
    y = -x.*sa + y.*ca;
    x = tmp;
 
  u = y./x;
  h = [-u./x, 1./x];
  
  h = [h(:,1).*ca + h(:,2).*sa, -h(:,1).*sa + h(:,2).*ca];
 


return
%xy = toCameraCoords(xy,C1);
sxx = xy(:,3);
sxy = xy(:,4);
syy = xy(:,6);

l1 = F1.*xy(:,2)./xy(:,1);
l2 = F2.*(xy(:,2)-C2(2))./(xy(:,1)-C2(1));

jb1 = -l1./xy(:,1);
jb2 =  F1./xy(:,1);
jb3 = -l2./(xy(:,1) - C2(1));
jb4 =  F2./(xy(:,1) - C2(1));

sigma = zeros(length(jb1),4);
sigma(:,1) = jb1.*jb1.*sxx+2.*jb1.*jb2.*sxy+jb2.*jb2.*syy;
sigma(:,2) = jb1.*jb3.*sxx+jb1.*jb4.*sxy+jb2.*jb3.*sxy+jb2.*jb4.*syy;
sigma(:,3) = sigma(:,2);
sigma(:,4) = jb3.*jb3.*sxx+2.*jb3.*jb4.*sxy+jb4.*jb4.*syy;

ll = [CX1 - l1, CX2 - l2,sigma];



function xy = toCameraCoords(xy, C)
  xy(:,1) = xy(:,1) - C(1);
  xy(:,2) = xy(:,2) - C(2);
  xy = translate_obs(xy,[0,0,-C(3)]);