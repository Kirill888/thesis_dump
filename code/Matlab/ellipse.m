function [x,y] = ellipse(X0, P)
%function [x,y] = ellipse(X0, P)
NPOINTS = 20;

n = size(X0,1);

x = zeros(n,NPOINTS+1);
y = zeros(n,NPOINTS+1);
r = 3 ;        %circle's radius in space X2


for i = 1:n
  if P(:,:,i) == zeros(2,2);
    %Leave x(i,:) y(i,:) as zeros, do nothing
  else
    [R,p] = chol(P(:,:,i));  % R*R'= P, X = R*X2
    if p == 0
      R = R';

      aaa = [0:NPOINTS]*2*pi/NPOINTS ; % sample angles
      xxx = [ r*cos(aaa) ; r*sin(aaa) ] ; % polar to x2,y2
      xxx = R*xxx ;

      x(i,:) = X0(i,1) + xxx(1,:);
      y(i,:) = X0(i,2) + xxx(2,:);
    else
      disp('warning: bad covariance.');
      P(:,:,i)
    end
  end
end

return;






























%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2002
%-------------------------------------------------------
% function h = draw_ellipse(pos, cov, color)
%
% draws an ellipse, centered at pos, corresponding to
% the confidence region determined by cov, given chi2(2)
%-------------------------------------------------------
%CHI2_2 = 5.99146455;
%
%persistent CIRCLE
%
%if isempty(CIRCLE)
%    tita = linspace(0, 2*pi,20);
%    CIRCLE = [cos(tita); sin(tita)];
%end
%
%[V,D]=eig(full(P(1:2,1:2)));
%ejes=sqrt(CHI2_2*diag(D));
%xy = (V*diag(ejes))*CIRCLE;
%x = xy(1,:) + pos(1);
%y = xy(2,:) + pos(2);

