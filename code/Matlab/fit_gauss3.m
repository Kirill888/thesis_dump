function [x,Sxx] = fit_gauss3(xx,w)
% function [x,Sxx] = fit_gauss3(xx,w)
%
%

if nargin > 1
  w = w./sum(w); %Normalise
else
  np = size(xx,1);
  w = ones(np,1)/np;
end



x = sum(xx(:,1:2).*[w,w]); % Weighted mean
x(3) = angle_mean(xx(:,3), w);


dx = [xx(:,1) - x(1), xx(:,2) - x(2), angle_diff(xx(:,3), x(3))];

Sxx(1,1) = sum(dx(:,1).*dx(:,1).*w);
Sxx(1,2) = sum(dx(:,1).*dx(:,2).*w);
Sxx(1,3) = sum(dx(:,1).*dx(:,3).*w);

Sxx(2,1) = Sxx(1,2);
Sxx(2,2) = sum(dx(:,2).*dx(:,2).*w);
Sxx(2,3) = sum(dx(:,2).*dx(:,3).*w);

Sxx(3,1) = Sxx(1,3);
Sxx(3,2) = Sxx(2,3);
Sxx(3,3) = sum(dx(:,3).*dx(:,3).*w);

if nargout < 2
  x = [x, Sxx(1,:), Sxx(2,:), Sxx(3,:)];
end


