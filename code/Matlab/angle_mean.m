function amean = angle_mean(a,w)
% function amean = angle_mean(a,w)
%
%

%

if nargin < 2
  w = ones(size(a))./size(a,1);
end

ca = cos(a); sa = sin(a);

x = sum(ca.*w);
y = sum(sa.*w);

amean = atan2(y,x);
