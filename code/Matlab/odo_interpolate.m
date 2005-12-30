function odo = odo_interpolate(odo0, t0, t)
% function odo = odo_interpolate(odo0, t0, t)
%
%

%

ii = find(diff(t0) > 0.0001);

x = interp1(t0(ii), odo0(ii,1), t);
y = interp1(t0(ii), odo0(ii,2), t);
a = interp1(t0(ii), odo0(ii,3), t);

odo = [x,y,a];
