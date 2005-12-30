function T_norm = normalise_points(X);
% normalise_points determines the homogeneous
% transformation matrix T_norm such that
%
% X_norm = T_norm*X
%
% defines an X_norm with a mean of 0 and rms
% (root mean squared) distance from the origin
% of sqrt(2).

numPoints = size(X,2);
x = X(1,:);
y = X(2,:);

centroid_x = sum(x)/numPoints;
centroid_y = sum(y)/numPoints;

x = x - centroid_x;
y = y - centroid_y;

RMS = sqrt(sum(x.*x + y.*y)/numPoints);

T_trans = [1 0 -centroid_x;
           0 1 -centroid_y;
           0 0 1];
       
T_scale = [sqrt(2)/RMS 0 0;
           0 sqrt(2)/RMS 0
           0 0 1];
           
T_norm = T_scale*T_trans;