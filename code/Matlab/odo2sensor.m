function sensor = odo2sensor(odo, sensor0)
% function odo = odo2sensor(odo, sensor0)
%
%  Input: 
%     odo     -- robot pose (global coords)
%     sensor0 -- pose of the sensor in robot ccordinates
%
%  Output:
%     sensor  -- pose of the sensor in global coords

%

ca = cos(-odo(:,3));
sa = sin(-odo(:,3));

x  = odo(:,1) + sensor0(1).*ca + sensor0(2).*sa;
y  = odo(:,2) - sensor0(1).*sa + sensor0(2).*ca;
a  = odo(:,3) + sensor0(3);

sensor = [x,y,a];


