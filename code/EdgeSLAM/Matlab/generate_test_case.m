function [ll] = generate_test_case(odo,point)
% function generate_test_case(odo,point)

global C2
odo2 = odo2sensor(odo, [C2(1)/1000, C2(2)/1000, C2(3)]);

[x1,y1] = projectPoint(odo , point);
[x2,y2] = projectPoint(odo2, point);


%Compute image coords from that.

F1 = 737.5912;
F2 = 711.0105;
CX1 = 314.2927;
CX2 = 333.1539;

l1 = F1.*y1./x1;
l2 = F2.*y2./x2;

ll = [CX1 - l1, CX2 - l2];

figure;
plot_robot(odo,'g'); hold on; 
plot_robot(odo2,'r');
plot(point(1), point(2), 'bs');
axis equal

function [x,y] = projectPoint(odo, point)
x = point(1) - odo(:,1);
y = point(2) - odo(:,2);


%Compute landmark in robot coords
ca = cos(odo(:,3));
sa = sin(odo(:,3));

tmp = x.*ca + y.*sa;
y  = -x.*sa + y.*ca;
x  =  tmp;
