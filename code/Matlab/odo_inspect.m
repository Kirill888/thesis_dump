function odo_inspect(odo1, odo2, from, to)
% function odo_inspect(odo1, odo2, from, to)
%
%

%


odo1 = zero_odo(odo1(from:to,1:3));
odo2 = zero_odo(odo2(from:to,1:3));
u1 = odo2u(odo1,0.1);
u2 = odo2u(odo2,0.1);
odo2_ = u2odo(u2,[0,0,0],0.1);

figure
subplot(2,1,1);
plot_robot(odo1,'r');hold on;
plot_robot(odo2,'g');axis equal;
plot_robot(odo2_,'b');

title(sprintf('Path: %04d->%04d',from,to));
xlabel('meters'); ylabel('meters');
ax = axis;
text(ax(1)+0.3, ax(4) - 0.3,'Green - perceived   Red - actual');

d1 = dist_travelled(odo1);
d2 = dist_travelled(odo2);

subplot(4,3,7);
plot(d2, odo1(:,1) - odo2(:,1),'b.-');
xlabel('Distance Travelled');
%xlabel('Odometry Reading');
ylabel('X - Error (meters)');

subplot(4,3,8);
plot(d2, odo1(:,2) - odo2(:,2),'b.-');
xlabel('Distance Travelled');
%xlabel('Odometry Reading');
ylabel('Y - Error (meters)');

subplot(4,3,9);
plot(d2, angle_diff(odo1(:,3),odo2(:,3))*180/pi,'b.-');
xlabel('Distance Travelled');
%xlabel('Odometry Reading');
ylabel('\theta - Error (degrees)');

subplot(4,3,10);hold on
plot(u1(:,1), 'r.-'); plot(u2(:,1),'g.-');
xlabel('Odometry Reading');
ylabel('Vt - (meters/sec)');

subplot(4,3,11);hold on
plot(u1(:,2)*180/pi,'r.-'); plot(u2(:,2)*180/pi,'g.-');
xlabel('Odometry Reading');
ylabel('Vr - (deg/sec)');

subplot(4,3,12);hold on
plot(u1(:,3)*180/pi,'r.-'); plot(u2(:,3)*180/pi,'g.-');
xlabel('Odometry Reading');
ylabel('Direction - (deg)');


function odo = u2odo(u, odo0, dt)
  odo = zeros(size(u,1)+1,3);
  odo(1,:) = odo0;
  
  for i = 1:size(u,1)
    
    Vt  = u(i,1);
    Vr  = u(i,2);
    dir = u(i,3);
    
    if abs(Vr) < 1*pi/180
      Vr = Vr - 0.46*pi/180*Vt;
      dir = dir - 0.46*pi/180*Vt;
    else
      if Vr > 0
	da = Vt*3*pi/180;
	Vr = Vr + da;
	dir = dir + da;
      else
	da = -Vt*3*pi/180;	
	Vr = Vr + da;
	dir = dir + da;
      end
    end
    
    
    x0 = odo(i,1);
    y0 = odo(i,2);
    a0 = odo(i,3);
    
    odo(i+1,1) = x0 + dt*Vt*cos(dir + a0);
    odo(i+1,2) = y0 + dt*Vt*sin(dir + a0);
    odo(i+1,3) = a0 + dt*Vr;
  end
  


function d = dist_travelled(odo)
 dx = diff(odo(:,1));
 dy = diff(odo(:,2));
 
 d2 = dx.*dx + dy.*dy;
 d  = cumsum(sqrt(d2));
 d  = [0;d];

function u = odo2u(odo, dt)
 dx = diff(odo(:,1));
 dy = diff(odo(:,2));
 
 Vt = sqrt(dx.*dx + dy.*dy)./dt;
 da = diff(odo(:,3));
 Vr = da./dt;
 

 dir = angle_diff(atan2(dy,dx), odo(1:end-1,3));
 
 u = [Vt,Vr,dir];
