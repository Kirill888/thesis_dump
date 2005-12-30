function inspect_trans(fname, sensorPose)
% function inspect_trans(fname, sensorPose)
%
%

%


run(fname)

odo1 = odo2sensor(odo(:,1:3), sensorPose);
odo2 = odo2sensor(odo(:,4:6), sensorPose);

odo2all = transit_pose(odo1,trans);

trans(:,5:6) = trans(:,5:6) + 1;

fig1 = figure;

plot_robot(odo1,'g');hold on
mplot(m1,'r.','r-'); axis equal

fig2 = figure;
plot_obs2d(obs,odo2,'y','m'); hold on
plot_robot(odo2all, 'g');
plot_robot(odo2,'b');
mplot(m2,'b.','b-');
axis equal




