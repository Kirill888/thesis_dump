function [profile] = obs_profile(landmark, odo, obs, do_plot)
% function profile = obs_profile(landmark, odo, obs, do_plot)
%

%

nodo = size(odo,1);
dx = landmark(1) - odo(:,1);
dy = landmark(2) - odo(:,2);

range = sqrt(dx.*dx + dy.*dy);
bearing = angle_diff(atan2(dy,dx), odo(:,3));

left_a  = pi/2 - 5*pi/180;
right_a = - left_a;

ii = find(bearing > right_a & bearing < left_a & range < 8.1);

was_observed = zeros(length(ii),1);

for i = 1:length(ii)
  scan = ii(i);
  i_obs = find(obs(:,1) == scan);
  
  if ~ isempty(i_obs)
    for j = 1:length(i_obs)
      o = obs(i_obs(j),2:end);
      
      dr = range(scan) - o(1);
      da = angle_diff(bearing(scan), o(2));
      
      md2 = dr*dr/o(3) + da*da/o(6);
      
      if md2 < 16
	was_observed(i) = 1;
      end
    end
  end
end


odo = odo(ii,1:3);
range = range(ii);
bearing = bearing(ii);
ii = find(was_observed);

i_range = 0.1:0.2:8;
r1 = hist(range,i_range);
r2 = hist(range(ii), i_range);

ii_nonzero = find(r1 ~= 0);
profile = zeros(size(r2));
profile(ii_nonzero) = r2(ii_nonzero)./r1(ii_nonzero);


if do_plot
  figure(1);
  clf
  subplot(2,1,1);
  plot_robot(odo,'r'); hold on; axis equal;
  plot_robot(odo(ii,:), 'b');
  plot(landmark(1), landmark(2), 'rs');

  subplot(2,2,3);
  polar(bearing, range, 'r.');
  hold on;
  polar(bearing(ii), range(ii), 'b.');

  subplot(2,2,4);

  bar(i_range, r1, 'b');hold on
  bar(i_range, r2, 'r');
end
