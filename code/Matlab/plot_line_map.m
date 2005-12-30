function h = plot_line_map(map, odo0)
% function h = plot_line_map(map, odo0)
%

%

if nargin > 1
  map = translate_line_map(map,odo0);
end

h = line(map(:,[1,3])',map(:,[2,4])');

function m = translate_line_map(map,odo0)

  p1 = map(:,1:2);
  p2 = map(:,3:4);

% translate
  p1 = translate_obs(p1,[-odo0(1:2),0]);
  p2 = translate_obs(p2,[-odo0(1:2),0]);
  
% rotate
  p1 = translate_obs(p1,[0,0,-odo0(3)]);
  p2 = translate_obs(p2,[0,0,-odo0(3)]);



  
  m = [p1,p2];
