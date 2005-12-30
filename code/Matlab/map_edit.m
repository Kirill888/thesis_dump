function [pp,style,ht] = map_edit(gmap,pp,style)
% function [pp,style,ht] = map_edit(gmap,pp,style)
%
%
%

%


fig=figure; 

if nargin < 2
  pp = gmap.map_poses(:,1:3);
end

if nargin < 3
  [style,ht] = plot_map_ref(gmap.maps,pp);
else
  [style,ht] = plot_map_ref(gmap.maps,pp,style);
end


axis equal;

nmaps = size(gmap.maps,2);

 

if nargin < 2

  pp = gmap.map_poses(:,1:3);

for i = 1: nmaps
  disp(sprintf('Update pose of map %d',i));

  figure(fig);
  xy = ginput(1);
  if isempty(xy)
    disp('Not moving the map.');
  else
    pp(i,1:2) = xy;
    cla
    if nargin < 3
      [style,ht] = plot_map_ref(gmap.maps,pp);
    else
      [style,ht] = plot_map_ref(gmap.maps,pp,style);
    end
  end
end

end

i = input('Correct pose of map:');



while ~isempty(i)

  figure(fig);
  xy = ginput(1);
  if isempty(xy)
    disp('Not moving the map.');
  else
    pp(i,1:2) = xy;
    cla
    if nargin < 3
      [style,ht] = plot_map_ref(gmap.maps,pp);
    else
      [style,ht] = plot_map_ref(gmap.maps,pp,style);
    end
  end

  i = input('Correct pose of map:');
end


disp('Place topological nodes');
tp = zeros(nmaps,2);

for i = 1: nmaps
  disp(sprintf('Update label %d',i));

  figure(fig);
  xy = ginput(1);
  set(ht(i),'Position',[xy,0]);
  tp(i,:) = xy;

end

h = plot_topology(tp,gmap);
set(h,'Marker','o');

h = plot_coord(pp,7);
set(h,'Color','k','LineWidth',2);

axis tight
xlabel('meters'); ylabel('meters');
 
function h = plot_topology(pp,gmap)
% function h = plot_topology(pp, gmap)
%
%
%
%

%

n = size(pp,2);

links = gmap.links(:,1:2);

x1 = pp(links(:,1),1);
x2 = pp(links(:,2),1);
y1 = pp(links(:,1),2);
y2 = pp(links(:,2),2);

h = line([x1';x2'],[y1';y2'],'LineWidth',3,'Color','b');
