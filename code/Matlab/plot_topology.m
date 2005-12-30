function h = plot_topology(ht,gmap)
% function h = plot_topology(ht, gmap)
%
%
%
%

%

n = length(ht);

pp = zeros(n,3);

for i = 1:n
  pp(i,:) = get(ht(i),'Position');
end

links = gmap.links(:,1:2);

x1 = pp(links(:,1),1);
x2 = pp(links(:,2),1);
y1 = pp(links(:,1),2);
y2 = pp(links(:,2),2);

h = line([x1';x2'],[y1';y2'],'LineWidth',3,'Color','b');
