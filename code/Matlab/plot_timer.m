function plot_timer(timer, name, range)

if nargin < 3
  m = mean(timer);
  s = std(timer);
  
  range(1) = max(0,m - 1*s);
  range(2) = m + 1*s;

end

  range = range*1000; %convert to ms
  
  nbins = 100;
  timer = timer*1000; %Convert to ms
  
  vmin = min(timer) - 0.1;
  vmax = max(timer) + 0.1;

  %ind1 = find(timer < range(2));
  %ind2 = find(timer > range(1));
  %ind  = intersect(ind1,ind2);

  edges = linspace(range(1),range(2), nbins);

  include_all = 0;
  if vmin < range(1) & include_all
    edges = [vmin, edges];
  end
  
  if vmax > range(2) & include_all
    edges = [edges vmax];
  end
  
  
%  edges'
  
  N = histc(timer,edges);
  N = N/sum(N)*100;
  
  barh(edges,N,'histc'); 
 
  hold on

  %plot(0,vmax,'ro');
  ylabel('ms');
  xlabel('%');
  
  if nargin > 1
    title(name);
  end
  
