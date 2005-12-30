function [style,htext] = plot_map_ref(mm, map_ref, style, r)
%  function style = plot_map_ref(mm, map_ref, style)
%  style = ['r+','gx','cd','b*','ms' ... nmap]
  
  plot_map_uncertainty = 0;
  plot_convex = 0;
  plot_grid = 1;
  
  nmap = length(mm);
  
  if nargin < 3
    default_style = ['r+';'gx';'cd';'b*';'ms'];
    ndefault = 5;
    
    for i = 1:nmap
      style(i,1:2) = default_style(mod(i-1,ndefault)+1,1:2);
    end
  end
  
  if nargin < 4
    r = 0.1;
  end

  if plot_grid
    hold on
    for i = 1:nmap
      plotGrid(mm(i).grid, map_ref(i,:), style(i,1));
    end
  end
    
  
  
  for i = 1: nmap
    map = map_propagate(mm(i),map_ref(i,:));
    
    if size(map.map,2) == 14
      disp('Corner Map');
      map.map(:,8:9) = map.map(:,8:9) + map_ref(i,3);
    end
    

    htext(i) = text(map_ref(i,1), map_ref(i,2), ...
	     sprintf('%02d',i));
    
    set(htext(i),'FontSize',15);
    set(htext(i),'Color',style(i,1));
  
    style_map = [style(i,[1,2,1]), '-'];
    style_odo = style(i,1);
    
    plot_map(map,style_map,'',style_odo,r);
    
    if plot_map_uncertainty
      h = plot_odo(map_ref(i,:),[],'b.b-b-b-',0.5);
      set(h,'LineWidth',3); 
    end
    
    if plot_convex
      h = plotConvex(map.map);
      set(h,'Color',style(i,1));
    end
    
    
  end

  
function h = plotGrid(regGrid, odo0, c)
  if isempty(regGrid.data)
    h = [];
    return
  end
  
  dim = regGrid.dim;
  xx = linspace(dim(1), dim(3), size(regGrid.data,2));
  yy = linspace(dim(2), dim(4), size(regGrid.data,1));
  [yi,xi] = find(regGrid.data > 0);
  xx = xx(xi)';  yy = yy(yi)';
  
  if ~isempty(odo0) % Translate points
    xy = translate_obs([xx,yy],odo0);
    xx = xy(:,1);
    yy = xy(:,2);
  end
  
  h = plot(xx,yy,[c '.']);
  ccc = get(h,'Color');
  ccc = rgb2hsv(ccc);
  ccc(2) = ccc(2)*0.3;
  ccc = hsv2rgb(ccc); 
  set(h,'Color',ccc, 'MarkerSize',10);


function h  = plotConvex(map)
  ind_core = find(map(:,size(map,2)) == 3);

  if length(ind_core) > 2
    k = convhull(map(ind_core,1),map(ind_core,2));
    k = ind_core(k);
    h = line(map(k,1), map(k,2));
  else
    h = [];
  end
