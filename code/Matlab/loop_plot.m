function loop_plot(in_file, map,links)
%function loop_plot(in_file,[map, links])

  if nargin < 2
    map = [];
  end
  
  if nargin < 3
    links = [];
  end
  
  
  n = size(in_file,1);
  if n == 1
    aux1(in_file,map,links);
    return
  end
  
  for i = 1:size(in_file,1)
    figure;
    if isfield(in_file,'name')

      ff = in_file(i).name;
      % Get rid of .m extension.
      ind_dot = find(ff == '.');
      if ~isempty(ind_dot)
	ff = ff(1:ind_dot(1)-1);
      end

      aux1(ff,map,links);
    else
      aux1(in_file(i,:),map,links);
    end
  end
  


function aux1(in_file,map, links)
  
  run(in_file);
  
  h =  plot(fp(:,1),fp(:,2),'y.');
  set(h,'MarkerSize',1);
  hold on

  
  if ~isempty(map)
    best_path = best_path(length(best_path):-1:1);
    aux2(best_path, imap2, imap1, map,links);
  end

  %ind = find(fp(:,4) > log(0.1));
  ind  = unique(smple+1);
  plot_robot(fp(ind,1:3),'r.-',0.1);

  %  h = plot(fp(ind,1),fp(ind,2),'r.');
  %  set(h,'MarkerSize',3);
  
  h = plot_odo(r1in2,[],'bsb-b-b-',1);
  set(h,'LineWidth',2);
  
  h = plot_robot([0 0 0],'go-',0.7);
  set(h,'LineWidth',3);
    
  mplot(map_a1,'r.','r:');
  mplot(map_a2,'g.','g:');
  
  %mplot(odo_a2,'ms','m:');
  %mplot(odo_a1,'bs','b:');  
  
  [v,i] = max(fp(:,4)); 
  bestP = fp(i,1:3);
  maxW  = fp(i,4);
  
  map1 = translate_obs(map1, bestP);
  
  mplot(map1,'ro','r-');
  mplot(map2,'go','g-');
  
  for i = 1:size(map1,1)
   line( [ map1(i,1) , map2(i,1)] ...
	 ,[ map1(i,2) , map2(i,2)]);
  end
  
  h = plot_robot(bestP,'ro-',0.7);
  set(h,'LineWidth',3);
  
  axis equal
  
  title(sprintf('Map Matching %02d => %02d (%d/%d ~ %.2f%%, W_{max}=%e)' ...
		,imap1, imap2, length(ind), size(fp,1)...
		,100*length(ind)/size(fp,1),maxW));

  
  
function aux2(path, i1, i2, map, links)
  
  n = length(path)
  
  gpose = zeros(n+1,3);
  
  for i = 1:n
    disp(sprintf('Step %d: %d',i, path(i)+1));
    gpose(i+1,:) = translate_odo(map(i + i1 - 1).odo_f(path(i)+1,:) ...
				 ,gpose(i,:));
    
  end
  
  if ~isempty(links)
    odo = zeros(1,3);
    aux3(odo,map(i1),{'c.c:','','c.-'});
    
    for i = i1+1:i2
      odo = translate_odo(links(i-1,1:3),odo);
      aux3(odo,map(i),{'c.c:','','c.-'});
    end
  end
  
  for i = 1:n+1
    aux3(gpose(i,:),map(i+i1-1),{'m.m-','','m.-'});
  end
  
  
  h = plot_robot(gpose,'bs-',0.3);
  set(h,'LineWidth',2);
  
function aux3(gpose, map, style)
  map = map_translate(map,gpose);
  plot_map(map,style{1},style{2},style{3},0.1);
  







