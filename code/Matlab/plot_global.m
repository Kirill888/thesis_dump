function plot_global(maps, links, grid)
%  function plot_global(maps, links, grid)

  if nargin < 3
    grid = 0;
  end
  
  nmaps = length(maps);

  trans_table = [zeros(1,12); links(:,:)];
  global_pose = trans_table;
  
  for i = 1:(nmaps-1)
    trans_table(i+1,:) = translate_odo(links(i,:), ...
				       trans_table(i,:));
    global_pose(i+1,:) = propogate_odo(links(i,:), ...
				       global_pose(i,:));
    
% $$$     cov = [global_pose(i+1,4:6);
% $$$ 	   global_pose(i+1,7:9);
% $$$ 	   global_pose(i+1,10:12)];
% $$$     
% $$$     eigs(cov)';
% $$$     eigs(cov(1:2,1:2))';
% $$$     d(i+1) = det(cov);
  end

%  plot(d,'o-');
%  figure
  
  if grid
  
    for i = 1:nmaps
      if mod(i - 1, 9) == 0
	figure;
      end
      
      plot_ind = mod(i - 1, 9) + 1;

      subplot(3,3, plot_ind);
      plot_map(maps(i), 'r.r-','g.g:','bs-',0.4);
      
      [mmg,og] = map2gauss(maps(i));
      if ~isempty(mmg)
        mplot(mmg,'bs','b-');
      end
      
      mplot(og,'go','g-.');
      
      axis equal
      title(sprintf('Map #%03d',i))
    end

    % Plot final poses as well
    if grid > 1
            
      for i = 1:nmaps
      	if mod(i - 1, 9) == 0
	  figure;
	end
     
	
	plot_ind = mod(i - 1, 9) + 1;
	subplot(3,3, plot_ind);

	plot_robot(maps(i).odo_f, 'r.-',0.01);
	hold on
	
	[v,ind] = max(maps(i).w_f);
	h = plot_robot(maps(i).odo_f(ind,:),'m*-',0.02);
	set(h,'MarkerSize',10);
	set(h,'LineWidth',3);
	
	h = plot_robot(links(i,1:3),'bs-',0.02);
	set(h,'MarkerSize',10);
	set(h,'LineWidth',3);
	
	%xym = links(i,[1:2 4:6 8:9]);
	%mplot(xym,'b.','b:');
	plot_odo(links(i,:),[],'m*m-m:m:',0.1);

	
	axis equal
	title(sprintf(['Map #%03d, final pose\n' ...
		       '3-sigma: x = %.1f mm,y = %.1f mm,a = %.3f deg'],i ...
		      ,3*sqrt(links(i,4))*1000 ...
		      ,3*sqrt(links(i,8))*1000 ...
		      ,3*sqrt(links(i,12))*180/pi));
      end
    
    end
    
  else %PLOT all in global frame
   do_plot_area = 0;

    hold on
    for i = 1:nmaps
      m = map_translate(maps(i), trans_table(i,:));
      
      s  = int2str(ones(size(m.map,1),1)*i);
      s  = [s num2str((1:size(m.map,1))',',%02d')];
      ht = text(m.map(:,1)+0.1, m.map(:,2)-0.1, s);

      if do_plot_area
	[mg,og] = map2gauss(maps(i));
	mg0= translate_obs(mg,trans_table(i,:));
	mg = propogate_obs(mg,global_pose(i,:));
	
	og0= translate_obs(og,trans_table(i,:));
	og = propogate_obs(og,global_pose(i,:));
      end
      
      
%      mmm = propogate_obs(maps(i).map,global_pose(i,:));
%      mplot(mmm,'gx','g-');
      
      if mod(i,2) == 1
	plot_map(m, 'r.r-','','r.-',0.1);
	if do_plot_area
	  mplot(mg,'rs','r-');
	  mplot(og,'g+','g-.');
	  mplot(og0,'g.','g:');
	  %	mplot(mg0,'rx','r:');	
	end
	set(ht,'Color','r');
      else
	plot_map(m, 'b.b-','','b.-',0.1);
        if do_plot_area
	  mplot(mg,'bo','b:');	
	  mplot(og,'c+','c-.');
	  mplot(og0,'c.','c:');
	  %	mplot(mg0,'bx','b:');	
	end
	
	set(ht,'Color','b');
      end
      
      h = plot_odo(trans_table(i,:),[],'ror-r-m-',0.2);
      set(h,'LineWidth',3);

      h = plot_odo(global_pose(i,:),[],'bob-b-c-',0.3);
      set(h,'LineWidth',3);
    end
  end








