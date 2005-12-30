function h = plot_path(odo, map_ind, map_poses,styles)
%function h = plot_path(odo, map_ind, map_poses, styles)

  nmap = size(map_poses,1);
  
  plot_cov = 0;
  
  if size(odo,2) == 12
    plot_cov = 1;
  end
  
  odo_r = 0.01;
  
  if ~exist('styles','var')
    for i = 1:nmap
      if plot_cov
          styles(i,:) = 'g.g-g-g-g-';
      else
          styles(i,:) = 'g.-';
      end
    end
  end
  
  if nargout > 0
    h = [];
  end
  
  if size(styles,2) == 1 % Only colors
    for i = 1:nmap
      c = styles(i,1);
      if plot_cov
          styles(i,1:10) = [c,'.',c,'-',c,'-',c,'-',c,'-'];
      else
          styles(i,1:3) = [c,'.-'];
      end
    end
  end
  
  
  
  for i = 1:nmap
    ind = find(map_ind == i);
    
    if ~isempty(ind)
      o = translate_odo(odo(ind,:), map_poses(i,1:3));
      
      if plot_cov
	h_ = plot_odo(o,[],styles(i,:),odo_r);
      else
	h_ = plot_robot(o,styles(i,:), odo_r);
      end
      
      if nargout > 0    
         h = [h;h_];
      end
    end
  end
  
  
  
  
