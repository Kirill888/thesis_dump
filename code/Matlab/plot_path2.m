function gpose = plot_path2(links, odo0)
  
  nlinks = size(links,1);

  gpose = [odo0; links(:,:)];
  
  for i = 1:nlinks
    gpose(i+1,:)   = gpose(i,:);

    gpose(i+1,1:2) = gpose(i,1:2) - links(i,1:2);
    
    gpose(i+1,:) = translate_odo(gpose(i+1,:), [0 0, -links(i,3)]);
%    gpose(i+1,1:2) = gpose(i,1:2) - links(i,1:2);
    
    gpose(i+1,1:2) 
    
  end
  
  plot_odo(gpose(nlinks+1,:),[],'rsr-b-b-',0.5);
%  plot_odo(links(nlinks,:),[],'gog-r-r-',0.5);
