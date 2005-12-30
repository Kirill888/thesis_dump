function [mapping] = ptrack(xy)
  
  MAXD = 0.1;
  MAXD2 = MAXD*MAXD;

  n = size(xy,1);

  mapping = zeros(n,1);

  nclusters  = 1;
  cluster_xy = xy(1,1:2);
  mapping(1) = 1;

  for i = 2:n
     mini = 0;
     mind2 = 2*MAXD2;

     for j = 1:nclusters
        d2 = dist2(xy(i,1:2), cluster_xy(j,:));

	if d2 < mind2 & d2 < MAXD2
	   mind2 = d2;
	   mini = j;
	end
     end

     if mini > 0  %Mapped to a cluster
        mapping(i) = mini;
        cluster_xy(mini,:) = xy(i,1:2);
     else         %Start new cluster
        nclusters = nclusters+1;

	mapping(i) = nclusters;
	cluster_xy(nclusters,:) = xy(i,1:2);
     end

  end



function d2 = dist2(xy1,xy2)
  
  dd = xy1 - xy2;
  d2 = dd*dd';
