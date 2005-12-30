function poses = map_optimise(links, map_poses0)
% function poses = map_optimise(links, map_poses0)

  pose0 = map_poses0(1,1:3);
  
  npose = size(map_poses0,1);
  
  ptmp = map_poses0(2:npose,1:3)';
  
  x0 = ptmp(:);
  
  opts = optimset( 'MaxFunEvals',1000 ...
		  ,'TolCon'     ,0.1  ...
                  ,'TolFun'     ,0.1  ...
                  ,'TolX'       ,0.01 ...
		  );
  x = fminsearch(@aux1,x0,opts,links,pose0);
  nx = length(x);
  poses = zeros(3,nx/3+1);
 
  for i = 1:nx
    poses(i+3) = x(i);
  end
 
  poses = poses'
  poses(1,:) = pose0;
 

function w = aux1(x, links, pose0)
%this function will be called by fminsearch

 nx = length(x);
 
 poses = zeros(3,nx/3+1);
 
 for i = 1:nx
   poses(i+3) = x(i);
 end
 
 poses = poses';
% pose0
 poses(1,:) = pose0;
 
% poses
 w = map_weight(poses,links);
 w = sum(w);
 
 
