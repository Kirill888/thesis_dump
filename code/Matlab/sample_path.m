function odo = sample_path(gmap,path,nsample,odo0)
% function odo = sample_path(gmap,path,nsample,odo0)
%
%
%
%

%

nhops = length(path);
odo = zeros(nsample,3);

if nargin > 3
  odo(:,1) = odo0(1);
  odo(:,2) = odo0(2);
  odo(:,3) = odo0(3);
end


for i = 1:nhops
 t = gmap.transitions(path(i));
 nt = size(t.odo,1);
 
 ind = floor(rand(1,nsample)*nt)+1;
 ind(find(ind>nt)) = 1;
 
 t_odo = t.odo(ind,:);
 
 ca = cos(odo(:,3));
 sa = sin(odo(:,3));
 x = t_odo(:,1).*ca - t_odo(:,2).*sa + odo(:,1);
 y = t_odo(:,1).*sa + t_odo(:,2).*ca + odo(:,2);
 a = t_odo(:,3) + odo(:,3);
 odo = [x,y,a];

end




return

%OLD CODE, useless
function [odo, ind] = sample_path(maps, n, from, to)
  
  if nargin < 2
    n  = 1;
  end
  
  if nargin < 4
    to = length(maps);
  end
  
  if nargin < 3
    from = 1;
  end
  
  odo = zeros(n, 3*(to - from));
  ind = zeros(to - from, n);
  
  %odo(:,1:3) is set to zero.
  %All particles are set to zero.
  
  for i = from:to-1
    k = sample(maps(i).w_cumSum,n);
    i0 = i - from +1;
    
    ind(i0,:) = k';
    
    i_odo = (3*i0+1):(3*i0+3);
    
    for j = 1:n
      odo(j,i_odo) = translate_odo(maps(i).odo_f(k(j),1:3), ...
				   odo(j,i_odo-3));
    end
    
  end
  
  
  
function ind = sample(w_cumSum, n)
  ind = rand(n,1);
  
  for i = 1:n
    tmp = find(w_cumSum >= ind(i));
    
    if isempty(tmp)
      ind(i) = length(w_cumSum);
    else
      ind(i) = tmp(1);
    end
    
  end
  
  