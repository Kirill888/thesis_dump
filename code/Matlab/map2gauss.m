function [gmap, godo] = map2gauss(map)
%  function g = map2gauss(map)
%
%  Compute gaussian approximation of map coverage
%

%
  
  m = mean(map.map(:,1:2),1);
  nm = size(map.map,1);
  
  if nm <= 2
    if nm == 0
      c = zeros(2);
    elseif nm == 1
      c = map.map(1,1:2);
    else
      xx = map.map(1,1:2) - m;
      r2 = xx*xx'/9;
      a  = atan2(xx(2),xx(1));
      R  = [cos(a) -sin(a); sin(a) cos(a)];
      c  = R*[r2 0; 0 (0.5/3)^2]*R';
    end
  else
    c = 0.2*cov(map.map(:,1:2));
  end
  
  
  gmap = [m , c(1,:), c(2,:)];
  
  if nargout > 1
    
    if 0
      odo_m   = mean(map.odo(:,1:2),1);
      
      odo_cov = cov(map.odo(:,1:2));
      
      [u,s,v] = svd(odo_cov);
      s(2,2) = max(s(2,2),0.25^2);
      odo_cov = u*s*v';
    else
      odo_x = map.odo(:,1);
      odo_y = map.odo(:,2);
      R     = 0.4;
      odo_ = [odo_x    , odo_y;
	      odo_x - R, odo_y;
	      odo_x + R, odo_y;
	      odo_x    , odo_y + R;
	      odo_x    , odo_y - R];
      odo_m = mean(odo_,1);
      odo_cov = cov(odo_,1);
    end
    
    godo = [odo_m, odo_cov(1,:), odo_cov(2,:)];
  end
  
	  
  
  
  return
  mplot(map.map, 'bs','b-');
  hold on
  
  mplot(g,'ro','r-');