function err = las_test(odo_r, obs, mapping, R, do_plot)

  if nargin < 5
    do_plot = 1;
  end

  odo_l = odo2las(odo_r,R);

  nobs = size(obs,1);

  p = zeros(2,2,nobs);
  x = zeros(nobs,1);
  y = zeros(nobs,1);

  for i = 1:nobs
    p = [obs(i,3:4); obs(i,5:6)];

    [x(i), y(i), p(:,:,i)] = translate(obs(i,1), obs(i,2), p, ...
                                       odo_l(obs(i,7),:));
  end

  [nc,v] = max(mapping);
  err = 0;
   
  cluster = zeros(nc,2);

  for i = 1:nc
    ind = find(mapping == i);
    
    cluster(i,:) = [mean(x(ind)), mean(y(ind))];

    d = sqrt(dist2(cluster(i,:), [x(ind),y(ind)]))';
    
    err = err + sum(d);
  end

  if do_plot
     figure
     plot(x,y,'ro');
     hold on
     axis equal

     h = plot(cluster(:,1),cluster(:,2),'bs');
     set(h,'MarkerSize',8);

     plot_robot(odo_l);

     title(sprintf('R = %f, err = %e',R,err));
  end


function d2 = dist2(from, to)
  n1 = size(from,1);
  n2 = size(to,1);

  d2 = zeros(n1,n2);

  for i = 1:n1
  for j = 1:n2
      
     dd = from(i,:) - to(j,:);

     d2(i,j) = dd*dd';
  end
  end

function [x,y,p] = translate(x,y,p,odo)
  a = odo(3);
  R = [ cos(a), sin(a);
       -sin(a), cos(a)];
  
  p  = R'*p*R;
  xy = [x,y]*R + odo(1:2);

  x  = xy(1);
  y  = xy(2);

