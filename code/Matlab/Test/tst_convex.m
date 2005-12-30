function tst_convex(xx, poly)

  lxx = [poly(:,1);poly(1,1)]';
  lyy = [poly(:,2);poly(1,2)]';
  clf;
  subplot(2,1,1)
  line(lxx,lyy);
  hold on
  plot(poly(:,1),poly(:,2),'bs');
  
  plot(xx(:,1),xx(:,2),'ro');
  axis equal
  
  nv = size(poly,1);
  nMax = 0;
  lMax = [];
  iMax = [];
  
  for i = 0:(nv-1)
    i0 = i + 1;
    
    i1 = mod(i-1, nv)  + 1;
    i2 = mod(i+1, nv)  + 1;
    
    subplot(2,1,1);
    h = plot(poly(i0,1),poly(i0,2),'gs', ...
	     poly([i1],1),poly([i1],2),'ro', ...
	     poly([i2],1),poly([i2],2),'bo'); 
    
    set(h,'MarkerSize',15);

    subplot(2,1,2)
    [l,n] = processVertex(poly(i0,:), poly(i1,:), poly(i2,:), ...
				 xx);
    pause
    delete(h);
    
    if n > nMax
      nMax = n;
      lMax = l;
    end
    
  end
  
  d = dist2Line(xx, lMax(1,:), lMax(2,:));
  iMax = find(d < 0);
  
  subplot(2,1,1);
  h = line(lMax(:,1), lMax(:,2)); 
  set(h,'Color','r','LineWidth',2);
  plot(xx(iMax,1), xx(iMax,2),'b.');


function [l, nPoints, i_good] = processVertex(v, v1, v2, xx)
   dx = dist2Line(xx,v1,v);
   dy = dist2Line(xx,v2,v);
   
   subplot(2,2,3);
   hold off
   plot(dx,dy,'rs');
   hold on

   line([0,1],[0,0]);
   line([0,0],[0,1]);
   h = line([-1,0],[0 0]); set(h,'Color','r');
   h = line([0,0],[-1,0]); set(h,'Color','r')
   
   ind_1 = find(dx > 0 & dy > 0);
   ind_2 = find(dx < 0 & dy > 0);
   ind_3 = find(dx < 0 & dy < 0);
   ind_4 = find(dx > 0 & dy < 0);
   
   baseGood = length(ind_2); %Points in second quadrant are always
                             %accepted
  
   np = length(ind_1) + length(ind_3);
   i1 = 1:length(ind_1);
   i2 = (length(ind_1)+1):np;

   a = [ dy(ind_1)./dx(ind_1); dy(ind_3)./dx(ind_3) ];
   a = [a, ones(np,1), ones(np,1)]
   a(i1,3) = ind_1;
   a(i2,3) = ind_3;
   a(i2,2) = -1;
   
   [a,i_sort] = sortrows(a,1);
   
   subplot(2,2,4);
   plot(a(:,1),a(:,2),'rs-');
   

   nGood = length(ind_1);
   prev  = a(1,2);

   if prev < 0
     nGood = nGood + 1;
   end
   
   nMax  = nGood;
   iMax  = 1;
   
   for i=2:np
     if prev == a(i,2)
       nGood = nGood - prev;

       if nGood > nMax
	 nGood = nMax;
	 iMax = i;
       end
     end
     prev = a(i,2);
   end

   if a(iMax,2) > 0
     l = [v; xx(a(iMax,3),1:2)];
   else
     l = [xx(a(iMax,3),1:2); v];
   end
   
   nPoints = nMax + baseGood;

   
function [d] = dist2Line(xx, v1,v2)
  np = size(xx,1);
  d = zeros(np,1);
  
  x1 = [v1(1);v1(2)];
  x2 = [v2(1);v2(2)];
  
  DX = x2 - x1;
  scaler = 1/sqrt(DX'*DX);  
  
  for i = 1:np
    x0 = xx(i,1:2)';
    A = [DX, x1 - x0];
    d(i) = det(A);
  end
  
  d = d*scaler;


  
function [l, nPoints, i_good] = processVertex_a(v, v1, v2, xx)
 
   a2 = atan2(v2(2)-  v(2), v2(1) -  v(1)); % v => v2
   a1 = atan2( v(2)- v1(2),  v(1) - v1(1)); % v1 => v
   
   
   if a2 < 0
     a2 = a2 + 2*pi;
   end
   
   if a1 < 0
     a1 = a1 + 2*pi;
   end
   
   
   a_min = a2;
   a_max = a1 - a2;
   if a_max < 0
     a_max = a_max + 2*pi;
   end
   

   dx = xx(:,1) - v(1);
   dy = xx(:,2) - v(2);

   a  = atan2(dy,dx) - a_min;
   ind = find(a < 0);
   a(ind) = a(ind) + 2*pi;   % a  is [0 -> 2pi)
   ind = find(a < 0);
   a(ind) = a(ind) + 2*pi;   % a  is [0 -> 2pi)
   
   np = length(a);
   [a,i_sort] = sort(a);
   
   nPoints = -1;
   i_max   = -1;
   i_good  = [];
   
   hold off
   polar(a,ones(size(a)),'bo'); 
   hold on
   polar([0,0],[0 1],'r');
   polar([0,a_max],[0 1],'b');
   polar([0,a_max+pi],[0 1],'g');
   
   
   
%   polar([0,a2],[0,1],'m-');
%   polar([0,a1],[0,1],'c-');

   
   for i = 1:np
     if a(i) < a_max | (a(i) > pi & a(i) < (a_max + pi))
      [count,ind] = count_points(a,i,np);
      if count > nPoints
	i_max = i;
	nPoints = count;
	i_good  = ind;
      end
      polar(a(i),1,'r.');
     end
   end
   
   if i_max > 0
     l = [v(1:2);xx(i_sort(i_max),1:2)];
     i_good = i_sort(i_good);
   else
     l = [];
   end
   
function [k,ind] = count_points(a,i, np)
  if a(i) > pi
    a1 = a(i) - pi;
    a2 = a(i);
  else
    a1 = a(i);
    a2 = a(i) + pi;
  end

  ind = find( a >= a1 & a <= a2);
  k = length(ind);
  
  disp(sprintf('a < %f -> %f > %d',a1*180/pi,a2*180/pi,k));

  
   
      

















function aux1(x0,xx,poly)
  figure;
  subplot(2,1,1);
  lx0 = ones(size(xx));
  lx0(:,1) = lx0(:,1)*x0(1);
  lx0(:,2) = lx0(:,2)*x0(2)
  lxx = [lx0(:,1), xx(:,1)]';
  lyy = [lx0(:,2), xx(:,2)]';
  line(lxx,lyy);
  hold on
  
  lxx = [poly(:,1);poly(1,1)]'
  lyy = [poly(:,2);poly(1,2)]'
  line(lxx,lyy);
  plot(poly(:,1),poly(:,2),'ro');
  
  axis equal

  poly_min = min(poly);
  poly_max = max(poly);
  
  if x0(1) < poly_min(1)
    xx(:,1)   = xx(:,1)   - x0(1);
    xx(:,2)   = xx(:,2)   - x0(2);
    poly(:,1) = poly(:,1) - x0(1);
    poly(:,2) = poly(:,2) - x0(2);
  
  elseif x0(1) > poly_max(1)
    xx(:,1)   = -xx(:,1)   + x0(1);
    xx(:,2)   =  xx(:,2)   - x0(2);
    poly(:,1) = -poly(:,1) + x0(1);
    poly(:,2) =  poly(:,2) - x0(2);

  elseif x0(2) < poly_min(2)
    xx(:,1)   = xx(:,1)   - x0(1);
    xx(:,2)   = xx(:,2)   - x0(2);
    poly(:,1) = poly(:,1) - x0(1);
    poly(:,2) = poly(:,2) - x0(2);
    
    % Y <= -X, X <= Y
    tmpx    =  xx(:,2);
    xx(:,2) = -xx(:,1);
    xx(:,1) =  tmpx;

    tmpx      =  poly(:,2);
    poly(:,2) = -poly(:,1);
    poly(:,1) =  tmpx;

  elseif x0(2) > poly_max(2)
    xx(:,1)   = xx(:,1)   - x0(1);
    xx(:,2)   = xx(:,2)   - x0(2);
    poly(:,1) = poly(:,1) - x0(1);
    poly(:,2) = poly(:,2) - x0(2);

    % Y <= X, X <= -Y
    tmpx    = -xx(:,2);
    xx(:,2) =  xx(:,1);
    xx(:,1) =  tmpx;

    tmpx      = -poly(:,2);
    poly(:,2) =  poly(:,1);
    poly(:,1) =  tmpx;
    
  else
    disp('Can''t handle this case.');
  end
  
  subplot(2,1,2);
  lxx = [zeros(size(xx,1),1), xx(:,1)]';
  lyy = [zeros(size(xx,1),1), xx(:,2)]';
  line(lxx,lyy);
  axis equal
  hold on
  lxx = [poly(:,1);poly(1,1)]'
  lyy = [poly(:,2);poly(1,2)]'
  line(lxx,lyy);
  plot(poly(:,1),poly(:,2),'ro');
