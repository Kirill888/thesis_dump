function las = simulate(odo, beacons, lines, maxRange, do_plot, ...
				add_noise)
% las = simulate(odo, beacons, lines, maxRange, ...
%                do_plot=0, add_noise=0)
%
%
%  

%

%odo: t,Vt,steer

if nargin < 5
  do_plot = 0;
end

if nargin < 6
  add_noise = 0;
end

np = size(odo,1);

pos = odo(1,2:4);

if do_plot
  fig = figure;
  set(fig,'DoubleBuffer','on');
  set(fig,'WindowButtonDownFcn','pause');
  plot_circle(beacons(:,1:2),beacons(:,3),'r');
  hold on
  line(lines(:,[1,3])',lines(:,[2,4])');
  h = plot_robot(pos(1),pos(2),pos(3));
  hobs = [];
  hlas = [];

  axis([-100 100 -100 100]); axis equal
else
  hwait = waitbar(0,'Computing Observations');
end


las = zeros(np,361+1);
  

for i = 1:np
  t = odo(i,1);

  pos = odo(i,2:4);
  sensor = robot2sensor(pos);
  
  laser = compute_obs(sensor,beacons,lines, maxRange,add_noise);
  las(i,:) = [t, laser(:,2)'];
  
  if do_plot
    delete(hlas);
    hlas = plot_laser(sensor,laser(:,1),laser(:,2));

    delete(h);
    h = plot_robot(pos(1),pos(2),pos(3));

    %  pause(0.1)
    drawnow
  elseif mod(i,10) == 0
    waitbar(i/np,hwait);
    drawnow
  end
 
end

if do_plot
else
  close(hwait);
end

function laser = compute_obs(pos, beac, lines, maxRange, add_noise)
  NBEAMS = 361;
  
  beac(:,1) = beac(:,1) - pos(1);
  beac(:,2) = beac(:,2) - pos(2);

  lines(:,[1,3]) = lines(:,[1,3]) - pos(1);
  lines(:,[2,4]) = lines(:,[2,4]) - pos(2);
  
  [th,r] = cart2pol(beac(:,1), beac(:,2));
  th = angle_diff(th,pos(3));
  
  i_range = find(r <= maxRange);
  i_th    = find(th <= pi/2 +pi/4 & th >= -pi/2 - pi/4);
  
  ind     = intersect(i_range, i_th); 

  a = linspace(-pi/2,pi/2,NBEAMS);
  ind2 = zeros(NBEAMS,1);
  R    = zeros(NBEAMS,1);

  for i = 1:NBEAMS
    R(i) = ray_trace(angle_diff(a(i),-pos(3)), beac(ind,:), ...
			       lines, maxRange, add_noise);
  end

  i = find(R > maxRange);
  R(i) = maxRange;
  laser = [a',R];

function R = ray_trace(a,beac, lines, maxRange, add_noise)
  sigma_d       = 0.0055;
  discrete_step = 0.001;
  beam_width    = 7/4700;
  NPOINTS       = 3;

  nbeac = size(beac,1);

  if add_noise
    aa  = linspace(a-0.5*beam_width,a+0.5*beam_width,NPOINTS);
    ca  = cos(aa); 
    sa  = sin(aa);
    R   = ones(NPOINTS,1)*maxRange;
    ind = zeros(NPOINTS,1);
    
    for i = 1:NPOINTS
      l  = ray_intersect(ca(i),sa(i),beac);
      l2 = ray_intersect_lines(ca(i),sa(i),lines);
      
      l = [l;l2];
      
      [lmin, j] = min(l);
      
      if lmin < maxRange
	R(i)   = lmin;
      end
    end
    
    ii = find(R < maxRange);
    
    if isempty(ii)
       R = maxRange;
    else
       %Add noise
       noise = randn(size(ii))*sigma_d;
       R = mean(R(ii) + noise);
       
       %Discretise
       R = round(R/discrete_step)*discrete_step;
    end
    
  else
    R   = maxRange;
    ca = cos(a);
    sa = sin(a);

    l = ray_intersect(ca,sa,beac);
    l2 = ray_intersect_lines(ca,sa,lines);
      
    l = [l;l2];
    lmin = min(l);
    
    if lmin < maxRange
      R = lmin;
    end
  end


  
function l = ray_intersect(ca,sa,beacons)
xc = beacons(:,1);
yc = beacons(:,2);
r  = beacons(:,3);

b  = -2*(xc.*ca + yc.*sa);
c  = xc.*xc + yc.*yc - r.*r;
D  = b.*b - 4*c;

l  = zeros(size(D)); 
l  = l + Inf;
i  = find(D >= 0);
l(i) = 0.5.*(-b(i) - sqrt(D(i)));
i = find(l < 0);
l(i) = Inf;

function R = ray_intersect_lines(ca,sa,l)
d1 = sa.*l(:,1) - ca.*l(:,2);
d2 = sa.*l(:,3) - ca.*l(:,4);

nl = size(l,1);
R  = ones(nl,1)*inf;

ind = find(d1.*d2 <= 0);

abc = line2abc(l);

R(ind) =        - abc(ind,3)./ ...
	 (abc(ind,1).*ca + abc(ind,2).*sa );

ii = find(R < 0); R(ii) = Inf;

  
function sensor = robot2sensor(pos)
a = 3;
b = 0;

ca = cos(pos(3));
sa = sin(pos(3));

sensor(1) = pos(1) + a*ca - b*sa;
sensor(2) = pos(2) + a*sa + b*ca;

sensor(3) = pos(3);

function pos = move_robot(pos0,dt,Vt,steer)
L = 3.0;

a = pos0(3);
Vc = Vt;

pos(1) = pos0(1) + dt*Vc*cos(a);
pos(2) = pos0(2) + dt*Vc*sin(a);
pos(3) = pos0(3) + dt*Vc*tan(steer)/L;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%GUI output functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function h = plot_robot(x,y,a)
rxx = [ 0     2     3     2     0     0;   
	1     1     0    -1    -1     1];
R = [cos(a), -sin(a); 
     sin(a), cos(a)];
rxx = R*rxx;
rxx(1,:) = rxx(1,:) + x;
rxx(2,:) = rxx(2,:) + y;

h = line(rxx(1,:),rxx(2,:));

function h = plot_obs(pos, o)
 
  if isempty(o)
    h = [];
    return
  end
  
  r = o(:,1);
  th = o(:,2) + pos(3);
  
  [x,y] = pol2cart(th,r);
  x = x + pos(1);
  y = y + pos(2);
  
  h = plot(x,y,'bs');
  
function h = plot_laser(sensor, th, range)
  a = th + sensor(3);
  [x,y] = pol2cart(a,range);
  x = x + sensor(1);
  y = y + sensor(2);
  
  x0 = ones(size(th))*sensor(1);
  y0 = ones(size(th))*sensor(2);
  
  h = line([x0';x'],[y0';y']);
  set(h,'Color','g');
  
