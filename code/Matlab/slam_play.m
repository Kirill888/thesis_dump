function slam_play(mask, from, to, X0)
% function slam_play(mask, from, to)
%
%

%

fig = gcf;
%ax  = gca;
set(fig,'DoubleBuffer','on');
hold on;


MAPS(20).map0 = [];
MAPS(20).h = [];
MAPS(20).update = 1;

MAPS(20).hGrid = [];
MAPS(20).grid = [];
MAPS(20).updateGrid = 1;
MAPS(20).hText = [];
MAPS(20).pose = [0 0];

HYPES(50).map     = {};
HYPES(50).grid    = {};
HYPES(50).links   = [];
HYPES(50).map_ref = [];

if nargin < 4
  X0 = [0,0,0];
end


cc = ['r','g','r','g','r','g','b','g','b','r','b','g','b'];
map_ref = zeros(0,3);

for i = 1:length(cc)
  h = plot(0,0,'.','MarkerSize',10);
  set(h,'XData',[], 'YData',[],'Color', cc(i));
  
  ccc = get(h,'Color');  
  ccc = rgb2hsv(ccc);
  ccc(2) = ccc(2)*0.3;
  ccc = hsv2rgb(ccc);
  
  set(h,'Color',ccc);
  
  MAPS(i).hGrid = h;
  MAPS(i).hText = text(0,0,sprintf('%02d',i),...
		       'FontSize', 15, 'Visible','off','Color',cc(i));
end

for i = 1:20
  hLinks(i) = plot(0,0,'k.-');
  set(hLinks(i),'XData',[],'YData',[]);
end

hParticles = plot(0,0,'g.');
hLas = plot(0,0,'y.');
hRobot = plotRobot([], [0 0 0]);
hObs = plot(0,0,'b.-');

ax = axis;
hStateLabel = text(ax(1), ax(3) + 0.5,'','FontSize',12, ...
		   'FontWeight','bold');

SENSOR_POSE = [0.227, 0.024 , 0];

iHype = 0;

for i = from:to
  run(sprintf(mask,i));
  % obs, las, odo, hype
  
  updateAllMaps = 0;
  replotLinks = 0;

  iHypePrev = iHype;
  
  nmax = 0; iHype = 0;
  ih = unique(odo(:,5));
  ActiveMaps = zeros(0,2);
  
  for j = 1:length(ih)
    hInd = ih(j);
    
    HYPES(hInd).map{hype(hInd).mapi} = hype(hInd).map;

    if ~isempty(hype(hInd).map_ref)
      %Update map_ref
      HYPES(hInd).map_ref = translate_odo( ...
	                             zero_odo(hype(hInd).map_ref(:,1:3)), ...
				     X0);
      HYPES(hInd).links = hype(hInd).links;

      replotLinks = 1;
      updateAllMaps = 1;
    end

    if ~isempty(hype(hInd).region)
      HYPES(hInd).grid{hype(hInd).mapi} = hype(hInd).region;
    end
    
    if isfield(hype(hInd),'prevMap') 
      if hype(hInd).prevMap > 0
	HYPES(hInd).grid{hype(hInd).prevMap} = hype(hInd).region_prev;
      end
    end
    
    map_ref = HYPES(hInd).map_ref;
    ii = find(odo(:,5) == hInd);
    odo(ii,1:3) = translate_odo(odo(ii,1:3), ...
				map_ref(hype(hInd).mapi,:));

    ActiveMaps = [ActiveMaps; [hype(hInd).mapi, length(ii)]];
    
    if length(ii) > nmax
      nmax = length(ii);
      iHype = hInd;
      odo0 = mean(odo(ii,:));
    end
  end

  %Update maps
  for j = 1:length(HYPES(iHype).map)
    if ~isempty(HYPES(iHype).map{j})
      MAPS(j).map0   = HYPES(iHype).map{j};
      MAPS(j).update = 1;
      HYPES(iHype).map{j} = [];
    end
  end
  
  %Update grids
  for j = 1:length(HYPES(iHype).grid)
    if ~isempty(HYPES(iHype).grid{j})
      MAPS(j).grid   = HYPES(iHype).grid{j};
      MAPS(j).updateGrid = 1;
      HYPES(iHype).grid{j} = [];
    end
  end
  
  map_ref = HYPES(iHype).map_ref;
  imap = hype(iHype).mapi;
  ref0 = map_ref(imap,:);
  
  odoLas = odo2sensor(odo0, SENSOR_POSE);

  if iHype ~= iHypePrev
    updateAllMaps = 1;
  end
  
  %Regions
  for j = 1:hype(iHype).mapn
    if updateAllMaps == 1 || MAPS(j).updateGrid == 1
      [xt,yt] = plotGrid(MAPS(j).hGrid, MAPS(j).grid, map_ref(j,:));
      MAPS(j).updateGrid = 0;
      
      set(MAPS(j).hText,'Position',[xt,yt],'Visible','on');
      MAPS(j).pose = [xt,yt];
      
      replotLinks = 1;
    end
  end
  
  %Links
  if replotLinks ~= 0 || iHype ~= iHypePrev
    ll = HYPES(iHype).links;
    
    for j = 1:size(ll,1)
      x1 = MAPS(ll(j,1)).pose;
      x2 = MAPS(ll(j,2)).pose;
      
      set(hLinks(j),'XData',[x1(1), x2(1)], 'YData', [x1(2),x2(2)]);
    end
  end
  
  %Maps
  for j = 1:hype(iHype).mapn
    if updateAllMaps == 1 || MAPS(j).update == 1
      delete(MAPS(j).h);
      map = translate_map(MAPS(j).map0, map_ref(j,:));
      MAPS(j).h = plotMap(map,cc(j));
      MAPS(j).update = 0;
    end
  end

  
  %Robot
  plotRobot(hRobot, odo0);
  
  %Laser
  plotLaser(hLas, las, odoLas);
  
  %Particles
  set(hParticles,'XData',odo(:,1), 'YData', odo(:,2));

  
  %Observations
  [xo,yo] = o2points(obs, 1, odoLas);
  set(hObs,'XData', xo, 'YData', yo);

  %State label
  set(hStateLabel,'String', [sprintf('S: %05d Map:',i), ...
		    sprintf(' %d (%d p)',ActiveMaps')]);
  %Flush Graphical Output
  if mod(i,5) == 0
    drawnow;
    %print('-dpng', sprintf('short_%04d.png',i/5-1));
  end
  
  clear obs las odo hype
end

function m = translate_map(m0, ref0)
if isempty(m0)
  m = [];
else
  m = m0;
  m(:,1:6) = translate_obs(m0(:,1:6),ref0);
  m(:,8:9) = m(:,8:9) + ref0(3);
end


function h = plotMap(map,color)
  if isempty(map)
    h = [];
  else
    h = plot_corner_map(map,0.4);
    set(h,'LineWidth',2,'Color',color);
  end

function odo = odo2las(odo0)
 odo = translate_odo(LAS_POSE, odo0);

function h = plotRobot(h, odo)
  robot_x = [0.300,0.290,0.260,0.212,0.150,0.078,0.000,-0.078,...
	     -0.150,-0.212,-0.260,-0.290,-0.300,-0.290,-0.260,...
	     -0.212,-0.150,-0.078,-0.000,0.078,0.150,0.212,0.260,...
	     0.290,0.300,0.230,0.180,0.080,0.080,0.180,0.230];
  
  robot_y = [ 0.000,0.078,0.150,0.212,0.260,0.290,0.300,0.290, ...
	      0.260,0.212,0.150,0.078,0.000,-0.078,-0.150,-0.212, ...
	      -0.260,-0.290,-0.300,-0.290,-0.260,-0.212,-0.150,-0.078, ...
	      -0.000,0.000,0.070,0.070,-0.070,-0.070,0.000];

  rr = translate_obs([robot_x',robot_y'], odo);
  
  if isempty(h)
    h = plot(rr(:,1), rr(:,2), 'b-', 'LineWidth',2);
  else
    set(h,'XData', rr(:,1), 'YData', rr(:,2));
  end

function [x,y] = plotGrid(h, regGrid, odo0)
  if isempty(regGrid.data)
    set(h,'XData',[],'YData',[]);
    x = []; y = [];
    return;
  end
%  regGrid.data = regGrid.data(1:3:end,1:3:end);

  dim = regGrid.dim;
  xx = linspace(dim(1), dim(3), size(regGrid.data,2));
  yy = linspace(dim(2), dim(4), size(regGrid.data,1));
  [yi,xi] = find(regGrid.data > 0);
  xx = xx(xi)';  yy = yy(yi)';
  
  if ~isempty(odo0) % Translate points
    xy = translate_obs([xx,yy],odo0);
    xx = xy(:,1);
    yy = xy(:,2);
  end
  
  set(h,'XData',xx,'YData',yy);
  
  x = mean(xx); y = mean(yy);

function plotLaser(h, las,odo0)
  a = linspace(-pi/2, pi/2, 361) + odo0(3);
  
  x = las.*cos(a) + odo0(1);
  y = las.*sin(a) + odo0(2);
  
  ii = find(las < 8.18);
  
  set(h,'XData', x(ii), 'YData', y(ii));

function [xx, yy] = o2points(obs,sd, odo0)
  if isempty(obs)
    xx = [];
    yy = [];
  else
    r = obs(:,1);
    a = obs(:,2) + odo0(3);
    dr = sd*sqrt(obs(:,3));
    da = sd*sqrt(obs(:,6));
    
    a1 = a - da;
    a2 = a + da;
    r1 = r + dr;
    r2 = r - dr;

    x1 = r1.*cos(a1) + odo0(1); y1 = r1.*sin(a1) + odo0(2);
    x2 = r1.*cos(a2) + odo0(1); y2 = r1.*sin(a2) + odo0(2);
    x3 = r2.*cos(a2) + odo0(1); y3 = r2.*sin(a2) + odo0(2);
    x4 = r2.*cos(a1) + odo0(1); y4 = r2.*sin(a1) + odo0(2);
    
    xx = zeros(length(x1)*6 + 1,1);
    yy = xx;
    
    xx(1:6:end) = odo0(1)  ; yy(1:6:end) = odo0(2);
    xx(2:6:end) = x1';       yy(2:6:end) = y1';
    xx(3:6:end) = x2';       yy(3:6:end) = y2';
    xx(4:6:end) = x3';       yy(4:6:end) = y3';
    xx(5:6:end) = x4';       yy(5:6:end) = y4';
    xx(6:6:end) = x3';       yy(6:6:end) = y3';
  end
