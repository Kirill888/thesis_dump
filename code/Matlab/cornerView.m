function cornerView(las,corners,scan)
% function cornerView(las,corners,scan)
%

%

if nargin < 3
  fig         = figure;
  ud.las      = las;
  
  if nargin < 2
    ud.corners = zeros(0,7);
  else
    ud.corners  = corners;
  end
  
  ud.scan     = 1;
  ud.maxScan  = size(las,1);
  
  ud.a  = linspace(0,pi,361);
  ud.ca = cos(ud.a);
  ud.sa = sin(ud.a);
  
  ud.hScan    = plot(0,0,'b.-'); hold on; axis equal;
  ud.hCorners = plot(0,0,'ro');
  ud.hRays    = plot(0,0,'g-');
  
  ud.hText    = text(-7,8,'TEXT');
  ud.hArms    = plot(zeros(3,100), zeros(3,100),'r-');
  ud.hLabel   = text(zeros(1,100), zeros(1,100),'');
  
  axis([-9 9 0 9]);
  
  set(fig,'UserData',ud ...
         ,'KeyPressFcn', @fig_KeyPressFcn ...
         ,'DoubleBuffer', 'on');
  
  setDisplayData(ud);
  return
end

ind = find(corners(:,1) == scan);
cc  = corners(ind,2:end)

las = las(scan,2:362);

a = linspace(0,pi,361);
ca = cos(a);
sa = sin(a);

lx = las.*ca;
ly = las.*sa;

plot(lx,ly,'b.-'); hold on; axis equal
hc = plot_corners(cc,0.3);
hold off;

function h = plot_corners(cc,R)
 [x,y] = corner_data(cc,R);
 h = plot(x', y','r-',cc(:,1),cc(:,2),'r.');
 set(h,'LineWidth',2,'MarkerSize',20);

function [x,y] = corner_data(cc,R)
  nc = size(cc,1);
  x = zeros(nc,3);
  y = zeros(nc,3);
  
  x(:,1) = cc(:,1) + R.*cc(:,3);
  y(:,1) = cc(:,2) + R.*cc(:,4);
  x(:,2) = cc(:,1);
  y(:,2) = cc(:,2);
  x(:,3) = cc(:,1) + R.*cc(:,5); 
  y(:,3) = cc(:,2) + R.*cc(:,6); 
  

function setDisplayData(ud)
  las = ud.las(ud.scan,2:362);
  lx = ud.ca.*las;
  ly = ud.sa.*las;
  
  set(ud.hScan,'XData',lx,'YData',ly);
  
  ind = find(ud.corners(:,1) == ud.scan);
  cc  = ud.corners(ind,2:end);
  set(ud.hCorners,'XData', cc(:,1), 'YData', cc(:,2));
  
  set(ud.hText,'String', sprintf('Scan Id: %05d',ud.scan));
  
  %Plot rays
  r2 = cc(:,1).*cc(:,1) + cc(:,2).*cc(:,2);
  r  = sqrt(r2) + 0.1;
  a  = atan2(cc(:,2), cc(:,1));
  a1 = a - 1/180*pi;
  a2 = a + 1/180*pi;
  x1 = r.*cos(a1);
  y1 = r.*sin(a1);
  x2 = r.*cos(a2);
  y2 = r.*sin(a2);
  
  xx = zeros(length(x1)*3 + 1,1);
  yy = xx;
  
  xx(1:3:end) = 0;  xx(2:3:end) = x1'; xx(3:3:end) = x2';
  yy(1:3:end) = 0;  yy(2:3:end) = y1'; yy(3:3:end) = y2';
  
  set(ud.hRays,'XData',xx,'YData',yy);
  
  %Process corner arms
  
  [xx,yy] = corner_data(cc,0.5);
  
  for i = 1:size(xx,1)
    set(ud.hLabel(i),'Position',[xx(i,2),yy(i,2),0],...
		      'String', type2label(cc(i,7)),...
		      'Visible','on');
    set(ud.hArms(i),'XData',xx(i,:), 'YData', yy(i,:),'Visible','on');
  end
  
  for i = (size(xx,1)+1):length(ud.hArms)
    set(ud.hArms(i),'Visible','off');
    set(ud.hLabel(i), 'Visible','off');
  end

function s = type2label(type)
  switch(type)
   case 1
    s = 'cc';
   case 2
    s = 'cvx';
   case 3
    s = 'cvh';
   case 4
    s = 'jmp';
   otherwise
    s = 'unknown';
  end
  
  
  

function fig_KeyPressFcn(hObject, eventdata, handles)
  ud = get(hObject,'UserData');
  
  k = get(hObject,'CurrentCharacter');
%  disp(sprintf('Key Pressed: %d\n', k));

  switch(k)
   case {29,32}
    ud.scan = ud.scan + 1;
   case {28}
    ud.scan = ud.scan - 1;
   case {'.'}
    ud.scan = ud.scan + 10;
   case {','}
    ud.scan = ud.scan - 10;
   
   case {'>'}
    ud.scan = ud.scan + 100;
   case {'<'}
    ud.scan = ud.scan - 100;
    
   case {'g'}
    disp('Go to');
    frame = input('Frame index:');
    if ~isempty(frame)
      ud.scan = frame;
    end
    
  end
  
  if ud.scan <= 0
    ud.scan = 1;
  end
  
  if ud.scan > ud.maxScan
    ud.scan = ud.maxScan;
  end
  
  setDisplayData(ud);
  
  set(hObject,'UserData',ud);
