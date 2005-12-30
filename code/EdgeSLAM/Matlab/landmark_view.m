function landmark_view(gmap,obs,map_ind, lndm_index)
% function landmark_view(gmap,obs,map_ind, lndm_index)
%
%
%

if nargin < 3
  map_ind = 1;
end

if nargin < 4
  lndm_index = 1;
end

ud.fig = figure;

ud.gmap    = gmap;
ud.obs     = obs;
ud.map     = gmap.maps(map_ind);
ud.ind_map = map_ind;

ud = initFig(ud);

ud = setLandmark(ud,lndm_index);



set(ud.fig ,'UserData',ud ...
           ,'KeyPressFcn', @fig_KeyPressFcn ...
	   ,'ButtonDownFcn',@fig_ButtonDownFcn ...
           ,'DoubleBuffer', 'on');

setDisplayData(ud);


function ud = initFig(ud)
  figure(ud.fig)
  subplot(2,1,1);
  ud.hImage = imshow(zeros(240,640*2));
  hold on
  plot([640,640],[1,240],'b-');
  ud.h_u1 = plot([1,1],[1,240],'r-');
  ud.h_u2 = plot([1,1],[1,240],'r-');

  set([ud.h_u1,ud.h_u2],'LineWidth',2);
  
  ud.hText = title('');
  set(ud.hText,'FontSize',13,'Color','b');

  subplot(2,2,3);
  mplot(ud.map.map,'r.','r-',1);
  axis equal
  
  subplot(2,2,4);
  ud.hTemplate = plot([1:21],[1:21]*0,'bo-');
  hold on;
  ud.hTemplate_avg = plot([1:21],[1:21]*0,'rs-');
  ud.hTemplate_all = [];
  set(ud.hTemplate,'LineWidth',2);
  set(ud.hTemplate_avg,'LineWidth',2);
  axis([1,21,-100,100]);
  
  nmap = size(ud.map.map,1);
% $$$   ud.slider = uicontrol(ud.fig,'Style','slider',...
% $$$ 			'Position',[100,10 500 20], ...
% $$$ 			'Min',1,'Max',nmap,'Value',1, ...
% $$$ 			'SliderStep',[1/nmap 1/nmap],...
% $$$ 			'Callback',@SliderCallback);

  s = sprintf('%d|',1:nmap);
  s = s(1:end-1);
  ud.menu = uicontrol(ud.fig,'Style','popup',...
			'Position',[10,10 60 20], ...
		        'String',s,...
			'Value',1, ...
			'Callback',@MenuCallback);
  
  
  
function ud = setLandmark(ud, l)
  figure(ud.fig)
  ud.ind_l = l;
  ud.ind_obs = ud.map.map2obs{l};
  ud.i       = 1;
  ud.maxI    = length(ud.ind_obs);

  tt = ud.obs(ud.ind_obs,10:end);
  
  subplot(2,2,4);
  delete(ud.hTemplate_all);
  ud.hTemplate_all = plot(tt','k.');
  set(ud.hTemplate_all,'MarkerSize',3);
  set(ud.hTemplate_avg,'YData',mean(tt));
  set(ud.menu,'Value',l);
  

function setDisplayData(ud)
 global LEFT_MASK
 global RIGHT_MASK
 global CX1 CX2 F1 F2

 figure(ud.fig)
 o = ud.obs(ud.ind_obs(ud.i),:);
 frame = o(1);
 
 %Convert to pixel from normalised
 u1 = CX1 - o(2)*F1;
 u2 = CX2 - o(3)*F2;
 
 v1 = o(8);
 v2 = o(9);
 
 template = o(10:end);
 
% disp(sprintf('Loading Images: %d',frame));
 
 I1 = imread(sprintf(LEFT_MASK,frame));
 I2 = imread(sprintf(RIGHT_MASK,frame));

 II = [I1, I2];

 set(ud.hImage,'CData',II);
 set(ud.h_u1,'XData',[u1,u1],'YData',[v1,v2]);
 set(ud.h_u2,'XData',640+[u2,u2],'YData',[v1,v2]);
 set(ud.hTemplate,'YData',template);
 
 set(ud.hText,'String', ...
	sprintf('Landmark %d, map %d (obs %d/%d, frame %d)',...
		ud.ind_l,ud.ind_map,ud.i,ud.maxI,frame));
 
 drawnow 
 
 
function MenuCallback(hObject, eventdata, handles)
  l = floor(get(hObject,'Value'));
  
  fig= get(hObject,'Parent');
  ud = get(fig,'UserData');

  ud = setLandmark(ud,l);

  setDisplayData(ud);
  
  set(fig,'UserData',ud,'Selected','on');
  figure(fig);

function fig_ButtonDownFcn(hObject, eventdata, handles)
  ud = get(hObject,'UserData');
  s = get(hObject,'SelectionType');
  
  switch(s)
   case 'normal'
    ud.i = ud.i + 1;
   case 'alt'
    ud.i = ud.i - 1;
   otherwise
    return
  end
  
  if ud.i <= 0
    ud.i = 1;
    return
  end
  
  if ud.i > ud.maxI
    ud.i = ud.maxI;
    return
  end
  
  setDisplayData(ud);
  
  set(hObject,'UserData',ud);
  
  

function fig_KeyPressFcn(hObject, eventdata, handles)
  ud = get(hObject,'UserData');
  
  k = get(hObject,'CurrentCharacter');
%  disp(sprintf('Key Pressed: %d\n', k));

  switch(k)
   case {29,32}
    ud.i = ud.i + 1;
   case {28}
    ud.i = ud.i - 1;

   case {'0'}
    ud.i = 1;
    
   case {'q'}
    close
    return
    
    case {'.'}
     ud.i = ud.i + 10;
    case {','}
     ud.i = ud.i - 10;
    
    case {'>'}
     ud.i = ud.i + 100;
    case {'<'}
     ud.i = ud.i - 100;
   
    case {'l'}
     disp('Go to landmark');
     l = input('Landmark index:');
     if ~isempty(l)
      ud = setLandmark(ud,l);
     end
     
   otherwise
    return
    
  end
  
  if ud.i <= 0
    ud.i = 1;
    return
  end
  
  if ud.i > ud.maxI
    ud.i = ud.maxI;
    return
  end
  
  setDisplayData(ud);
  
  set(hObject,'UserData',ud);





