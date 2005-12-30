function varargout = mapView(varargin)
% MAPVIEW M-file for mapView.fig
%      MAPVIEW, by itself, creates a new MAPVIEW or raises the existing
%      singleton*.
%
%      H = MAPVIEW returns the handle to a new MAPVIEW or the handle to
%      the existing singleton*.
%
%      MAPVIEW('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MAPVIEW.M with the given input arguments.
%
%      MAPVIEW('Property','Value',...) creates a new MAPVIEW or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before mapView_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to mapView_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help mapView

% Last Modified by GUIDE v2.5 09-Sep-2004 11:47:16

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @mapView_OpeningFcn, ...
                   'gui_OutputFcn',  @mapView_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin & isstr(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before mapView is made visible.
function mapView_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to mapView (see VARARGIN)

% Choose default command line output for mapView
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

if ~isempty(varargin)
    gmap = varargin{1};
    
    initControls(handles, gmap);
end



% UIWAIT makes mapView wait for user response (see UIRESUME)
% uiwait(handles.fig);


% --- Outputs from this function are returned to the command line.
function varargout = mapView_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes during object creation, after setting all properties.
function sliderMap_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sliderMap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background, change
%       'usewhitebg' to 0 to use default.  See ISPC and COMPUTER.
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on slider movement.
function sliderMap_Callback(hObject, eventdata, handles)
% hObject    handle to sliderMap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
  val = round(get(hObject,'Value'));
  set(hObject,'Value',val);

  set(handles.txtCurrentMap,'String', val);
  displayMap(val, handles)

  
% --- Executes on button press in btnShowAll.
function btnShowAll_Callback(hObject, eventdata, handles)
% hObject    handle to btnShowAll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
  figure;
  plot_map_ref(handles.ud.mm, handles.ud.map_poses(:,1:3),handles.ud.style);
  axis equal



% --- Executes on button press in btnZoom.
function btnZoom_Callback(hObject, eventdata, handles)
% hObject    handle to btnZoom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%disp('Zoom');
lbl = get(hObject,'String');

switch(lbl)
 case 'Zoom'
  zoom on
  lbl = 'Zoom Off';
 case 'Zoom Off'
  zoom off
  lbl = 'Zoom';
  axis auto;
  axis equal;
end

set(hObject,'String',lbl);


% --- Executes on button press in btnQuit.
function btnQuit_Callback(hObject, eventdata, handles)
% hObject    handle to btnQuit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close(handles.fig);



% --- Executes on button press in btnOdo.
function btnOdo_Callback(hObject, eventdata, handles)
% hObject    handle to btnOdo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

figure;
gmap = handles.ud.gmap;
plot(gmap.odo(:,1));
axis tight
ax = axis;
set(gca,'YTick', [ax(3):1:ax(4)]);
grid on


% --- Executes on button press in btnColorChange.
function btnColorChange_Callback(hObject, eventdata, handles)
% hObject    handle to btnColorChange (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
labels = {'Red (r)','Green (g)','Blue (b)','Magenta (m)', ...
	  'Cyan (c)','Yellow (y)', 'Black (k)','Cancel'};
colors = {'r','g','b','m','c','y','k'};
k = menu('Change Color', labels);

if k <= length(colors)
  i = handles.ud.currentMap;
  handles.ud.colors(i) = colors{k};
  handles.ud.style(i,1)= colors{k};
  
  guidata(handles.fig,handles);
  
  displayMap(i, handles);
end





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NON-Callback Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function initControls(handles, gmap)
  nmap = size(gmap.maps,2);

  set(handles.sliderMap,'Min',1);  set(handles.sliderMap,'Max',nmap);
  set(handles.sliderMap,'SliderStep',[1/nmap 1/nmap]);
  set(handles.sliderMap,'Value',1);

  set(handles.txtMin,'String','1');
  set(handles.txtMax,'String',nmap);

  ud.gmap      = gmap;
  ud.mm        = gmap.maps;
  ud.trans     = gmap.transitions;
  ud.links     = gmap.links;
  ud.map_poses = gmap.map_poses;
  
  %Init colors
  raw_colors = 'rgbmcy';
  raw_style  = 'o*sd^h';
  ind = 1+ mod( 0:1:(nmap-1), length(raw_colors) );
  ud.colors  = raw_colors(ind);
  ud.style   = [raw_colors(ind)', raw_style(ind)'];
  
  ud.plotOdo    = 1;
  
  ud.currentMap = 1;
  
  
  handles.ud    = ud;
  guidata(gcf, handles);
  
  displayMap(1, handles);


function displayMap(mapId, handles)

  ud = handles.ud;
  c = ud.colors(mapId);
  
  cla;

  odo0 = ud.map_poses(mapId,1:3);
  mm   = translate_map(ud.mm(mapId),odo0);
  
  
  %Plot grid
  h = plotGrid(mm.grid, odo0, c); 
  hold on

 
  %Compute Neighbours
  i1 = find(ud.links(:,1) == mapId);
  i2 = find(ud.links(:,2) == mapId);
  ind_t = union(i1,i2);
  
  %Plot transitions
  plotTransitions(ud.trans, ind_t, mapId, ud.mm, odo0, ud.colors);

  
  %Plot current map
  ind_core = find(mm.map(:,size(mm.map,2)) == 3);
  ind_per  = find(mm.map(:,size(mm.map,2)) ~= 3);
    
  [mapCore,mapPer] = splitMap(mm.map); 
  
  h1 = mplot(mapCore,[c 'o'], [c '-']);
  h2 = mplot(mapPer, [c '+'], [c '-']);
  set(h1,'MarkerSize',10);
  set(h2,'MarkerSize',10);
  txt = num2str([1:size(mm.map,1)]');
  h = text(mm.map(:,1), mm.map(:,2), txt);
  set(h,'Color',c,'FontSize',15);

  if ud.plotOdo
    plot_robot(mm.odo,c);
  end
  
  %plot coordinate frame
  h = plotCoord(odo0,ud.colors(mapId),0.6); set(h,'LineWidth',2);
  
  
  
  axis equal
  handles.ud.currentMap = mapId;
  guidata(handles.fig, handles);


function h = plotTransitions(tr,tr_ind, mapId, mm, odo0,colors)
  h = [];

  for  i = 1:length(tr_ind)
    t = tr(tr_ind(i));
    
    if t.map1 == mapId
      odo = translate_odo(t.odo, odo0);
      otherMapId = t.map2;
    else
      odo = invert_transition(t.odo);
      odo = translate_odo(odo,odo0);
      otherMapId = t.map1;
    end
    
    c = colors(otherMapId);
    h = [h; plot_robot(odo, c)];
    
    [v,imax] = max(t.w);
    modo = odo(imax,1:3);
    
    h  = [h; plotGrid(mm(otherMapId).grid, modo, c)];
    ht = text(modo(1), modo(2),sprintf('%02d',otherMapId));
    set(ht,'Color',c); set(ht,'FontSize',17);
        
    set(ht,'UserData',otherMapId);
    set(ht,'ButtonDownFcn',@GotoMap_ButtonDownFcn);
    
    
    h = [h;ht];
    h = [h; plotCoord(modo,c,0.5)];
    h = [h; plotNeighbour(mm(otherMapId),modo,c)];
  end
  
function h = plotNeighbour(mm, odo0,color)
  map = translate_obs(mm.map, odo0);
  [mc,mp] = splitMap(map);
  
  h = mplot(mc,[color(1) 'o'], [color(1) '-']);
  h = [h; mplot(mp,[color(1) '+'], [color(1) '-'])];
%  h = [h; plotConvex(mc,color(1))];
  str = num2str([1:size(map,1)]');
  ht = text(map(:,1),map(:,2),str);
  set(ht,'Color',color(1));
  h = [h;ht];

function h = plotGrid(regGrid, odo0, c)
  if isempty(regGrid.data)
    h = [];
    return;
  end
  
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
  
  h = plot(xx,yy,[c '.']);
  ccc = get(h,'Color');  set(h,'Color',ccc*0.7);
  set(h,'MarkerSize',1);


  

function [core,per] = splitMap(map)
  ind_core = find(map(:,size(map,2)) == 3);
  ind_per  = find(map(:,size(map,2)) ~= 3);
    
  core = map(ind_core,:);
  per  = map(ind_per,:);


function o = invert_transition(odo)
  for i = 1:size(odo,1)
    pose = [-odo(i,1:2),0];
    o(i,:) = translate_odo(pose, [0,0,-odo(i,3)]);
  end
  

function h  = plotConvex(map,color)
  if size(map,1) > 2
    k = convhull(map(:,1),map(:,2));
    h = line(map(k,1), map(k,2));
    set(h,'Color',color);
  else
    h = [];
  end

function h = plotCoord(odo,color, r)
  odo      = [odo; odo; odo];
  odo(2,3) = odo(2,3) - pi/2;
  odo(3,3) = odo(3,3) + pi/2;
  h = plot_robot(odo, color, r);

function GotoMap_ButtonDownFcn(hObject,moreData)
  handles = guidata(hObject);
  
  mapId = get(hObject,'UserData');
  
  displayMap(mapId, handles);
  set(handles.sliderMap, 'Value', mapId);
  set(handles.txtCurrentMap,'String', mapId);
  


