function varargout = mapInspector(varargin)
% mapInspector(mm,trans,links,map_poses)

% Edit the above text to modify the response to help mapInspector

% Last Modified by GUIDE v2.5 06-May-2004 17:19:28

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @mapInspector_OpeningFcn, ...
                   'gui_OutputFcn',  @mapInspector_OutputFcn, ...
                   'gui_LayoutFcn',  [], ...
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


% --- Executes just before mapInspector is made visible.
function mapInspector_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for mapInspector
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes mapInspector wait for user response (see UIRESUME)
% uiwait(handles.figure1);

if ~isempty(varargin)
  mm        = varargin{1};
  trans     = varargin{2};
  links     = varargin{3};
  map_poses = varargin{4};

  initControls(handles, mm, trans, links, map_poses);
end


% --- Outputs from this function are returned to the command line.
function varargout = mapInspector_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes when fig window is resized.
function fig_ResizeFcn(hObject, eventdata, handles)
% hObject    handle to fig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on mouse press over axes background.
function axes1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%get(hObject,'CurrentPoint')


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


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over sliderMap.
function sliderMap_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to sliderMap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


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


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over btnShowAll.
function btnShowAll_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to btnShowAll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in btnShowAll.
function btnShowAll_Callback(hObject, eventdata, handles)
% hObject    handle to btnShowAll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
  figure;
  plot_map_ref(handles.ud.mm, handles.ud.map_poses(:,1:3));
  axis equal

function btnZoom_Callback(hObject, eventdata)
  disp('Zoom');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NON-Callback Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function initControls(handles, mm, trans, links, map_poses)
  nmap = size(mm,2);

  set(handles.sliderMap,'Min',1);  set(handles.sliderMap,'Max',nmap);
  set(handles.sliderMap,'SliderStep',[1/nmap 1/nmap]);
  set(handles.sliderMap,'Value',1);

  set(handles.txtMin,'String','1');
  set(handles.txtMax,'String',nmap);
  
  ud.mm        = mm;
  ud.trans     = trans;
  ud.links     = links;
  ud.map_poses = map_poses;
  
  ud.hMapCore  = [];
  ud.hBoundary = [];
  ud.hMapPer   = [];
  ud.hOdo      = [];
  ud.hTrans    = [];
  
  ud.coreStyle  = 'ror-';
  ud.perStyle   = 'rsr:';
  
  ud.odoStyle   = 'r';
  ud.odoR       = 0.01;
  
  ud.plotOdo    = 1;
  
  ud.currentMap = 1;
  
  
  handles.ud    = ud;
  guidata(gcf, handles);
  
  displayMap(1, handles);


function displayMap(mapId, handles)
  ud = handles.ud;

  cla;
% $$$   delete(ud.hMapCore);
% $$$   delete(ud.hBoundary);
% $$$   delete(ud.hMapPer);
% $$$   delete(ud.hOdo);
% $$$   delete(ud.hTrans);
  
  odo0 = ud.map_poses(mapId,1:3);
  mm   = translate_map(ud.mm(mapId),odo0);
  
  ind_core = find(mm.map(:,size(mm.map,2)) == 3);
  ind_per  = find(mm.map(:,size(mm.map,2)) ~= 3);
    
  [mapCore,mapPer] = splitMap(mm.map); 
  
  hold on
  handles.ud.hMapCore = mplot(mapCore, ud.coreStyle(1:2), ud.coreStyle(3:4));
  handles.ud.hMapPer  = mplot(mapPer,  ud.perStyle(1:2),  ud.perStyle(3:4));

  %plot Convex
  handles.ud.hBoundary = plotConvex(mapCore,ud.coreStyle(1));
  set(handles.ud.hBoundary,'LineWidth',3);
  
  if ud.plotOdo
      handles.ud.hOdo = plot_robot(mm.odo,ud.odoStyle, ud.odoR);
  end
  
  %plot coordinate frame
  h = plotCoord(odo0,ud.odoStyle,0.6); set(h,'LineWidth',2);
  
  %Compute Neighbours
  colors = 'bgmcyk';

  i1 = find(ud.links(:,1) == mapId);
  i2 = find(ud.links(:,2) == mapId);
  
  %Plot transitions
  ind_t = union(i1,i2);
  handles.ud.hTrans = plotTransitions(ud.trans, ind_t, mapId ...
				      ,ud.mm, odo0, colors);
  
  axis equal
  handles.ud.currentMap = mapId;
  guidata(handles.fig, handles);


function h = plotTransitions(tr,tr_ind, mapId, mm, odo0,color)
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
    
    c = color(mod(i-1,length(color))+1);
    h = [h; plot_robot(odo, c)];
    
    [v,imax] = max(t.w);
    modo = odo(imax,1:3);
    
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
  h = [h; mplot(mp,[color(1) 's'], [color(1) '-'])];
  h = [h; plotConvex(mc,color(1))];
  str = num2str([1:size(map,1)]');
  ht = text(map(:,1),map(:,2),str);
  set(ht,'Color',color(1));
  h = [h;ht];
	

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
  
