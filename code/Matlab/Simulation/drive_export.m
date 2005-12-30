function varargout = drive_export(varargin)
% DRIVE_EXPORT M-file for drive_export.fig
%      DRIVE_EXPORT, by itself, creates a new DRIVE_EXPORT or raises the existing
%      singleton*.
%
%      H = DRIVE_EXPORT returns the handle to a new DRIVE_EXPORT or the handle to
%      the existing singleton*.
%
%      DRIVE_EXPORT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DRIVE_EXPORT.M with the given input arguments.
%
%      DRIVE_EXPORT('Property','Value',...) creates a new DRIVE_EXPORT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before drive_export_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to drive_export_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help drive_export

% Last Modified by GUIDE v2.5 28-Dec-2004 17:51:06

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @drive_export_OpeningFcn, ...
                   'gui_OutputFcn',  @drive_export_OutputFcn, ...
                   'gui_LayoutFcn',  @drive_export_LayoutFcn, ...
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

% --- Executes just before drive_export is made visible.
function drive_export_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to drive_export (see VARARGIN)

% Choose default command line output for drive_export
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% This sets up the initial plot - only do when we are invisible
% so window can get raised using drive_export.
if strcmp(get(hObject,'Visible'),'off')
  global DRIVE_UD
  global DRIVE_DATA
  
  
  DRIVE_DATA = zeros(30000, 6); % Time(s), Vt (m/s), Steer (rad), x,y,rot
  
  ud.x      = 0; % m
  ud.y      = 0; % m
  ud.a      = 0; % rad
  ud.Vt     = 0; % m/s
  ud.steer  = 0; % rad (steer angle)
  ud.acc    = 0; % m/s/s
  ud.t      = 0; % Continuous time (s)
  ud.k      = 0; % Discrete time (no units)
  
  ud.txt    = handles.textVt;
  ud.ax     = handles.axes1;
  ud.ax2    = handles.axes2;
  ud.fig    = handles.figure1;
  ud.hsteer = handles.slider_steer;
  
  if nargin > 3
    ud.beacons = varargin{1};
  else
    %Generate random beacons
    ud.beacons = 200*rand(100,2)-100;
    ud.beacons(:,3) = 0.15*rand(100,1) + 0.35;
  end
  
  if nargin > 4
    ud.lines = varargin{2};
  else
    ud.lines = zeros(0,4);
  end
  
  
  ud.view = 0;
  ud.stop = 1;
  ud = setup_GUI(ud);
  
  set(ud.fig,'WindowButtonMotionFcn',@mouse_callback);
  
  ud.timer = timer('TimerFcn',@timer_callback,'Period',0.2, ...
		   'ExecutionMode','fixedrate');
  
  plot_state(ud);
  set_ud(ud);

end

% UIWAIT makes drive_export wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = drive_export_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure


% --- Executes on key press over figure1 with no controls selected.
function figure1_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

k = get(hObject,'CurrentCharacter');
%disp(sprintf('Key Press: %d',k));
ud = get_ud;

switch(k)
 case {27,13,32}
   if ud.stop
     ud.stop = 0;
     start(ud.timer);
   else
     ud.stop = 1;
     stop(ud.timer);
     set(ud.txt,'String','[PAUSED]');
   end
   
   set_ud(ud);
 otherwise
end


function timer_callback(varargin)
global DRIVE_DATA

DT = 0.2;
ud = get_ud;

% Save control
ud.k = ud.k + 1;
ud.t = ud.t + DT;

DRIVE_DATA(ud.k, 1:3) = [ud.t, ud.Vt, ud.steer];
DRIVE_DATA(ud.k, 4:6) = [ud.x, ud.y, ud.a];


%Update state
if ud.Vt ~= 0 || ud.acc ~= 0
  ud     = move_robot(ud,DT, 1);
end


% GUI output
plot_state(ud);
set(ud.txt,'String',sprintf('Vt = %7.2f m/s %7.2f km/h %04d', ...
			    ud.Vt, ud.Vt*3.6, ud.k));
set(ud.hsteer,'value', -ud.steer/(pi/4));

% Save updated state
set_ud(ud);



function mouse_callback(obj,eventdata)
VT_MAX = 27;
ud = get_ud;
point = get(obj,'CurrentPoint');
position = get(obj,'Position');

x = point(1)/position(3)*2 - 1;
ud.steer = -x*pi/4;

%y = point(2)/position(4)*2 - 0.3;
%disp(sprintf('Mouse: %9.3f, %9.3f',x,y));
%ud.Vt = y*VT_MAX;

set_ud(ud);

function [ud] = setup_GUI(ud)
  axis(ud.ax2,[-100, 100, -100, 100],'equal');
  if ~isfield(ud,'hpath')
     axes(ud.ax2);
     ud.hpath = plot(0,0,'g.');
     set(ud.hpath,'MarkerSize',2);
     hold on
  end
  

  if ud.view == 0
    figure(ud.fig); axes(ud.ax);
    axis(ud.ax, [-100, 100, -100, 100],'equal');
    ud.hrobot = plot_robot([],ud.x,ud.y, ud.a);
    hold on
    ud.hbeac  = plot_circle(ud.beacons(:,1:2),ud.beacons(:,3),'r');
    ud.hlines = line(ud.lines(:,[1,3])', ud.lines(:,[2,4])');
    
  elseif ud.view == 1
    figure(ud.fig); axes(ud.ax);
    axis(ud.ax, [-55, 55, -10, 100]); axis equal
    ud.hrobot = plot_robot([],0,0,pi/2);
    hold on
    ud.hbeac  = plot(0,0,'ro');
    ud.hlines = line(zeros(2,size(ud.lines,1)),zeros(2,size(ud.lines,1)));
    
    axes(ud.ax2);
    line(ud.lines(:,[1,3])', ud.lines(:,[2,4])');
    plot_circle(ud.beacons(:,1:2),ud.beacons(:,3),'r');
  end
  
  
function ud = changeView(ud)
  delete(ud.hrobot);
  delete(ud.hbeac);
  delete(ud.hlines);
  
  ud.view = ~(ud.view);
  
  ud = setup_GUI(ud);
  
  plot_state(ud);
  
  axes(ud.ax2);

function plot_state(ud)
  figure(ud.fig); axes(ud.ax);
  ax = axis(ud.ax);
  
  if ud.view == 0
    plot_robot(ud.hrobot, ud.x,ud.y,ud.a);
    axis(ax);
  elseif ud.view == 1
    global DRIVE_DATA
    axes(ud.ax);
    beac = ud.beacons;
    lines = ud.lines;
    
    beac(:,1) = beac(:,1) - ud.x;
    beac(:,2) = beac(:,2) - ud.y;

    ca = cos(ud.a-pi/2); sa = sin(ud.a-pi/2);
    rot = [ca , -sa; 
	   sa,  ca];
    
    beac(:,1:2) = beac(:,1:2)*rot;
    lines(:,[1,3]) = lines(:,[1,3]) - ud.x;
    lines(:,[2,4]) = lines(:,[2,4]) - ud.y;
    lines(:,1:2) = lines(:,1:2)*rot;
    lines(:,3:4) = lines(:,3:4)*rot;

    set(ud.hbeac,'XData',beac(:,1), ...
		 'YData',beac(:,2));
    
    for i = 1:length(ud.hlines)
      set(ud.hlines(i),'XData',lines(i,[1,3]), 'YData',lines(i,[2,4]));
    end
    
      
    axis(ax);

    axes(ud.ax2);
    set(ud.hpath,'XData',DRIVE_DATA(:,4),'YData', DRIVE_DATA(:,5));
  end

  


function h = plot_robot(h, x,y,a)
rxx = [ 0     2     3     2     0     0;   
	1     1     0    -1    -1     1];
R = [cos(a), -sin(a); 
     sin(a), cos(a)];
rxx = R*rxx;
rxx(1,:) = rxx(1,:) + x;
rxx(2,:) = rxx(2,:) + y;

if isempty(h)
  h = line(rxx(1,:),rxx(2,:));
else
  set(h,'XData',rxx(1,:),'YData',rxx(2,:));
end


function ud = move_robot(ud0,DT,add_noise)
L  = 3.0;
ud = ud0;

sigma_v = 0.1*abs(ud0.Vt) + 0.05;
sigma_steer = 0.5*pi/180;

if add_noise
  Vc    = ud0.Vt    + randn(1)*sigma_v;
  steer = ud0.steer + randn(1)*sigma_steer;
else
  Vc    = ud0.Vt;
  steer = ud0.steer;
end

a     = ud0.a;

ud.x = ud0.x + DT*Vc*cos(a);
ud.y = ud0.y + DT*Vc*sin(a);
ud.a = ud0.a + DT*Vc*tan(steer)/L;

if add_noise
  ud.x = ud.x + randn(1)*0.1;
  ud.y = ud.y + randn(1)*0.1;
  ud.a = ud.a + randn(1)*(0.3*pi/180);
end


%Update the velocity
sigma_acc = 0.1*abs(ud0.acc) + 0.05;
ud.Vt = ud.Vt + DT*(ud0.acc + randn(1)*sigma_acc);

function ud = get_ud(handles)
  global DRIVE_UD
  ud = DRIVE_UD;


function set_ud(ud,handles)
  global DRIVE_UD
  DRIVE_UD = ud;


% --- Executes during object deletion, before destroying properties.
function figure1_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ud = get_ud;

if ~isempty(ud) && isfield(ud,'timer')
  stop(ud.timer);
  delete(ud.timer);
  ud.timer = [];
  set_ud(ud);
end




% --- Executes during object creation, after setting all properties.
function slider_steer_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_steer (see GCBO)
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
function slider_steer_Callback(hObject, eventdata, handles)
% hObject    handle to slider_steer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

MAX_STEER = pi/4;

v = get(hObject,'Value');
ud = get_ud;
ud.steer = -v*MAX_STEER;
set_ud(ud);


% --- Executes on mouse press over figure background, over a disabled or
% --- inactive control, or over an axes background.
function figure1_WindowButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

st = get(hObject,'SelectionType');

ud = get_ud;

switch st
   case 'normal'
       ud.acc = 1; %m/s/s

   case 'alt'
       ud.acc = -1; %m/s/s
       
   case 'extend'
       ud.Vt = 0;
       ud.acc = 0;
       
 otherwise
  ud = changeView(ud);
  
end

set_ud(ud);
       


% --- Executes on mouse press over figure background, over a disabled or
% --- inactive control, or over an axes background.
function figure1_WindowButtonUpFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
st = get(hObject,'SelectionType');
ud = get_ud;

ud.acc = 0;

set_ud(ud);


% --- Executes on button press in btnPause.
function btnPause_Callback(hObject, eventdata, handles)
% hObject    handle to btnPause (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ud = get_ud;

if ud.stop
  ud.stop = 0;
  start(ud.timer);
else
  ud.stop = 1;
  stop(ud.timer);
  set(ud.txt,'String','[PAUSED]');
end

set_ud(ud);
   

% --- Executes on button press in btnResetOdo.
function btnResetOdo_Callback(hObject, eventdata, handles)
% hObject    handle to btnResetOdo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ud = get_ud;
ud.k = 0;
ud.t = 0;
set_ud(ud);

% --- Executes on button press in btnCahngeView.
function btnCahngeView_Callback(hObject, eventdata, handles)
% hObject    handle to btnCahngeView (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

ud = get_ud;
ud = changeView(ud);
set_ud(ud);


% --- Creates and returns a handle to the GUI figure. 
function h1 = drive_export_LayoutFcn(policy)
% policy - create a new figure or use a singleton. 'new' or 'reuse'.

persistent hsingleton;
if strcmpi(policy, 'reuse') & ishandle(hsingleton)
    h1 = hsingleton;
    return;
end

h1 = figure(...
'Units','characters',...
'PaperUnits',get(0,'defaultfigurePaperUnits'),...
'Color',[0.701960784313725 0.701960784313725 0.701960784313725],...
'Colormap',[0 0 0.5625;0 0 0.625;0 0 0.6875;0 0 0.75;0 0 0.8125;0 0 0.875;0 0 0.9375;0 0 1;0 0.0625 1;0 0.125 1;0 0.1875 1;0 0.25 1;0 0.3125 1;0 0.375 1;0 0.4375 1;0 0.5 1;0 0.5625 1;0 0.625 1;0 0.6875 1;0 0.75 1;0 0.8125 1;0 0.875 1;0 0.9375 1;0 1 1;0.0625 1 1;0.125 1 0.9375;0.1875 1 0.875;0.25 1 0.8125;0.3125 1 0.75;0.375 1 0.6875;0.4375 1 0.625;0.5 1 0.5625;0.5625 1 0.5;0.625 1 0.4375;0.6875 1 0.375;0.75 1 0.3125;0.8125 1 0.25;0.875 1 0.1875;0.9375 1 0.125;1 1 0.0625;1 1 0;1 0.9375 0;1 0.875 0;1 0.8125 0;1 0.75 0;1 0.6875 0;1 0.625 0;1 0.5625 0;1 0.5 0;1 0.4375 0;1 0.375 0;1 0.3125 0;1 0.25 0;1 0.1875 0;1 0.125 0;1 0.0625 0;1 0 0;0.9375 0 0;0.875 0 0;0.8125 0 0;0.75 0 0;0.6875 0 0;0.625 0 0;0.5625 0 0],...
'DoubleBuffer','on',...
'InvertHardcopy',get(0,'defaultfigureInvertHardcopy'),...
'KeyPressFcn','drive_export(''figure1_KeyPressFcn'',gcbo,[],guidata(gcbo))',...
'MenuBar','none',...
'Name','Untitled',...
'NumberTitle','off',...
'PaperPosition',get(0,'defaultfigurePaperPosition'),...
'PaperSize',[20.98404194812 29.67743169791],...
'PaperType',get(0,'defaultfigurePaperType'),...
'Position',[128.6 -10.978021978022 181.666666666667 65.2857142857143],...
'WindowButtonDownFcn','drive_export(''figure1_WindowButtonDownFcn'',gcbo,[],guidata(gcbo))',...
'WindowButtonUpFcn','drive_export(''figure1_WindowButtonUpFcn'',gcbo,[],guidata(gcbo))',...
'ButtonDownFcn','drive_export(''figure1_ButtonDownFcn'',gcbo,[],guidata(gcbo))',...
'DeleteFcn','drive_export(''figure1_DeleteFcn'',gcbo,[],guidata(gcbo))',...
'Tag','figure1',...
'UserData',zeros(1,0));

setappdata(h1, 'GUIDEOptions', struct(...
'active_h', 1, ...
'taginfo', struct(...
'figure', 2, ...
'axes', 4, ...
'pushbutton', 6, ...
'popupmenu', 2, ...
'frame', 2, ...
'checkbox', 2, ...
'slider', 3, ...
'text', 2), ...
'override', 1, ...
'release', 13, ...
'resize', 'simple', ...
'accessibility', 'on', ...
'mfile', 1, ...
'callbacks', 1, ...
'singleton', 1, ...
'syscolorfig', 1, ...
'lastSavedFile', '/home/users/kirill/SW/ShinySLAM/Matlab/Simulation/drive.m'));


h2 = uimenu(...
'Parent',h1,...
'Callback','drive_export(''FileMenu_Callback'',gcbo,[],guidata(gcbo))',...
'Label','File',...
'Tag','FileMenu');

h3 = uimenu(...
'Parent',h2,...
'Callback','drive_export(''OpenMenuItem_Callback'',gcbo,[],guidata(gcbo))',...
'Label','Open ...',...
'Tag','OpenMenuItem');

h4 = uimenu(...
'Parent',h2,...
'Callback','drive_export(''PrintMenuItem_Callback'',gcbo,[],guidata(gcbo))',...
'Label','Print ...',...
'Tag','PrintMenuItem');

h5 = uimenu(...
'Parent',h2,...
'Callback','drive_export(''CloseMenuItem_Callback'',gcbo,[],guidata(gcbo))',...
'Label','Close',...
'Separator','on',...
'Tag','CloseMenuItem');

h6 = axes(...
'Parent',h1,...
'Box','on',...
'CameraPosition',[0.5 0.5 9.16025403784439],...
'CameraPositionMode',get(0,'defaultaxesCameraPositionMode'),...
'Color',get(0,'defaultaxesColor'),...
'ColorOrder',get(0,'defaultaxesColorOrder'),...
'Position',[0.044954128440367 0.12144420131291 0.918348623853211 0.855579868708972],...
'XColor',get(0,'defaultaxesXColor'),...
'YColor',get(0,'defaultaxesYColor'),...
'ZColor',get(0,'defaultaxesZColor'),...
'Tag','axes1');


h7 = get(h6,'title');

set(h7,...
'Parent',h6,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
'Position',[0.5 1.00703324808184 1.00005459937205],...
'VerticalAlignment','bottom',...
'HandleVisibility','off');

h8 = get(h6,'xlabel');

set(h8,...
'Parent',h6,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
'Position',[0.499000999000999 -0.0287723785166238 1.00005459937205],...
'VerticalAlignment','cap',...
'HandleVisibility','off');

h9 = get(h6,'ylabel');

set(h9,...
'Parent',h6,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
'Position',[-0.0274725274725275 0.498081841432225 1.00005459937205],...
'Rotation',90,...
'VerticalAlignment','bottom',...
'HandleVisibility','off');

h10 = get(h6,'zlabel');

set(h10,...
'Parent',h6,...
'Color',[0 0 0],...
'HorizontalAlignment','right',...
'Position',[-0.0494505494505494 1.02493606138107 1.00005459937205],...
'HandleVisibility','off',...
'Visible','off');

h11 = uicontrol(...
'Parent',h1,...
'Units','normalized',...
'BackgroundColor',[0.9 0.9 0.9],...
'Callback','drive_export(''slider_steer_Callback'',gcbo,[],guidata(gcbo))',...
'Min',-1,...
'Position',[0.136697247706422 0.0667396061269147 0.734862385321101 0.0350109409190372],...
'String',{ '' },...
'Style','slider',...
'CreateFcn','drive_export(''slider_steer_CreateFcn'',gcbo,[],guidata(gcbo))',...
'Tag','slider_steer');


h12 = uicontrol(...
'Parent',h1,...
'Units','normalized',...
'Position',[0.411926605504587 0.012035010940919 0.184403669724771 0.0382932166301969],...
'String','Press Space to start.',...
'Style','text',...
'Tag','textVt');


h13 = axes(...
'Parent',h1,...
'CameraPosition',[0.5 0.5 9.16025403784439],...
'CameraPositionMode',get(0,'defaultaxesCameraPositionMode'),...
'Color',get(0,'defaultaxesColor'),...
'ColorOrder',get(0,'defaultaxesColorOrder'),...
'Position',[0.778899082568807 0.777899343544857 0.184403669724771 0.198030634573304],...
'XColor',get(0,'defaultaxesXColor'),...
'YColor',get(0,'defaultaxesYColor'),...
'ZColor',get(0,'defaultaxesZColor'),...
'Tag','axes2');


h14 = get(h13,'title');

set(h14,...
'Parent',h13,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
'Position',[0.5 1.03038674033149 1.00005459937205],...
'VerticalAlignment','bottom',...
'HandleVisibility','off');

h15 = get(h13,'xlabel');

set(h15,...
'Parent',h13,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
'Position',[0.495024875621891 -0.124309392265193 1.00005459937205],...
'VerticalAlignment','cap',...
'HandleVisibility','off');

h16 = get(h13,'ylabel');

set(h16,...
'Parent',h13,...
'Color',[0 0 0],...
'HorizontalAlignment','center',...
'Position',[-0.13681592039801 0.494475138121547 1.00005459937205],...
'Rotation',90,...
'VerticalAlignment','bottom',...
'HandleVisibility','off');

h17 = get(h13,'zlabel');

set(h17,...
'Parent',h13,...
'Color',[0 0 0],...
'HorizontalAlignment','right',...
'Position',[-4.22636815920398 1.11325966850829 1.00005459937205],...
'HandleVisibility','off',...
'Visible','off');

h18 = uicontrol(...
'Parent',h1,...
'Units','normalized',...
'Callback','drive_export(''btnPause_Callback'',gcbo,[],guidata(gcbo))',...
'Position',[0.0174311926605505 0.012035010940919 0.08 0.03],...
'String','Start',...
'Tag','btnPause');


h19 = uicontrol(...
'Parent',h1,...
'Units','normalized',...
'Callback','drive_export(''btnResetOdo_Callback'',gcbo,[],guidata(gcbo))',...
'Position',[0.108256880733945 0.012035010940919 0.08 0.03],...
'String','Reset Odo',...
'Tag','btnResetOdo');


h20 = uicontrol(...
'Parent',h1,...
'Units','normalized',...
'Callback','drive_export(''btnCahngeView_Callback'',gcbo,[],guidata(gcbo))',...
'Position',[0.196330275229358 0.012035010940919 0.08 0.03],...
'String','ChangeView',...
'Tag','btnCahngeView');



hsingleton = h1;


% --- Handles default GUIDE GUI creation and callback dispatch
function varargout = gui_mainfcn(gui_State, varargin)


%   GUI_MAINFCN provides these command line APIs for dealing with GUIs
%
%      DRIVE_EXPORT, by itself, creates a new DRIVE_EXPORT or raises the existing
%      singleton*.
%
%      H = DRIVE_EXPORT returns the handle to a new DRIVE_EXPORT or the handle to
%      the existing singleton*.
%
%      DRIVE_EXPORT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DRIVE_EXPORT.M with the given input arguments.
%
%      DRIVE_EXPORT('Property','Value',...) creates a new DRIVE_EXPORT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before untitled_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to untitled_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".

%   Copyright 1984-2002 The MathWorks, Inc.
%   $Revision: 1.4 $ $Date: 2002/05/31 21:44:31 $

gui_StateFields =  {'gui_Name'
                    'gui_Singleton'
                    'gui_OpeningFcn'
                    'gui_OutputFcn'
                    'gui_LayoutFcn'
                    'gui_Callback'};
gui_Mfile = '';
for i=1:length(gui_StateFields)
    if ~isfield(gui_State, gui_StateFields{i})
        error('Could not find field %s in the gui_State struct in GUI M-file %s', gui_StateFields{i}, gui_Mfile);        
    elseif isequal(gui_StateFields{i}, 'gui_Name')
        gui_Mfile = [getfield(gui_State, gui_StateFields{i}), '.m'];
    end
end

numargin = length(varargin);

if numargin == 0
    % DRIVE_EXPORT
    % create the GUI
    gui_Create = 1;
elseif numargin > 3 & ischar(varargin{1}) & ishandle(varargin{2})
    % DRIVE_EXPORT('CALLBACK',hObject,eventData,handles,...)
    gui_Create = 0;
else
    % DRIVE_EXPORT(...)
    % create the GUI and hand varargin to the openingfcn
    gui_Create = 1;
end

if gui_Create == 0
    varargin{1} = gui_State.gui_Callback;
    if nargout
        [varargout{1:nargout}] = feval(varargin{:});
    else
        feval(varargin{:});
    end
else
    if gui_State.gui_Singleton
        gui_SingletonOpt = 'reuse';
    else
        gui_SingletonOpt = 'new';
    end
    
    % Open fig file with stored settings.  Note: This executes all component
    % specific CreateFunctions with an empty HANDLES structure.
    
    % Do feval on layout code in m-file if it exists
    if ~isempty(gui_State.gui_LayoutFcn)
        gui_hFigure = feval(gui_State.gui_LayoutFcn, gui_SingletonOpt);
    else
        gui_hFigure = local_openfig(gui_State.gui_Name, gui_SingletonOpt);            
        % If the figure has InGUIInitialization it was not completely created
        % on the last pass.  Delete this handle and try again.
        if isappdata(gui_hFigure, 'InGUIInitialization')
            delete(gui_hFigure);
            gui_hFigure = local_openfig(gui_State.gui_Name, gui_SingletonOpt);            
        end
    end
    
    % Set flag to indicate starting GUI initialization
    setappdata(gui_hFigure,'InGUIInitialization',1);

    % Fetch GUIDE Application options
    gui_Options = getappdata(gui_hFigure,'GUIDEOptions');
    
    if ~isappdata(gui_hFigure,'GUIOnScreen')
        % Adjust background color
        if gui_Options.syscolorfig 
            set(gui_hFigure,'Color', get(0,'DefaultUicontrolBackgroundColor'));
        end

        % Generate HANDLES structure and store with GUIDATA
        guidata(gui_hFigure, guihandles(gui_hFigure));
    end
    
    % If user specified 'Visible','off' in p/v pairs, don't make the figure
    % visible.
    gui_MakeVisible = 1;
    for ind=1:2:length(varargin)
        if length(varargin) == ind
            break;
        end
        len1 = min(length('visible'),length(varargin{ind}));
        len2 = min(length('off'),length(varargin{ind+1}));
        if ischar(varargin{ind}) & ischar(varargin{ind+1}) & ...
                strncmpi(varargin{ind},'visible',len1) & len2 > 1
            if strncmpi(varargin{ind+1},'off',len2)
                gui_MakeVisible = 0;
            elseif strncmpi(varargin{ind+1},'on',len2)
                gui_MakeVisible = 1;
            end
        end
    end
    
    % Check for figure param value pairs
    for index=1:2:length(varargin)
        if length(varargin) == index
            break;
        end
        try, set(gui_hFigure, varargin{index}, varargin{index+1}), catch, break, end
    end

    % If handle visibility is set to 'callback', turn it on until finished
    % with OpeningFcn
    gui_HandleVisibility = get(gui_hFigure,'HandleVisibility');
    if strcmp(gui_HandleVisibility, 'callback')
        set(gui_hFigure,'HandleVisibility', 'on');
    end
    
    feval(gui_State.gui_OpeningFcn, gui_hFigure, [], guidata(gui_hFigure), varargin{:});
    
    if ishandle(gui_hFigure)
        % Update handle visibility
        set(gui_hFigure,'HandleVisibility', gui_HandleVisibility);
        
        % Make figure visible
        if gui_MakeVisible
            set(gui_hFigure, 'Visible', 'on')
            if gui_Options.singleton 
                setappdata(gui_hFigure,'GUIOnScreen', 1);
            end
        end

        % Done with GUI initialization
        rmappdata(gui_hFigure,'InGUIInitialization');
    end
    
    % If handle visibility is set to 'callback', turn it on until finished with
    % OutputFcn
    if ishandle(gui_hFigure)
        gui_HandleVisibility = get(gui_hFigure,'HandleVisibility');
        if strcmp(gui_HandleVisibility, 'callback')
            set(gui_hFigure,'HandleVisibility', 'on');
        end
        gui_Handles = guidata(gui_hFigure);
    else
        gui_Handles = [];
    end
    
    if nargout
        [varargout{1:nargout}] = feval(gui_State.gui_OutputFcn, gui_hFigure, [], gui_Handles);
    else
        feval(gui_State.gui_OutputFcn, gui_hFigure, [], gui_Handles);
    end
    
    if ishandle(gui_hFigure)
        set(gui_hFigure,'HandleVisibility', gui_HandleVisibility);
    end
end    

function gui_hFigure = local_openfig(name, singleton)
if nargin('openfig') == 3 
    gui_hFigure = openfig(name, singleton, 'auto');
else
    % OPENFIG did not accept 3rd input argument until R13,
    % toggle default figure visible to prevent the figure
    % from showing up too soon.
    gui_OldDefaultVisible = get(0,'defaultFigureVisible');
    set(0,'defaultFigureVisible','off');
    gui_hFigure = openfig(name, singleton);
    set(0,'defaultFigureVisible',gui_OldDefaultVisible);
end

