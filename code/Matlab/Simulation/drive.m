function varargout = drive(varargin)
% DRIVE M-file for drive.fig
%      DRIVE, by itself, creates a new DRIVE or raises the existing
%      singleton*.
%
%      H = DRIVE returns the handle to a new DRIVE or the handle to
%      the existing singleton*.
%
%      DRIVE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DRIVE.M with the given input arguments.
%
%      DRIVE('Property','Value',...) creates a new DRIVE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before drive_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to drive_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help drive

% Last Modified by GUIDE v2.5 24-Sep-2004 16:38:10

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @drive_OpeningFcn, ...
                   'gui_OutputFcn',  @drive_OutputFcn, ...
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

% --- Executes just before drive is made visible.
function drive_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to drive (see VARARGIN)

% Choose default command line output for drive
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% This sets up the initial plot - only do when we are invisible
% so window can get raised using drive.
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

% UIWAIT makes drive wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = drive_OutputFcn(hObject, eventdata, handles)
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
