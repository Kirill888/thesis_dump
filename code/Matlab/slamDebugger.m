function varargout = slamDebugger(varargin)
% SLAMDEBUGGER M-file for slamDebugger.fig
%      SLAMDEBUGGER, by itself, creates a new SLAMDEBUGGER or raises the existing
%      singleton*.
%
%      H = SLAMDEBUGGER returns the handle to a new SLAMDEBUGGER or the handle to
%      the existing singleton*.
%
%      SLAMDEBUGGER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SLAMDEBUGGER.M with the given input arguments.
%
%      SLAMDEBUGGER('Property','Value',...) creates a new SLAMDEBUGGER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before slamDebugger_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to slamDebugger_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help slamDebugger

% Last Modified by GUIDE v2.5 30-Sep-2004 15:54:01

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @slamDebugger_OpeningFcn, ...
                   'gui_OutputFcn',  @slamDebugger_OutputFcn, ...
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

function displayStep(handles)
  ud = handles.ud;
  k = ud.ind;
  fname = ud.files{k};
  
  set(handles.txtTop,'String', sprintf('File: %s, %04d/%04d', ...
				       fname, k, ud.nf));
  set(handles.sliderInd,'Value', k);

  run(fname); %obs, odo, hype
  
  axMap = handles.axMap;
  axW   = handles.axWeight;
  axP   = handles.axPoses;
  
  %Plot map and obs an las
  axes(axMap);
  cla(axMap);
  hold on

  sensor = odo2sensor(odo(:,1:3), ud.SENSOR_POSE);
  [wmax,imax] = max(odo(:,4));
  
  if ~isempty(obs)
    plot_obs2d(obs, sensor, 'y.-');
    plot_obs2d(obs, sensor(imax,:),'b.-');
  end

  if ~isempty(las)
    lsx = las.*ud.las_ca;
    lsy = las.*ud.las_sa;
    laser = translate_obs([lsx',lsy'],sensor(imax,:));
    plot(laser(:,1),laser(:,2),'b.');
  end

  robot_x = [0.300,0.290,0.260,0.212,0.150,0.078,0.000,-0.078,...
	     -0.150,-0.212,-0.260,-0.290,-0.300,-0.290,-0.260,...
	     -0.212,-0.150,-0.078,-0.000,0.078,0.150,0.212,0.260,...
	     0.290,0.300,0.230,0.180,0.080,0.080,0.180,0.230];
  
  robot_y = [ 0.000,0.078,0.150,0.212,0.260,0.290,0.300,0.290, ...
	      0.260,0.212,0.150,0.078,0.000,-0.078,-0.150,-0.212, ...
	      -0.260,-0.290,-0.300,-0.290,-0.260,-0.212,-0.150,-0.078, ...
	      -0.000,0.000,0.070,0.070,-0.070,-0.070,0.000];

  plot_robot(odo(:,1:3),'g');
  
  rr = translate_obs([robot_x',robot_y'], odo(imax,1:3));
  
  plot(rr(:,1), rr(:,2),'b-','LineWidth',2);
  main_hype = hype(odo(imax,5));
  
  if ~isempty(main_hype.map)
    mplot(main_hype.map,'r.','r-');
  end

  axis equal;  

  %Plot poses
  axes(axP);
  cla(axP);
  plot_robot(odo(:,1:3),'g');
  %plot(odo(imax,1),odo(imax,2),'ro');
  axis equal

  %Plot weights
  axes(axW);
  cla(axW);
  plot(odo(:,4),'r.');


% --- Executes just before slamDebugger is made visible.
function slamDebugger_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to slamDebugger (see VARARGIN)

% Choose default command line output for slamDebugger
handles.output = hObject;

nargs = length(varargin);

if nargs > 0
  dd = dir(varargin{1});
else
  dd = dir('*.m');
end

nf = 0;
for i = 1:length(dd)
  if ~dd(i).isdir
    nf = nf + 1;
    ind = find(dd(i).name == '.');

    if isempty(ind)
      names{nf} = dd(i).name;
    else
      names{nf} = dd(i).name(1:(ind-1));
    end
  end
end

ud.files = names;
ud.nf    = nf;
ud.ind   = 1;
ud.SENSOR_POSE = [0.227, 0.024, 0];
ud.las_a = linspace(-pi/2,pi/2,361);
ud.las_ca = cos(ud.las_a);
ud.las_sa = sin(ud.las_a);

%Update controls
set(handles.sliderInd,'Min',1,'Max',nf,'SliderStep',[1/nf 1/nf], ...
		  'Value',1);


% Update handles structure
handles.ud = ud;
guidata(hObject, handles);

displayStep(handles);

% UIWAIT makes slamDebugger wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = slamDebugger_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes during object creation, after setting all properties.
function sliderInd_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sliderInd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: sliderInd controls usually have a light gray background, change
%       'usewhitebg' to 0 to use default.  See ISPC and COMPUTER.
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on sliderInd movement.
function sliderInd_Callback(hObject, eventdata, handles)
% hObject    handle to sliderInd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of sliderInd
%        get(hObject,'Min') and get(hObject,'Max') to determine
%        range of sliderInd

k = round(get(hObject,'Value'));
%set(hObject,'Value',k);

handles.ud.ind = k;
guidata(hObject, handles);

displayStep(handles);


% --- Executes on button press in btnExport.
function btnExport_Callback(hObject, eventdata, handles)
% hObject    handle to btnExport (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in btnZoom.
function btnZoom_Callback(hObject, eventdata, handles)
% hObject    handle to btnZoom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
v = get(hObject,'String');

switch(v)
 case 'Zoom'
  zoom on
  v = 'Zoom Off';
 case 'Zoom Off'
  zoom off
  v = 'Zoom';
end

set(hObject,'String',v);


% --- Executes on button press in btnFwd10.
function btnFwd10_Callback(hObject, eventdata, handles)
% hObject    handle to btnFwd10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

k = handles.ud.ind + 10;
if k > handles.ud.nf
  k = handles.ud.nf;
end

handles.ud.ind = k;
guidata(hObject, handles);

displayStep(handles);


% --- Executes on button press in btnBck10.
function btnBck10_Callback(hObject, eventdata, handles)
% hObject    handle to btnBck10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
k = handles.ud.ind - 10;

if k < 1
  k = 1;
end

handles.ud.ind = k;

guidata(hObject, handles);
displayStep(handles);


% --- Executes on button press in chkAll.
function chkAll_Callback(hObject, eventdata, handles)
% hObject    handle to chkAll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of chkAll


% --- Executes on button press in chkBest.
function chkBest_Callback(hObject, eventdata, handles)
% hObject    handle to chkBest (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of chkBest


