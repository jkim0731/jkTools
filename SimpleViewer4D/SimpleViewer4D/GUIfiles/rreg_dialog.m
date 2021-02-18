function varargout = rreg_dialog(varargin)
% RREG_DIALOG MATLAB code for rreg_dialog.fig
%      RREG_DIALOG, by itself, creates a new RREG_DIALOG or raises the existing
%      singleton*.
%
%      H = RREG_DIALOG returns the handle to a new RREG_DIALOG or the handle to
%      the existing singleton*.
%
%      RREG_DIALOG('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in RREG_DIALOG.M with the given input arguments.
%
%      RREG_DIALOG('Property','Value',...) creates a new RREG_DIALOG or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before rreg_dialog_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to rreg_dialog_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help rreg_dialog

% Last Modified by GUIDE v2.5 30-Jul-2014 14:14:50

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @rreg_dialog_OpeningFcn, ...
    'gui_OutputFcn',  @rreg_dialog_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before rreg_dialog is made visible.
function rreg_dialog_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to rreg_dialog (see VARARGIN)

% Choose default command line output for rreg_dialog
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% Get the father gui
global handles_father;
handles_father = guidata(varargin{1});
if 0
%if handles_father.lastParamsToBeUsed ==2
    % load from the normal registration
    tx = handles_father.regmatrix(1,4);
    ty = handles_father.regmatrix(2,4);
    tz = handles_father.regmatrix(3,4);
        
else
    set(handles.rreg_slider_x,'Value',handles_father.regparams(1));
    set(handles.rreg_text_x,'String', sprintf('%1.1f',handles_father.regparams(1)));
    set(handles.rreg_slider_y,'Value',handles_father.regparams(2));
    set(handles.rreg_text_y,'String', sprintf('%1.1f',handles_father.regparams(2)));
    set(handles.rreg_slider_z,'Value',handles_father.regparams(3));
    set(handles.rreg_text_z,'String', sprintf('%1.1f',handles_father.regparams(3)));
    set(handles.rreg_slider_rx,'Value',handles_father.regparams(4)*180/pi);
    set(handles.rreg_text_rx,'String', sprintf('%1.1f',handles_father.regparams(4)*180/pi));
    set(handles.rreg_slider_ry,'Value',handles_father.regparams(4)*180/pi);
    set(handles.rreg_text_ry,'String', sprintf('%1.1f',handles_father.regparams(5)*180/pi));
    set(handles.rreg_slider_rz,'Value',handles_father.regparams(6)*180/pi);
    set(handles.rreg_text_rz,'String', sprintf('%1.1f',handles_father.regparams(6)*180/pi));
end
handles_father.lastParamsToBeUsed=1;
guidata(varargin{1},handles_father);


% UIWAIT makes rreg_dialog wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = rreg_dialog_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function rreg_slider_x_Callback(hObject, eventdata, handles)
% hObject    handle to rreg_slider_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global handles_father;
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
val = get(hObject,'Value');
set(handles.rreg_text_x,'String', sprintf('%1.1f',val));

% make a change in the handles.father and trigger a callback
M = matrix_from_parameters(hObject,eventdata,handles);
handles_father.regmatrix=M;
guidata(handles_father.figure1,handles_father);
cb = get(handles_father.checkbox_overlay,'Callback');
cb(handles_father.checkbox_overlay,[]);

% the matrix is about the centre, let's compensate for that
c0 = handles_father.im{2}.GetGeometricalCenter();
M1 = eye(4);
M1(1:3,4)=c0;
p0 = parametersFromRigidMatrix(M1*M/M1)'


% --- Executes during object creation, after setting all properties.
function rreg_slider_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rreg_slider_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function rreg_slider_y_Callback(hObject, eventdata, handles)
% hObject    handle to rreg_slider_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global handles_father;
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
val = get(hObject,'Value');
set(handles.rreg_text_y,'String', sprintf('%1.1f',val));
M = matrix_from_parameters(hObject,eventdata,handles);
handles_father.regmatrix=M;
guidata(handles_father.figure1,handles_father);
cb = get(handles_father.checkbox_overlay,'Callback');
cb(handles_father.checkbox_overlay,[]);
c0 = handles_father.im{2}.GetGeometricalCenter();
M1 = eye(4);
M1(1:3,4)=c0;
p0 = parametersFromRigidMatrix(M1*M/M1)'

% --- Executes during object creation, after setting all properties.
function rreg_slider_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rreg_slider_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function rreg_slider_z_Callback(hObject, eventdata, handles)
% hObject    handle to rreg_slider_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global handles_father;
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
val = get(hObject,'Value');
set(handles.rreg_text_z,'String', sprintf('%1.1f',val));
M = matrix_from_parameters(hObject,eventdata,handles);
handles_father.regmatrix=M;
guidata(handles_father.figure1,handles_father);
cb = get(handles_father.checkbox_overlay,'Callback');
cb(handles_father.checkbox_overlay,[]);
c0 = handles_father.im{2}.GetGeometricalCenter();
M1 = eye(4);
M1(1:3,4)=c0;
p0 = parametersFromRigidMatrix(M1*M/M1)'

% --- Executes during object creation, after setting all properties.
function rreg_slider_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rreg_slider_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function rreg_slider_rx_Callback(hObject, eventdata, handles)
% hObject    handle to rreg_slider_rx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global handles_father;
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
val = get(hObject,'Value');
set(handles.rreg_text_rx,'String', sprintf('%1.1f',val));
M = matrix_from_parameters(hObject,eventdata,handles);
handles_father.regmatrix=M;
guidata(handles_father.figure1,handles_father);
cb = get(handles_father.checkbox_overlay,'Callback');
cb(handles_father.checkbox_overlay,[]);
c0 = handles_father.im{2}.GetGeometricalCenter();
M1 = eye(4);
M1(1:3,4)=c0;
p0 = parametersFromRigidMatrix(M1*M/M1)'

% --- Executes during object creation, after setting all properties.
function rreg_slider_rx_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rreg_slider_rx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function rreg_slider_ry_Callback(hObject, eventdata, handles)
% hObject    handle to rreg_slider_ry (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global handles_father;
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
val = get(hObject,'Value');
set(handles.rreg_text_ry,'String', sprintf('%1.1f',val));
M = matrix_from_parameters(hObject,eventdata,handles);
handles_father.regmatrix=M;
guidata(handles_father.figure1,handles_father);
cb = get(handles_father.checkbox_overlay,'Callback');
cb(handles_father.checkbox_overlay,[]);
c0 = handles_father.im{2}.GetGeometricalCenter();
M1 = eye(4);
M1(1:3,4)=c0;
p0 = parametersFromRigidMatrix(M1*M/M1)'


% --- Executes during object creation, after setting all properties.
function rreg_slider_ry_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rreg_slider_ry (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function rreg_slider_rz_Callback(hObject, eventdata, handles)
% hObject    handle to rreg_slider_rz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global handles_father;
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
val = get(hObject,'Value');
set(handles.rreg_text_rz,'String', sprintf('%1.1f',val));
M = matrix_from_parameters(hObject,eventdata,handles);
handles_father.regmatrix=M;
guidata(handles_father.figure1,handles_father);
cb = get(handles_father.checkbox_overlay,'Callback');
cb(handles_father.checkbox_overlay,[]);
c0 = handles_father.im{2}.GetGeometricalCenter();
M1 = eye(4);
M1(1:3,4)=c0;
p0 = parametersFromRigidMatrix(M1*M/M1)'

% --- Executes during object creation, after setting all properties.
function rreg_slider_rz_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rreg_slider_rz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function M = matrix_from_parameters(hObject,eventdata,handles)
global handles_father;
tx = get(handles.rreg_slider_x,'Value');
ty = get(handles.rreg_slider_y,'Value');
tz = get(handles.rreg_slider_z,'Value');

a = get(handles.rreg_slider_rx,'Value')*pi/180;
b = get(handles.rreg_slider_ry,'Value')*pi/180;
c = get(handles.rreg_slider_rz,'Value')*pi/180;

M = rigidMatrixFromParameters([tx ty tz a b c]');

handles_father.regparams(1)=tx;
handles_father.regparams(2)=ty;
handles_father.regparams(3)=tz;
handles_father.regparams(4)=a;
handles_father.regparams(5)=b;
handles_father.regparams(6)=c;
guidata(handles_father.figure1,handles_father);



% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global handles_father;
set(handles.rreg_slider_x,'Value',0.0);
set(handles.rreg_text_x,'String', sprintf('%1.1f',0));
set(handles.rreg_slider_y,'Value',0.0);
set(handles.rreg_text_y,'String', sprintf('%1.1f',0));
set(handles.rreg_slider_z,'Value',0.0);
set(handles.rreg_text_z,'String', sprintf('%1.1f',0));
set(handles.rreg_slider_rx,'Value',0.0);
set(handles.rreg_text_rx,'String', sprintf('%1.1f',0));
set(handles.rreg_slider_ry,'Value',0.0);
set(handles.rreg_text_ry,'String', sprintf('%1.1f',0));
set(handles.rreg_slider_rz,'Value',0.0);
set(handles.rreg_text_rz,'String', sprintf('%1.1f',0));


M = matrix_from_parameters(hObject,eventdata,handles);
handles_father.regmatrix=M;
guidata(handles_father.figure1,handles_father);
cb = get(handles_father.checkbox_overlay,'Callback');
cb(handles_father.checkbox_overlay,[]);


% --- Executes on button press in rreg_pushbutton_register.
function rreg_pushbutton_register_Callback(hObject, eventdata, handles)
% hObject    handle to rreg_pushbutton_register (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

tx = get(handles.rreg_slider_x,'Value');
ty = get(handles.rreg_slider_y,'Value');
tz = get(handles.rreg_slider_z,'Value');
a = get(handles.rreg_slider_rx,'Value')*pi/180;
b = get(handles.rreg_slider_ry,'Value')*pi/180;
c = get(handles.rreg_slider_rz,'Value')*pi/180;

% params display
disp([ 'Parameters: ' num2str(tx) ', '  num2str(ty) ', '  num2str(tz) ', '  num2str(a) ', '  num2str(b) ', '  num2str(c)])

% initialParameters = [tx ty tz a b c];
% tic;
% 
% 
% 
% 
% totaltime=toc;
%  disp(['Registered with standard RREG in ' num2str(totaltime) ' seconds'])
% Ps(4:6,3)=Ps(4:6,3)*180/pi;
% 
% set(handles.rreg_slider_x,'Value',Ps(1,3));
% set(handles.rreg_text_x,'String', sprintf('%1.1f',Ps(1,3)));
% set(handles.rreg_slider_y,'Value',Ps(2,3));
% set(handles.rreg_text_y,'String', sprintf('%1.1f',Ps(2,3)));
% set(handles.rreg_slider_z,'Value',Ps(3,3));
% set(handles.rreg_text_z,'String', sprintf('%1.1f',Ps(3,3)));
% set(handles.rreg_slider_rx,'Value',Ps(4,3));
% set(handles.rreg_text_rx,'String', sprintf('%1.1f',Ps(4,3)));
% set(handles.rreg_slider_ry,'Value',Ps(5,3));
% set(handles.rreg_text_ry,'String', sprintf('%1.1f',Ps(5,3)));
% set(handles.rreg_slider_rz,'Value',Ps(6,3));
% set(handles.rreg_text_rz,'String', sprintf('%1.1f',Ps(6,3)));
% 
% M = matrix_from_parameters(hObject,eventdata,handles);
% handles_father.regmatrix=M;
% guidata(handles_father.figure1,handles_father);
% cb = get(handles_father.checkbox_overlay,'Callback');
% cb(handles_father.checkbox_overlay,[]);
% 
% Ms
% Ps(:,3)'

