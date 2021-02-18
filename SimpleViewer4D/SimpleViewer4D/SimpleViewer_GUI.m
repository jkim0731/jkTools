function varargout = SimpleViewer_GUI(varargin)
% SIMPLEVIEWER_GUI MATLAB code for SimpleViewer_GUI.fig
%      SIMPLEVIEWER_GUI, by itself, creates a new SIMPLEVIEWER_GUI or raises the existing
%      singleton*.
%
%      H = SIMPLEVIEWER_GUI returns the handle to a new SIMPLEVIEWER_GUI or the handle to
%      the existing singleton*.
%
%      SIMPLEVIEWER_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SIMPLEVIEWER_GUI.M with the given input arguments.
%
%      SIMPLEVIEWER_GUI('Property','Value',...) creates a new SIMPLEVIEWER_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SimpleViewer_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SimpleViewer_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
%  By Alberto Gomez (c) 2014 - King's Collee London -  alberto.gomez@kcl.ac.uk
%
% See also: GUIDE, GUIDATA, GUIHANDLES



% Last Modified by GUIDE v2.5 16-Feb-2016 11:39:15

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @SimpleViewer_GUI_OpeningFcn, ...
    'gui_OutputFcn',  @SimpleViewer_GUI_OutputFcn, ...
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


% --- Executes just before SimpleViewer_GUI is made visible.
function SimpleViewer_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SimpleViewer_GUI (see VARARGIN)
% ------------------------------------------------------------------------

%------------------------------------------------------------------------
% Choose default command line output for SimpleViewer_GUI
handles.output = hObject;

enable3D = false;

% general parameters
colors(1,:) = [1 0 0];
colors(2,:) = [0 0 1];
colors(3,:) = [0 0.5 0 ];

Mslice{1}=[1 0 0
    0 -1 0
    0 0 -1]';
Mslice{2}=[1 0 0
    0 0 1
    0 -1 0]';
Mslice{3}=[0 1 0
    0 0 1
    1 0 0]';


RotZ = eye(3);
RotY = eye(3);
RotX = eye(3);
CurrentAxisMatrix = eye(3);
handles.CurrentAxisMatrix = CurrentAxisMatrix;
handles.colors = colors;
handles.currentFrame =1;
handles.Mslice = Mslice;

handles.evaluate_metric = false;
handles.metric_to_evaluate = 'SSD';

factor=0.5;
% setup plots
axis_h = zeros(3,1);

% for registration

handles.regmatrix = eye(4);
handles.regparams = zeros(6,1);
handles.regparams_s = zeros(6,1);
handles.regparams_s(6)=60;
handles.lastParamsToBeUsed = 0;

axis_h(1)=handles.axes1;
axis_h(2)=handles.axes2;
axis_h(3)=handles.axes3;
axis_h(4)=handles.axes4;
handles.axis_h = axis_h;
handles.pause_loop = false;
guidata(hObject, handles);

handles.opacity = 0.5;

handles.factor=factor;
handles.enable3D=enable3D;
handles.Rot{1} = RotZ;
handles.Rot{2} = RotY;
handles.Rot{3} = RotX;
handles.n_images = [0 0];

% default colormaps

handles.colormap{1}=gray;
handles.colormap{2}=dopplerColors;
handles.overly_th = -1;



% default window limits
handles.windowLimits{1}=[0 255];
handles.windowLimits{2}=[0 255];
% Update handles structure
guidata(hObject, handles);

% input data: start with a all-black image----------------------------------------------

im = PatchType([10 10 10]',[0 0 0]',[1 1 1]',eye(3));
if numel(varargin)>0
    % first argument is an image
    if isa(varargin{1},'ImageType') || isa(varargin{1},'PatchType')
        im = varargin{1};
    else
        data = varargin{1};
        sz = size(data)';
        sp = sz*0+1;
        ori = -(sz-1)/2.*sp;
        im = PatchType(sz,ori,sp,eye(numel(sz)));
        im.data = data;
    end
    
    handles.windowLimits{1}=[min(im.data(:)) max(im.data(:))];
    
    if im.ndimensions ==2
        % convert the image to 3D to avoid problems
        M = eye(3);
        M(1:2,1:2) = im.orientation;
        im2 = PatchType([im.size; 2], [im.origin ; -0.5], [im.spacing ; 1], M);
        im2.data = repmat(im.data,1,1,2);
        im = im2;
    end
    
end

handles.im{1} = im;
handles.n_images = [1 0];

if numel(varargin)>1
    % second argument as overlay
    if isa(varargin{2},'ImageType') || isa(varargin{2},'PatchType')
        im2 = varargin{2};
    else
        data = varargin{2};
        sz = size(data)';
        sp = sz*0+1;
        ori = -(sz-1)/2.*sp;
        im2 = PatchType(sz,ori,sp,eye(numel(sz)));
        im2.data = data;
    end

    im2 = resampleImage(im2,handles.im{1},'in_gui');
    handles.im{2} = im2;
    handles.windowLimits{2}=[-max(abs(im2.data(:))) max(abs(im2.data(:)))];
    set(handles.checkbox_overlay,'Enable','on') ; %set to off
    val = get(handles.checkbox_overlay,'Max');
    set(handles.checkbox_overlay,'Value',val);  % uncheck
    handles.n_images = [1 1];
end


% see if there is a file with the recently opened images

fid = fopen([getuserdir '/.SimpleViewer_GUI_recentFiles.txt']);
if fid<0
    % create the file
    fid = fopen([getuserdir '/.SimpleViewer_GUI_recentFiles.txt'],'w');
else
    % read the recent files
    files = (fread(fid,'*char'))';
    if numel(files)
        files = strsplit(files);
        set(handles.fileMenuOpenRecent_tag,'Enable','On');
        for i=1:numel(files)
            handles.fileMenuOpenRecent_tag_files(1) = uimenu(handles.fileMenuOpenRecent_tag,'Label',files{i},...
                'Callback',{@open_image_from_file, handles,files{i}});
            %associate a callback with this menu
        end
    end
end
fclose(fid);


guidata(hObject, handles);
updateData();

%-----------------------------------------------------------



% UIWAIT makes SimpleViewer_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = SimpleViewer_GUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --------------------------------------------------------------------
function fileMenu_tag_Callback(hObject, eventdata, handles)
% hObject    handle to fileMenu_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function fileMenuOpen_tag_Callback(hObject, eventdata, handles)
% hObject    handle to fileMenuOpen_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[filename,pathname]= uigetfile('*.mhd','Open mhd image',[getuserdir '/data']);
handles.filename = [pathname filename];
handles.pathname = pathname;
guidata(hObject, handles);
if ~filename
    return;
end

% Add image to the recent images menu. First read the images already
% available, to make sure there is no more than 10
newfiles{1}=[pathname filename];
MAX_FILES=10;
fid = fopen([getuserdir '/.SimpleViewer_GUI_recentFiles.txt'],'r');
if fid>0
    files = (fread(fid,'*char'))';
    if numel(files)
        % split by return
        files = strsplit(files);
        
        for i=1:min(numel(files),MAX_FILES-1)
            newfiles{i+1}=files{i};
        end
    end
    
    
end
fclose(fid);

% now write the file
fid = fopen([getuserdir '/.SimpleViewer_GUI_recentFiles.txt'],'w');
if fid>0 && numel(newfiles)
    for i =1:numel(newfiles)
        fprintf(fid,'%s\n',newfiles{i});
    end
end
fclose(fid);

% enable the "open recent" tag

open_image_from_file(hObject, eventdata, handles, [pathname filename])

function open_image_from_file(hObject, eventdata, handles, filename)

set(handles.text_filename,'String',filename);
im = read_mhd(filename);

tmp = strfind(filename,filesep);
handles.pathname = filename(1:tmp(end));
handles.filename =filename;
guidata(hObject, handles);
if im.ndimensions ==2
    % convert the image to 3D to avoid problems
    M = eye(3);
    M(1:2,1:2) = im.orientation;
    im2 = PatchType([im.size; 2], [im.origin ; -0.5], [im.spacing ; 1], M);
    im2.data = repmat(im.data,1,1,2);
    im = im2;
end

handles.filename = filename;
guidata(hObject, handles);

handles.im{1} = im;
handles.windowLimits{1}=[min(im.data(:)) max(im.data(:))];
handles.im{2} = [];
handles.n_images = [1 0];
guidata(hObject, handles);
updateData();






% --- Executes on slider movement.
function slider_time_Callback(hObject, eventdata, handles)
% hObject    handle to slider_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

t=floor(get(hObject,'Value'));
set(handles.frame_label,'String',[  num2str(t) '/' num2str(handles.im{1}.size(4))  ]);
handles.currentFrame = t;
guidata(hObject, handles);
for i=3:-1:1
    handles =bv_sliceUpdate_Fcn(0,i,hObject);
    guidata(hObject, handles);
end


%;cf_plot_slice_x(bmc,dc,slx,t,venc, doppler_th); subplot(2,2,2);cf_plot_slice_y(bmc,dc,sly,t,venc, doppler_th); subplot(2,2,4); cf_plot_slice_z(bms,dsr,slz,t,venc, doppler_th); subplot(2,2,1);hold on;  hspx= cf_plot_sphere_x(bmc,slx,bms,slz,nav);    hold off;  subplot(2,2,2);hold on;  hspy= cf_plot_sphere_y(bmc,sly,bms,slz,nav);    hold off;   % Callback string.


% --- Executes during object creation, after setting all properties.
function slider_time_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
M = 2;
set(hObject,'Max',M,'Min',1,'Value',1,'SliderStep',[1/M 3/M]);
% slider_frame = uicontrol(fh,'Style','slider','Max',bms.size(4),'Min',1,'Value',1,'SliderStep',[1/bms.size(4) 10/bms.size(4)],'Position',[60 10 180 25],'callback',cb_frame);
% label_frame = uicontrol(fh,'Style','text','Position',[5 10 50 20],'String','Frame');
% label_frame2 = uicontrol(fh,'Style','text','Position',[250 10 40 20],'String',['1/' num2str(bmc.size(4))  ]);


% --- Executes on button press in pushbutton_pause.
function pushbutton_pause_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_pause (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.pause_loop = true;
%set(handles.pushbutton_pause,'Enable','off');
guidata(hObject, handles);


% --- Executes on button press in pushbutton_play.
function pushbutton_play_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_play (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.pushbutton_play,'Enable','off');
while true % this is broken by the pause button only
    
    %for t=1:handles.im{1}.size(4)
    handles=guidata(hObject);
    if (handles.pause_loop)
        break;
    end
    t = mod(handles.currentFrame, handles.im{1}.size(4))+1;
    set(handles.slider_time,'Value',t);
    set(handles.frame_label,'String',[  num2str(t) '/' num2str(handles.im{1}.size(4))  ]);
    handles.currentFrame = t;
    guidata(hObject, handles);
    for i=3:-1:1
        handles =bv_sliceUpdate_Fcn(0,i,hObject);
        guidata(hObject, handles);
    end
    pause(0.01);
end
set(handles.pushbutton_play,'Enable','on');
handles.pause_loop = false;
guidata(hObject, handles);
%set(handles.pushbutton_pause,'Enable','on');


% --------------------------------------------------------------------
function fileMenuOpenOverly_tag_Callback(hObject, eventdata, handles)
% hObject    handle to fileMenuOpenOverly_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if ~handles.n_images(1)
    disp('No image data!')
    return;
end
[filename,pathname]= uigetfile('*.mhd','Open mhd overly image');
if ~filename
    return;
end
im = read_mhd([pathname filename]);
im = resampleImage(im,handles.im{1},'in_gui');
handles.im{2} = im;
handles.windowLimits{2}=[-max(abs(im.data(:))) max(abs(im.data(:)))];
set(handles.checkbox_overlay,'Enable','on') ; %set to off
val = get(handles.checkbox_overlay,'Max');
set(handles.checkbox_overlay,'Value',val);  % uncheck
guidata(hObject, handles);
checkbox_overlay_Callback(handles.checkbox_overlay, eventdata, handles);
%updateData();


function updateData()
% this function is to be called when there are changes in input data. It
% should reload the views, adjust the slider, etc. It should first of all
% clean up all elements in the axis.

handles = guidata(gcf);

if ~handles.n_images(1)
    disp('No image data!')
    return;
end

for i=1:3
    all_objects = get(handles.axis_h(i),'Children');
    delete(all_objects );% if there is image data, remove it
end

if handles.enable3D
    all_objects = get(handles.axis_h(4),'Children');
    delete(all_objects );% if there is image data, remove it
end

guidata(gcf, handles);
bounds = handles.im{1}.GetBounds();
if numel(bounds<6)
    bounds(5:6)=[-0.5 0.5]';
end
axis_radius = norm(bounds([2 4 6])-bounds([1 3 5]))/6;

if numel(handles.im{1}.size)==4
    % 4D data
    set(handles.slider_time,'Enable','on');
    set(handles.uipanel_temporal,'Visible','on');
    set(handles.slider_time,'Max',handles.im{1}.size(4),'Min',1,'Value',1,'SliderStep',[1/(handles.im{1}.size(4)-1) 3/handles.im{1}.size(4)]);
else
    set(handles.uipanel_temporal,'Visible','off');
    set(handles.slider_time,'Enable','off');
end
xmin=bounds(1);
xmax=bounds(2);
ymin=bounds(3);
ymax=bounds(4);
zmin=bounds(5);
zmax=bounds(6);
im_centroid =[(xmax+xmin)/2 (ymax+ymin)/2 (zmax+zmin)/2]';
handles.centroid = im_centroid;
for i=1:3
    handles.image_centre{i} =im_centroid;
    guidata(gcf, handles);
end

cidx = [3 3 2];
sp_l = cell(3,1); % dragable lines
sp_line = cell(3,1); % non dragable lines
for i=1:3
    sp_l{i,1} = imline(handles.axis_h(i),[-axis_radius*handles.factor 0; 0+axis_radius 0]);
    set(gcf,'CurrentAxes',handles.axis_h(i));
    sp_line{i,1} = line([ 0 0], [-axis_radius*handles.factor  +axis_radius],'Color',handles.colors(cidx(i),:));
    axis equal;
end

handles.axis_radius=axis_radius;
handles.sp_l = sp_l;
handles.sp_line = sp_line;
%handles.sp_p = sp_p;
handles.handle_3D_slice=[0 0 0];
guidata(gcf, handles);

% update callbacks
sp_sliceUpdate=cell(3,3);
for i=1:3
    handles =bv_sliceUpdate_Fcn(0,i,handles.figure1);
    guidata(gcf, handles);
    sp_sliceUpdate{i} = @(x)bv_sliceUpdate_Fcn(x,i,gcf);
end

setColor(sp_l{1},handles.colors(2,:));
sp1_cf = @(x)bv_axis_constrainFcn(x,1);
setPositionConstraintFcn(sp_l{1},sp1_cf);
changesp1_updatesp2 = addNewPositionCallback(sp_l{1},sp_sliceUpdate{2});
changesp1_updatesp3 = addNewPositionCallback(sp_l{1},sp_sliceUpdate{3});


%% Subplot (2,1)
% callback_point_handles{2} = addNewPositionCallback(sp_p{2},sp_lineUpdate{2});

setColor(sp_l{2},handles.colors(1,:));
sp2_cf = @(x)bv_axis_constrainFcn(x,2);
setPositionConstraintFcn(sp_l{2},sp2_cf);
changesp2_updatesp1 = addNewPositionCallback(sp_l{2},sp_sliceUpdate{1});
changesp2_updatesp3 = addNewPositionCallback(sp_l{2},sp_sliceUpdate{3});

%% Subplot (2,2)
%  callback_point_handles{3} = addNewPositionCallback(sp_p{3},sp_lineUpdate{3});

setColor(sp_l{3},handles.colors(1,:));
sp3_cf = @(x)bv_axis_constrainFcn(x,3);
setPositionConstraintFcn(sp_l{3},sp3_cf);
changesp3_updatesp1 = addNewPositionCallback(sp_l{3},sp_sliceUpdate{1});
changesp3_updatesp2 = addNewPositionCallback(sp_l{3},sp_sliceUpdate{2});




% --------------------------------------------------------------------
function fileMenuRemoveOverly_tag_Callback(hObject, eventdata, handles)
% hObject    handle to fileMenuRemoveOverly_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~handles.n_images(2)
    disp('No overly data!')
    return;
end
handles.im{2} = [];
handles.windowLimits{2}=[0 255];
handles.n_images = [handles.n_images(1)  0];
guidata(hObject, handles);
val = get(handles.checkbox_overlay,'Min');
set(handles.checkbox_overlay,'Value',val);  % uncheck
set(handles.checkbox_overlay,'Enable','off') ; %set to off
updateData();


% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1

active = get(hObject,'Value');
if active
    handles.enable3D = true;
    guidata(hObject, handles);
    % update visualisation
    for i=3:-1:1
        handles =bv_sliceUpdate_Fcn(0,i,hObject);
        guidata(hObject, handles);
    end
else
    set(gcf,'CurrentAxes',handles.axis_h(4));
    % Something wrong here. These are not the appropriate handles
    for i=3:-1:1
        delete(handles.handle_3D_slice(i));
        handles.handle_3D_slice(i)=0;
    end
    handles.enable3D = false;
end
guidata(hObject, handles);





% --- Executes on button press in checkbox_overlay.
function checkbox_overlay_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_overlay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_overlay
OnVal = get(hObject,'Max');
val = get(hObject,'Value');
if (val==OnVal)
    handles.n_images = [handles.n_images(1)  1];
else
    handles.n_images = [handles.n_images(1)  0];
end
guidata(hObject, handles);
% update visualisation
for i=3:-1:1
    handles =bv_sliceUpdate_Fcn(0,i,hObject);
    guidata(hObject, handles);
end



% --------------------------------------------------------------------
function about_menu_tag_Callback(hObject, eventdata, handles)
% hObject    handle to about_menu_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
h = msgbox('SimpleViewer4D (c) Alberto Gomez 2014 - Kings College London. alberto.gomez@kcl.ac.uk','About SimpleViewer4D');


% --------------------------------------------------------------------
function fileMenuOpenLocal_tag_Callback(hObject, eventdata, handles)
% hObject    handle to fileMenuOpenLocal_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

ims = uigetvariables({'Please select an image from the workspace','Please select an overlay from the workspace'}, 'InputTypes',{'ImageType','ImageType'});
figure(handles.figure1);
%tvar = uigetvariables({'Pick a number:'}, ...         'InputType',{'string'});

if numel(ims{1})
    handles.im{1} = ims{1};
    handles.windowLimits{1}=[min(ims{1}.data(:)) max(ims{1}.data(:))];
    handles.im{2} = [];
    handles.n_images = [1 0];
    guidata(hObject, handles);
end

if numel(ims{2})
    % add an overlay
    if ~handles.n_images(1)
        disp('No image data! Cannot add overlay')
        return;
    end
    
    im = resampleImage(ims{2},handles.im{1},'in_gui');
    handles.im{2} = im;
    handles.windowLimits{2}=[-max(abs(im.data(:))) max(abs(im.data(:)))];
    set(handles.checkbox_overlay,'Enable','on') ; %set to off
    val = get(handles.checkbox_overlay,'Max');
    set(handles.checkbox_overlay,'Value',val);  % uncheck
    guidata(hObject, handles);
    checkbox_overlay_Callback(handles.checkbox_overlay, eventdata, handles);
end
updateData();


% --------------------------------------------------------------------
function toolMenu_tag_Callback(hObject, eventdata, handles)
% hObject    handle to toolMenu_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function toolMenu_rreg_Callback(hObject, eventdata, handles)
% hObject    handle to toolMenu_rreg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if (numel(handles.im{2}))
    rreg_dialog(hObject);
else
    h = msgbox('ERROR: you need an image and an overlay for registration!','ERROR');
end




% --------------------------------------------------------------------


% --------------------------------------------------------------------
function Untitled_1_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% do something


% --- Executes on button press in pushbutton_previous.
function pushbutton_previous_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_previous (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

filter = get(handles.edit1,'String');
list = dir([handles.pathname filter]);

if numel(list)==0
    return;
end

if get(handles.checkbox7,'Value')
    % sort by time in name
    numbers = zeros(numel(list), 1);
    for i=1:numel(list)
        names = strsplit(list(i).name, '3D_t');
        number = names{2}(1:13);
        numbers(i) = str2num(number);
    end
    [~,idx] = sort(numbers);
    data = list(idx);
else
    % sort by time
    [~,idx] = sort([list.datenum]);
    data = list(idx);
end

for i=1:length(data)
    is = strcmp([ handles.pathname data(i).name] ,handles.filename);
    if is
        break;
    end
end

handles.current_file = i;
guidata(hObject, handles);

previous_file = i-1;
if previous_file<1
    previous_file= length(data);
end

open_image_from_file(hObject, eventdata, handles, [ handles.pathname data(previous_file).name] );



% --- Executes on button press in pushbutton_next.
function pushbutton_next_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_next (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


filter = get(handles.edit1,'String');
list = dir([handles.pathname filter]);

if numel(list)==0
    return;
end

if get(handles.checkbox7,'Value')
    % sort by time in name
    numbers = zeros(numel(list), 1);
    for i=1:numel(list)
        names = strsplit(list(i).name, '3D_t');
        %names = strsplit(list(i).name, '2D_t');
        number = names{2}(1:13);
        numbers(i) = str2num(number);
    end
    [~,idx] = sort(numbers);
    data = list(idx);
else
    % sort by time
    [~,idx] = sort([list.datenum]);
    data = list(idx);
end


for i=1:length(data)
    is = strcmp([ handles.pathname data(i).name] ,handles.filename);
    if is
        break;
    end
end

handles.current_file = i;
guidata(hObject, handles);

next_file = i+1;
if next_file>length(data)
    next_file = 1;
end

open_image_from_file(hObject, eventdata, handles, [ handles.pathname data(next_file).name] );


% --- Executes on button press in checkbox7.
function checkbox7_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox7


% --- Executes on selection change in popupmenu_metrics.
function popupmenu_metrics_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_metrics (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_metrics contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_metrics

contents = cellstr(get(hObject,'String'));
nitem =contents{get(hObject,'Value')} ;
if strcmp(nitem,'No metric')
    handles.evaluate_metric = false;
else
    handles.evaluate_metric = true;
end
handles.metric_to_evaluate = nitem;


% --- Executes during object creation, after setting all properties.
function popupmenu_metrics_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_metrics (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_delete_file.
function pushbutton_delete_file_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_delete_file (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% find next to display afterwards

file_to_delete = handles.filename;

if get(handles.checkbox7,'Value')
    % sort by time in name
    numbers = zeros(numel(list), 1);
    for i=1:numel(list)
        names = strsplit(list(i).name, '3D_t');
        number = names{2}(1:13);
        numbers(i) = str2num(number);
    end
    [~,idx] = sort(numbers);
    data = list(idx);
else
    % sort by time
    [~,idx] = sort([list.datenum]);
    data = list(idx);
end

for i=1:length(data)
    is = strcmp([ handles.pathname data(i).name] ,handles.filename);
    if is
        break;
    end
end


% Construct a questdlg with are you sure? question
choice = questdlg(['WARNING: Do you want to delete ' file_to_delete '? This operation cannot be undone.'], ...
    'Delete file', ...
    'Yes','No','No');
% Handle response
switch choice
    case 'Yes'
        disp([' Deleting ' file_to_delete])
    case 'No'
        return
end

% switch to next image

handles.current_file = i;
guidata(hObject, handles);

next_file = i+1;
if next_file>length(data)
    next_file = 1;
end
open_image_from_file(hObject, eventdata, handles, [ handles.pathname data(next_file).name] );

% delete file
idx_separator = strfind(file_to_delete,'.');
system(['rm ' file_to_delete(1:idx_separator) '*']);
disp([' File ' file_to_delete ' deleted'])


% --------------------------------------------------------------------
function fileMenuOpenRecent_tag_Callback(hObject, eventdata, handles)
% hObject    handle to fileMenuOpenRecent_tag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton_previous10.
function pushbutton_previous10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_previous10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

filter = get(handles.edit1,'String');
list = dir([handles.pathname filter]);

if get(handles.checkbox7,'Value')
    % sort by time in name
    numbers = zeros(numel(list), 1);
    for i=1:numel(list)
        names = strsplit(list(i).name, '3D_t');
        number = names{2}(1:13);
        numbers(i) = str2num(number);
    end
    [~,idx] = sort(numbers);
    data = list(idx);
else
    % sort by time
    [~,idx] = sort([list.datenum]);
    data = list(idx);
end


for i=1:length(data)
    is = strcmp([ handles.pathname data(i).name] ,handles.filename);
    if is
        break;
    end
end

handles.current_file = i;
guidata(hObject, handles);

previous_file = i-10;
if previous_file<1
    previous_file= length(data)+previous_file;
end

open_image_from_file(hObject, eventdata, handles, [ handles.pathname data(previous_file).name] );


% --- Executes on button press in pushbutton_next10.
function pushbutton_next10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_next10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


filter = get(handles.edit1,'String');
list = dir([handles.pathname filter]);


if get(handles.checkbox7,'Value')
    % sort by time in name
    numbers = zeros(numel(list), 1);
    for i=1:numel(list)
        names = strsplit(list(i).name, '3D_t');
        %names = strsplit(list(i).name, '2D_t');
        number = names{2}(1:13);
        numbers(i) = str2num(number);
    end
    [~,idx] = sort(numbers);
    data = list(idx);
else
    % sort by time
    [~,idx] = sort([list.datenum]);
    data = list(idx);
end

for i=1:length(data)
    is = strcmp([ handles.pathname data(i).name] ,handles.filename);
    if is
        break;
    end
end

handles.current_file = i;
guidata(hObject, handles);

next_file = i+10;
if next_file>length(data)
    next_file = 1+next_file-length(data);
end

open_image_from_file(hObject, eventdata, handles, [ handles.pathname data(next_file).name] );



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider_opacity_Callback(hObject, eventdata, handles)
% hObject    handle to slider_opacity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

handles.opacity = get(hObject,'Value');
guidata(hObject, handles);
updateData();



% --- Executes during object creation, after setting all properties.
function slider_opacity_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_opacity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on selection change in popupmenu_colormap.
function popupmenu_colormap_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_colormap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_colormap contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_colormap
contents = cellstr(get(hObject,'String'));
val = contents{get(hObject,'Value')};

handles.windowLimits{2}=[min(handles.im{2}.data(:)) max(handles.im{2}.data(:))];

if strcmp(lower(val),'dopplercolors')
    handles.colormap{2}=dopplerColors;
    handles.windowLimits{2}=[-max(abs(handles.im{2}.data(:))) max(abs(handles.im{2}.data(:)))];
elseif strcmp(lower(val),'gray')
    handles.colormap{2}=gray;
elseif strcmp(lower(val) ,'green')
    handles.colormap{2}=summer;
elseif strcmp(lower(val) ,'jet')
    handles.colormap{2}=jet;
elseif strcmp(lower(val) ,'blue')
    handles.colormap{2}=winter;
end
guidata(hObject, handles);
updateData();



% --- Executes during object creation, after setting all properties.
function popupmenu_colormap_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_colormap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_getAxes.
function pushbutton_getAxes_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_getAxes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


M = eye(4,4);
M(1:3,1:3) = handles.CurrentAxisMatrix;
M(1:3,4) = mean([handles.image_centre{1} handles.image_centre{2} handles.image_centre{3}],2);
M


% --- Executes on button press in checkbox9.
function checkbox9_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox9

dohide = get(hObject,'Value');
if dohide
    for i=1:3
        delete(handles.sp_l{i,1});
        delete(handles.sp_line{i,1})
    end
else
    cidx = [3 3 2];
    sp_l = cell(3,1); % dragable lines
    sp_line = cell(3,1); % non dragable lines
    for i=1:3
        handles.sp_l{i,1} = imline(handles.axis_h(i),[-handles.axis_radius*handles.factor 0; 0+handles.axis_radius 0]);
        set(gcf,'CurrentAxes',handles.axis_h(i));
        handles.sp_line{i,1} = line([ 0 0], [-handles.axis_radius*handles.factor  +handles.axis_radius],'Color',handles.colors(cidx(i),:));
        axis equal;
    end
    guidata(hObject, handles);
    
    % update callbacks
    sp_sliceUpdate=cell(3,3);
    for i=1:3
        handles_ =bv_sliceUpdate_Fcn(0,i,handles.figure1);
        guidata(gcf, handles_);
        sp_sliceUpdate{i} = @(x)bv_sliceUpdate_Fcn(x,i,gcf);
    end
    
    setColor(handles.sp_l{1},handles.colors(2,:));
    sp1_cf = @(x)bv_axis_constrainFcn(x,1);
    setPositionConstraintFcn(handles.sp_l{1},sp1_cf);
    changesp1_updatesp2 = addNewPositionCallback(handles.sp_l{1},sp_sliceUpdate{2});
    changesp1_updatesp3 = addNewPositionCallback(handles.sp_l{1},sp_sliceUpdate{3});
    
    
    %% Subplot (2,1)
    % callback_point_handles{2} = addNewPositionCallback(sp_p{2},sp_lineUpdate{2});
    
    setColor(handles.sp_l{2},handles.colors(1,:));
    sp2_cf = @(x)bv_axis_constrainFcn(x,2);
    setPositionConstraintFcn(handles.sp_l{2},sp2_cf);
    changesp2_updatesp1 = addNewPositionCallback(handles.sp_l{2},sp_sliceUpdate{1});
    changesp2_updatesp3 = addNewPositionCallback(handles.sp_l{2},sp_sliceUpdate{3});
    
    %% Subplot (2,2)
    %  callback_point_handles{3} = addNewPositionCallback(sp_p{3},sp_lineUpdate{3});
    
    setColor(handles.sp_l{3},handles.colors(1,:));
    sp3_cf = @(x)bv_axis_constrainFcn(x,3);
    setPositionConstraintFcn(handles.sp_l{3},sp3_cf);
    changesp3_updatesp1 = addNewPositionCallback(handles.sp_l{3},sp_sliceUpdate{1});
    changesp3_updatesp2 = addNewPositionCallback(handles.sp_l{3},sp_sliceUpdate{2});
    
    
end