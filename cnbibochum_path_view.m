function varargout = cnbibochum_path_view(varargin)
% CNBIBOCHUM_PATH_VIEW MATLAB code for cnbibochum_path_view.fig
%      CNBIBOCHUM_PATH_VIEW, by itself, creates a new CNBIBOCHUM_PATH_VIEW or raises the existing
%      singleton*.
%
%      H = CNBIBOCHUM_PATH_VIEW returns the handle to a new CNBIBOCHUM_PATH_VIEW or the handle to
%      the existing singleton*.
%
%      CNBIBOCHUM_PATH_VIEW('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CNBIBOCHUM_PATH_VIEW.M with the given input arguments.
%
%      CNBIBOCHUM_PATH_VIEW('Property','Value',...) creates a new CNBIBOCHUM_PATH_VIEW or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before cnbibochum_path_view_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to cnbibochum_path_view_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help cnbibochum_path_view

% Last Modified by GUIDE v2.5 31-Mar-2020 18:06:27

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @cnbibochum_path_view_OpeningFcn, ...
                   'gui_OutputFcn',  @cnbibochum_path_view_OutputFcn, ...
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
end

% --- Executes just before cnbibochum_path_view is made visible.
function cnbibochum_path_view_OpeningFcn(hObject, eventdata, handles, varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to cnbibochum_path_view (see VARARGIN)

    % Initizialization
    handles = cnbibochum_path_view_initialization(handles);

    % Choose default command line output for cnbibochum_path_view
    handles.output = hObject;

    % Update handles structure
    guidata(hObject, handles);
end


% --- Outputs from this function are returned to the command line.
function varargout = cnbibochum_path_view_OutputFcn(hObject, eventdata, handles) 
    % varargout  cell array for returning output args (see VARARGOUT);
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Get default command line output from handles structure
    varargout{1} = handles.output;
end

function handles = cnbibochum_path_view_initialization(handles)
    
    % Initialize elements
    set(handles.pose_x, 'String', num2str(nan));
    set(handles.pose_y, 'String', num2str(nan));
    set(handles.pose_diff, 'String', num2str(nan));
    set(handles.timestamp, 'String', '');
    set(handles.timestamp_id, 'String', '');
    set(handles.timeslider, 'Min', 0);
    set(handles.timeslider, 'Max', 1);
    set(handles.timeslider, 'SliderStep', [0.01 0.1]);
    set(handles.timeslider, 'Value', 0)
    set(handles.startpoint_id, 'String', num2str(nan));
    set(handles.waypoint1_id, 'String', num2str(nan));
    set(handles.waypoint2_id, 'String', num2str(nan));
    set(handles.waypoint3_id, 'String', num2str(nan));
    set(handles.waypoint4_id, 'String', num2str(nan));
    set(handles.stoppoint_id, 'String', num2str(nan));
    set(handles.timing_total, 'String', num2str(nan));
    set(handles.timing_waypoint1, 'String', num2str(nan));
    set(handles.timing_waypoint2, 'String', num2str(nan));
    set(handles.timing_waypoint3, 'String', num2str(nan));
    set(handles.timing_waypoint4, 'String', num2str(nan));
    
    if isfield(handles, 'time')

        set(handles.timeslider, 'Min', 1);
        set(handles.timeslider, 'Max', length(handles.time));
        set(handles.timeslider, 'SliderStep', [1./length(handles.time) 100./length(handles.time)]);
        set(handles.timeslider, 'Value', 1);
        set(handles.timestamp, 'String', num2str(handles.time(1)));
        set(handles.timestamp_id, 'String', num2str(1));
    end
    
    % Default values
    handles.default.graphics.markersize  = 6;
    handles.default.graphics.markercolor = 'k';
    handles.default.graphics.cursorsize  = 6;
    handles.default.graphics.cursorcolor = 'g';
    handles.default.graphics.segment.width = 2;
    handles.default.save.path = './analysis/navigation/new/waypoints/';
    handles.default.load.path = './analysis/navigation/new/processed/';
    handles.isvalid = false;
    
end


function cnbibochum_view_path_plot_path(data, origin, varargin)
    
    data = data - origin;
    hold on;
    plot(data(:, 2), data(:, 1), varargin{:});
    hold off;
end

function handles = cnbibochum_view_path_load_data(handles, filename)

    % Load filename and get info
    posed  = load(filename);
    info   = cnbibochum_view_path_getinfo(filename);
    
    % Plotting map and paths
    origin = posed.map.info.origin;
    cnbibochum_show_map(posed.map.info.x, posed.map.info.y, posed.map.data);
    cnbibochum_view_path_plot_path(posed.pose.xy, origin, 'r.');
    cnbibochum_view_path_plot_path(posed.spose.xy, origin, 'g');
    title([info.subject ' | ' info.datetime]);

    % Store data in handles variable
    handles.pose   = posed.pose;
    handles.spose  = posed.spose;
    handles.map    = posed.map;
    handles.time   = posed.T;
    handles.origin = posed.map.info.origin;
    handles.info   = info;
    
    % Re-initialize elements
    handles = cnbibochum_path_view_initialization(handles);
    handles.isvalid = true;
end


function info = cnbibochum_view_path_getinfo(filename)

    [~, cfilename] = fileparts(filename);
    tokens = regexp(cfilename, '(\w*)\.', 'tokens');
    tokens = vertcat(tokens{:});
    info.subject  = tokens{1};
    info.datetime = datestr(datetime([tokens{2} tokens{3}], 'InputFormat', 'yyyyMMddHHmmss'),  'dd-mmm-yyyy HH:MM:SS');
end


% --- Executes on slider movement.
function timeslider_Callback(hObject, eventdata, handles)
    % hObject    handle to timeslider (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    if(handles.isvalid == false)
        return
    end
    
    currId = floor(get(hObject, 'Value'));

    set(handles.timestamp, 'String', num2str(handles.time(currId)));
    set(handles.timestamp_id, 'String', num2str(currId));


    set(handles.pose_x, 'String', num2str(handles.spose.xy(currId, 2) - handles.origin(2), '%4.3f'));
    set(handles.pose_y, 'String', num2str(handles.spose.xy(currId, 1) - handles.origin(1), '%4.3f'));

    currdiff = 0;
    if (currId > 1)
        currdiff = sqrt( (handles.spose.xy(currId, 2) - handles.spose.xy(currId-1, 2)).^2 + (handles.spose.xy(currId, 1) - handles.spose.xy(currId-1, 1)).^2 );
    end

    set(handles.pose_diff, 'String', num2str(currdiff, '%4.3f'));

    x = handles.spose.xy(currId, 2) - handles.origin(2);
    y = handles.spose.xy(currId, 1) - handles.origin(1);
    CursorSize   = handles.default.graphics.cursorsize;
    CursorColor  = handles.default.graphics.cursorcolor;
    addmarker(handles.map_axes, x, y, 'pose_marker', 'o', 'MarkerEdgeColor', CursorColor, 'MarkerFaceColor', CursorColor, 'MarkerSize', CursorSize);
end

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function timeslider_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to timeslider (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called


    % Hint: slider controls usually have a light gray background.
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end
    
end



function timestamp_Callback(hObject, eventdata, handles)
% hObject    handle to timestamp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of timestamp as text
%        str2double(get(hObject,'String')) returns contents of timestamp as a double
end

% --- Executes during object creation, after setting all properties.
function timestamp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to timestamp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end



function timestamp_id_Callback(hObject, eventdata, handles)
% hObject    handle to timestamp_id (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of timestamp_id as text
%        str2double(get(hObject,'String')) returns contents of timestamp_id as a double
end

% --- Executes during object creation, after setting all properties.
function timestamp_id_CreateFcn(hObject, eventdata, handles)
% hObject    handle to timestamp_id (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end



function pose_x_Callback(hObject, eventdata, handles)
% hObject    handle to pose_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pose_x as text
%        str2double(get(hObject,'String')) returns contents of pose_x as a double
end

% --- Executes during object creation, after setting all properties.
function pose_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pose_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end



function pose_y_Callback(hObject, eventdata, handles)
% hObject    handle to pose_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pose_y as text
%        str2double(get(hObject,'String')) returns contents of pose_y as a double
end

% --- Executes during object creation, after setting all properties.
function pose_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pose_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function pose_diff_Callback(hObject, eventdata, handles)
% hObject    handle to pose_diff (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pose_diff as text
%        str2double(get(hObject,'String')) returns contents of pose_diff as a double
end

% --- Executes during object creation, after setting all properties.
function pose_diff_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pose_diff (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end



function startpoint_id_Callback(hObject, eventdata, handles)
% hObject    handle to startpoint_id (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of startpoint_id as text
%        str2double(get(hObject,'String')) returns contents of startpoint_id as a double
end

% --- Executes during object creation, after setting all properties.
function startpoint_id_CreateFcn(hObject, eventdata, handles)
% hObject    handle to startpoint_id (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function addmarker(haxes, x, y, tag, varargin)
marker_h = findobj(haxes.Children, 'Tag', tag);
if (isempty(marker_h) == false)
    delete(marker_h);
end
hold on;
plot(x, y, varargin{:});
hold off;
h = gca;
set(h.Children(1), 'Tag', tag);
end

function update_timing(hobj, sobj1, sobj2, timestamps)

time1_id = str2double(get(sobj1, 'String'));
time2_id = str2double(get(sobj2, 'String'));

if(isnan(time1_id) || isnan(time2_id))
    TimeDiff = nan;
else
    TimeDiff = timestamps(time2_id) - timestamps(time1_id);
end
set(hobj, 'String', num2str(TimeDiff, '%4.3f'));
end

function handles = delete_graphic_element(handles, tags)
    for i = 1:length(tags)
        marker_h = findobj(handles.map_axes.Children, 'Tag', tags{i});
        if (isempty(marker_h) == false)
            delete(marker_h);
        end
    end
end

function handles = update_path_segment(handles)
    points(1) = str2double(get(handles.startpoint_id, 'String'));
    points(2) = str2double(get(handles.waypoint1_id, 'String'));
    points(3) = str2double(get(handles.waypoint2_id, 'String'));
    points(4) = str2double(get(handles.waypoint3_id, 'String'));
    points(5) = str2double(get(handles.waypoint4_id, 'String'));
    
    LineColor(:, 1) = get(handles.startpoint_store, 'BackgroundColor');
    LineColor(:, 2) = get(handles.waypoint1_store, 'BackgroundColor');
    LineColor(:, 3) = get(handles.waypoint2_store, 'BackgroundColor');
    LineColor(:, 4) = get(handles.waypoint3_store, 'BackgroundColor');
    LineColor(:, 5) = get(handles.waypoint4_store, 'BackgroundColor');
    LineWidth = handles.default.graphics.segment.width;
    
    segmenttag{1} = 'segm_startpoint';
    segmenttag{2} = 'segm_waypoint1';
    segmenttag{3} = 'segm_waypoint2';
    segmenttag{4} = 'segm_waypoint3';
    segmenttag{5} = 'segm_waypoint4';
    
    handles = delete_graphic_element(handles, segmenttag);
    
    for i = 2:length(points)
        cstart = points(i-1);
        cstop  = points(i);
        if(isnan(cstart) || isnan(cstop))
            continue;
        end
        hold on;
        plot(handles.spose.xy(cstart:cstop, 2) - handles.origin(2), handles.spose.xy(cstart:cstop, 1) - handles.origin(1), 'LineWidth', LineWidth, 'Color', LineColor(:, i));
        hold off;
        h = gca;
        set(h.Children(1), 'Tag', segmenttag{i});
    end
end

% --- Executes on button press in startpoint_store.
function startpoint_store_Callback(hObject, eventdata, handles)
% hObject    handle to startpoint_store (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if(handles.isvalid == false), return, end

currId = floor(get(handles.timeslider, 'Value'));
set(handles.startpoint_id, 'String', num2str(currId));

x = handles.spose.xy(currId, 2) - handles.origin(2);
y = handles.spose.xy(currId, 1) - handles.origin(1);
MarkerColor = get(hObject, 'BackgroundColor');
MarkerSize  = handles.default.graphics.markersize;
addmarker(handles.map_axes, x, y, 'startpoint_marker', 's', 'MarkerEdgeColor', MarkerColor, 'MarkerFaceColor', MarkerColor, 'MarkerSize', MarkerSize);
update_timing(handles.timing_total, handles.startpoint_id, handles.waypoint4_id, handles.time);
handles = update_path_segment(handles);
guidata(hObject, handles);
end

% --- Executes on button press in waypoint1_store.
function waypoint1_store_Callback(hObject, eventdata, handles)
% hObject    handle to waypoint1_store (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if(handles.isvalid == false), return, end

currId = floor(get(handles.timeslider, 'Value'));
set(handles.waypoint1_id, 'String', num2str(currId));

x = handles.spose.xy(currId, 2) - handles.origin(2);
y = handles.spose.xy(currId, 1) - handles.origin(1);
MarkerColor = get(hObject, 'BackgroundColor');
MarkerSize  = handles.default.graphics.markersize;
addmarker(handles.map_axes, x, y, 'waypoint1_marker', 's', 'MarkerEdgeColor', MarkerColor, 'MarkerFaceColor', MarkerColor, 'MarkerSize', MarkerSize);
update_timing(handles.timing_waypoint1, handles.startpoint_id, handles.waypoint1_id, handles.time);
handles = update_path_segment(handles);
guidata(hObject, handles);
end

% --- Executes on button press in waypoint2_store.
function waypoint2_store_Callback(hObject, eventdata, handles)
% hObject    handle to waypoint2_store (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if(handles.isvalid == false), return, end

currId = floor(get(handles.timeslider, 'Value'));
set(handles.waypoint2_id, 'String', num2str(currId));

x = handles.spose.xy(currId, 2) - handles.origin(2);
y = handles.spose.xy(currId, 1) - handles.origin(1);
MarkerColor = get(hObject, 'BackgroundColor');
MarkerSize  = handles.default.graphics.markersize;
addmarker(handles.map_axes, x, y, 'waypoint2_marker', 's', 'MarkerEdgeColor', MarkerColor, 'MarkerFaceColor', MarkerColor, 'MarkerSize', MarkerSize);
update_timing(handles.timing_waypoint2, handles.waypoint1_id, handles.waypoint2_id, handles.time);
handles = update_path_segment(handles);
guidata(hObject, handles);
end

% --- Executes on button press in waypoint3_store.
function waypoint3_store_Callback(hObject, eventdata, handles)
% hObject    handle to waypoint3_store (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if(handles.isvalid == false), return, end

currId = floor(get(handles.timeslider, 'Value'));
set(handles.waypoint3_id, 'String', num2str(currId));

x = handles.spose.xy(currId, 2) - handles.origin(2);
y = handles.spose.xy(currId, 1) - handles.origin(1);
MarkerColor = get(hObject, 'BackgroundColor');
MarkerSize  = handles.default.graphics.markersize;
addmarker(handles.map_axes, x, y, 'waypoint3_marker', 's', 'MarkerEdgeColor', MarkerColor, 'MarkerFaceColor', MarkerColor, 'MarkerSize', MarkerSize);
update_timing(handles.timing_waypoint3, handles.waypoint2_id, handles.waypoint3_id, handles.time);
handles = update_path_segment(handles);
guidata(hObject, handles);
end

% --- Executes on button press in waypoint4_store.
function waypoint4_store_Callback(hObject, eventdata, handles)
% hObject    handle to waypoint4_store (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if(handles.isvalid == false), return, end

currId = floor(get(handles.timeslider, 'Value'));
set(handles.waypoint4_id, 'String', num2str(currId));

x = handles.spose.xy(currId, 2) - handles.origin(2);
y = handles.spose.xy(currId, 1) - handles.origin(1);
MarkerColor = get(hObject, 'BackgroundColor');
MarkerSize  = handles.default.graphics.markersize;
addmarker(handles.map_axes, x, y, 'waypoint4_marker', 's', 'MarkerEdgeColor', MarkerColor, 'MarkerFaceColor', MarkerColor, 'MarkerSize', MarkerSize);
update_timing(handles.timing_waypoint4, handles.waypoint3_id, handles.waypoint4_id, handles.time);

handles = update_path_segment(handles);
guidata(hObject, handles);
end

function waypoint1_id_Callback(hObject, eventdata, handles)
% hObject    handle to waypoint1_id (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of waypoint1_id as text
%        str2double(get(hObject,'String')) returns contents of waypoint1_id as a double
end

% --- Executes during object creation, after setting all properties.
function waypoint1_id_CreateFcn(hObject, eventdata, handles)
% hObject    handle to waypoint1_id (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function waypoint2_id_Callback(hObject, eventdata, handles)
% hObject    handle to waypoint2_id (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of waypoint2_id as text
%        str2double(get(hObject,'String')) returns contents of waypoint2_id as a double
end

% --- Executes during object creation, after setting all properties.
function waypoint2_id_CreateFcn(hObject, eventdata, handles)
% hObject    handle to waypoint2_id (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function waypoint3_id_Callback(hObject, eventdata, handles)
% hObject    handle to waypoint3_id (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of waypoint3_id as text
%        str2double(get(hObject,'String')) returns contents of waypoint3_id as a double
end

% --- Executes during object creation, after setting all properties.
function waypoint3_id_CreateFcn(hObject, eventdata, handles)
% hObject    handle to waypoint3_id (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function waypoint4_id_Callback(hObject, eventdata, handles)
% hObject    handle to waypoint4_id (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of waypoint4_id as text
%        str2double(get(hObject,'String')) returns contents of waypoint4_id as a double
end

% --- Executes during object creation, after setting all properties.
function waypoint4_id_CreateFcn(hObject, eventdata, handles)
% hObject    handle to waypoint4_id (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function waypoint4_y_Callback(hObject, eventdata, handles)
% hObject    handle to waypoint4_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of waypoint4_y as text
%        str2double(get(hObject,'String')) returns contents of waypoint4_y as a double
end

% --- Executes during object creation, after setting all properties.
function waypoint4_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to waypoint4_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

% --- Executes on button press in load.
function load_Callback(hObject, eventdata, handles)
    % hObject    handle to load (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    [sfile, spath] = uigetfile([handles.default.load.path '*.mat']);
    cfile = matfile([spath sfile]);

    if( isempty(who(cfile, 'map')) || isempty(who(cfile, 'pose')) || isempty(who(cfile, 'T')))
        errordlg('File does not have the correct format (missing ''map'', ''pose'' or ''T'')');
    else
        handles.filename = sfile;
        handles = cnbibochum_view_path_load_data(handles, [spath sfile]);
        guidata(hObject, handles);
    end
end


% --- Executes on button press in close.
function close_Callback(hObject, eventdata, handles)
% hObject    handle to close (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
closereq();
end

% --- Executes on button press in save.
function save_Callback(hObject, eventdata, handles)
    % hObject    handle to save (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    if(handles.isvalid == false), return, end



    durwp1 = str2double(get(handles.timing_waypoint1, 'String'));
    durwp2 = str2double(get(handles.timing_waypoint2, 'String'));
    durwp3 = str2double(get(handles.timing_waypoint3, 'String'));
    durwp4 = str2double(get(handles.timing_waypoint4, 'String'));
    durtot = str2double(get(handles.timing_total, 'String'));
    idxstr = str2double(get(handles.startpoint_id, 'String'));
    idxwp1 = str2double(get(handles.waypoint1_id, 'String'));
    idxwp2 = str2double(get(handles.waypoint2_id, 'String'));
    idxwp3 = str2double(get(handles.waypoint3_id, 'String'));
    idxwp4 = str2double(get(handles.waypoint4_id, 'String'));
    idxstp = str2double(get(handles.stoppoint_id, 'String'));
    
    
    T = handles.time;
%     POS = [idxstr idxwp1 idxwp2 idxwp3 idxwp4 idxstp];
    POS = [idxstr idxstr idxwp1 idxwp2 idxwp3];
    event.TYP = [500 501 502 503 504];
    event.DUR = [durtot durwp1 durwp2 durwp3 durwp4];
    event.LBL = {'path', 'wp1', 'wp2', 'wp3', 'wp4'};
    
    isvalid = isnan(POS) == false;
    event.TYP = event.TYP(isvalid)';
    event.TIM = T(POS(isvalid));
    event.DUR = event.DUR(isvalid)';
    event.LBL = event.LBL(isvalid)';
    
    filepath = [handles.default.save.path handles.filename];
    answer = questdlg(['Events will be saved in: ' filepath], 'Saving..', 'Ok', 'Cancel', 'Ok');
    
    switch(answer)
        case 'Ok'
            warning('backtrace', 'off')
            mkdir(handles.default.save.path);
            warning('backtrace', 'on')
            save(filepath, 'event', 'T');
        otherwise
    end
    
    
end


function timing_total_Callback(hObject, eventdata, handles)
% hObject    handle to timing_total (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of timing_total as text
%        str2double(get(hObject,'String')) returns contents of timing_total as a double
end

% --- Executes during object creation, after setting all properties.
function timing_total_CreateFcn(hObject, eventdata, handles)
% hObject    handle to timing_total (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function timing_waypoint1_Callback(hObject, eventdata, handles)
% hObject    handle to timing_waypoint1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of timing_waypoint1 as text
%        str2double(get(hObject,'String')) returns contents of timing_waypoint1 as a double
end

% --- Executes during object creation, after setting all properties.
function timing_waypoint1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to timing_waypoint1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function timing_waypoint2_Callback(hObject, eventdata, handles)
% hObject    handle to timing_waypoint2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of timing_waypoint2 as text
%        str2double(get(hObject,'String')) returns contents of timing_waypoint2 as a double
end

% --- Executes during object creation, after setting all properties.
function timing_waypoint2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to timing_waypoint2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function timing_waypoint4_Callback(hObject, eventdata, handles)
% hObject    handle to timing_waypoint4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of timing_waypoint4 as text
%        str2double(get(hObject,'String')) returns contents of timing_waypoint4 as a double
end

% --- Executes during object creation, after setting all properties.
function timing_waypoint4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to timing_waypoint4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function timing_waypoint3_Callback(hObject, eventdata, handles)
% hObject    handle to timing_waypoint3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of timing_waypoint3 as text
%        str2double(get(hObject,'String')) returns contents of timing_waypoint3 as a double
end

% --- Executes during object creation, after setting all properties.
function timing_waypoint3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to timing_waypoint3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


% --- Executes on button press in reset.
function reset_Callback(hObject, eventdata, handles)
% hObject    handle to reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    segmenttag{1} = 'segm_startpoint';
    segmenttag{2} = 'segm_waypoint1';
    segmenttag{3} = 'segm_waypoint2';
    segmenttag{4} = 'segm_waypoint3';
    segmenttag{5} = 'segm_waypoint4';
    markertag{1}  = 'startpoint_marker';
    markertag{2}  = 'waypoint1_marker';
    markertag{3}  = 'waypoint2_marker';
    markertag{4}  = 'waypoint3_marker';
    markertag{5}  = 'waypoint4_marker';
    markertag{5}  = 'stoppoint_marker';
    handles = cnbibochum_path_view_initialization(handles);
    handles = delete_graphic_element(handles, segmenttag);
    handles = delete_graphic_element(handles, markertag);
    handles = delete_graphic_element(handles, {'pose_marker'});
    guidata(hObject, handles);
end



function stoppoint_id_Callback(hObject, eventdata, handles)
% hObject    handle to stoppoint_id (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of stoppoint_id as text
%        str2double(get(hObject,'String')) returns contents of stoppoint_id as a double
end


% --- Executes during object creation, after setting all properties.
function stoppoint_id_CreateFcn(hObject, eventdata, handles)
% hObject    handle to stoppoint_id (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


% --- Executes on button press in stoppoint_store.
function stoppoint_store_Callback(hObject, eventdata, handles)
% hObject    handle to stoppoint_store (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if(handles.isvalid == false), return, end

currId = floor(get(handles.timeslider, 'Value'));
set(handles.stoppoint_id, 'String', num2str(currId));

x = handles.spose.xy(currId, 2) - handles.origin(2);
y = handles.spose.xy(currId, 1) - handles.origin(1);
MarkerColor = get(hObject, 'BackgroundColor');
MarkerSize  = handles.default.graphics.markersize;
addmarker(handles.map_axes, x, y, 'stoppoint_marker', 's', 'MarkerEdgeColor', MarkerColor, 'MarkerFaceColor', MarkerColor, 'MarkerSize', MarkerSize);
update_timing(handles.timing_total, handles.startpoint_id, handles.stoppoint_id, handles.time);
guidata(hObject, handles);
end
