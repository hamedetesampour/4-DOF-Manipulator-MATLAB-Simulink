function varargout = GUI_noFrame(varargin)
% GUI_NOFRAME MATLAB code for GUI_noFrame.fig
%      GUI_NOFRAME, by itself, creates a new GUI_NOFRAME or raises the existing
%      singleton*.
%
%      H = GUI_NOFRAME returns the handle to a new GUI_NOFRAME or the handle to
%      the existing singleton*.
%
%      GUI_NOFRAME('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_NOFRAME.M with the given input arguments.
%
%      GUI_NOFRAME('Property','Value',...) creates a new GUI_NOFRAME or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_noFrame_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_noFrame_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI_noFrame

% Last Modified by GUIDE v2.5 07-Dec-2024 21:40:09

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_noFrame_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_noFrame_OutputFcn, ...
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


% --- Executes just before GUI_noFrame is made visible.
function GUI_noFrame_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI_noFrame (see VARARGIN)

% Choose default command line output for GUI_noFrame
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes GUI_noFrame wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_noFrame_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

ModelName = 'SCARA_Sim';
%Opens the Simulink mode
open_system(ModelName);
set_param(ModelName,'BlockReduction','off');
set_param(ModelName,'StopTime','inf');
set_param(ModelName,'simulationMode','normal');
set_param(ModelName,'StartFcn','1');
set_param(ModelName,'SimulationCommand','start');
% Disable Start(Forward Kinematics) button
set(handles.pushbutton1,'Enable','off');

set(handles.slider8,'Enable','on');
set(handles.slider9,'Enable','on');
set(handles.slider10,'Enable','on');
set(handles.slider11,'Enable','on');
set(handles.slider12,'Enable','off');
set(handles.slider13,'Enable','off');
set(handles.slider14,'Enable','off');

% Enable Inverse Kinematics pushbutton
set(handles.pushbutton6,'Enable','on');
%initial variables and values
l1 = 0.45; %Joint1_to_GRN_Offset(Z): Base Length - ConnectionBase_to_Link1 
l2 = 0.45; %Joint2_to_Joint1_Offset(X): Link1 Length
l3 = 0.72; %Joint3_to_Joint2_Offset(X): Link2 Length
l4 = 0.15; %Joint3_to_EndEffector_Offset(Z): L4 = L1(0.45) - Theta4_max(0.3) - Pz(=0 for Theta4_max)
Theta1 = 0;
Theta2 = 0;
Theta3 = 0;
Theta4 = 0;
%transfer parameters to gain
set_param([ModelName '/Gain'],'Gain',num2str(Theta1));
set_param([ModelName '/Gain1'],'Gain',num2str(Theta2));
set_param([ModelName '/Gain2'],'Gain',num2str(Theta3));
set_param([ModelName '/Gain3'],'Gain',num2str(Theta4));
%1st Joint (Theta1)-Link1 transfomation matrix
T1 = [cosd(Theta1) -sind(Theta1)  0  l2*cosd(Theta1);
      sind(Theta1)  cosd(Theta1)  0  l2*sind(Theta1);
          0             0         1         l1      ;
          0             0         0         1      ];
%2nd Joint (Theta2)-Link2 transfomation matrix
T2 = [cosd(Theta2) -sind(Theta2)  0  l3*cosd(Theta2);
      sind(Theta2)  cosd(Theta2)  0  l3*sind(Theta2);
          0             0         1         0       ;
          0             0         0         1      ];
%3rd Joint (Theta3)-Link3 transfomation matrix
T3 = [cosd(Theta3) -sind(Theta3)  0         0       ;
      sind(Theta3)  cosd(Theta3)  0         0       ;
          0             0         1         0       ;
          0             0         0         1       ];
%4th Joint (Theta4)-Link4 transfomation matrix
T4 = [    1             0         0         0       ;
          0             1         0         0       ;
          0             0         1    -l4-Theta4   ;
          0             0         0         1      ];
%Full End-Effector transformation matrix
T = T1 * T2 * T3 * T4;
%position
Px = T(1,4);
Py = T(2,4);
Pz = T(3,4);
set(handles.slider8,'value',Theta1);
set(handles.slider9,'value',Theta2);
set(handles.slider10,'value',Theta3);
set(handles.slider11,'value',Theta4);
set(handles.edit3,'string',num2str(0));
set(handles.edit4,'string',num2str(0));
set(handles.edit5,'string',num2str(0));
set(handles.edit6,'string',num2str(0));
set(handles.edit7,'string',num2str(Px));
set(handles.edit8,'string',num2str(Py));
set(handles.edit9,'string',num2str(Pz));


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ModelName = 'SCARA_Sim';
%initial variables and values
Theta1 = 0;
Theta2 = 0;
Theta3 = 0;
Theta4 = 0;
l1 = 0.45; %Joint1_to_GRN_Offset(Z): Base Length - ConnectionBase_to_Link1 
l2 = 0.45; %Joint2_to_Joint1_Offset(X): Link1 Length
l3 = 0.72; %Joint3_to_Joint2_Offset(X): Link2 Length
l4 = 0.15; %Joint3_to_EndEffector_Offset(Z): L4 = L1(0.45) - Theta4_max(0.3) - Pz(=0 for Theta4_max)
%transfer parameters to gain
set_param([ModelName '/Gain'],'Gain',num2str(Theta1));
set_param([ModelName '/Gain1'],'Gain',num2str(Theta2));
set_param([ModelName '/Gain2'],'Gain',num2str(Theta3));
set_param([ModelName '/Gain3'],'Gain',num2str(Theta4));
%1st Joint (Theta1)-Link1 transfomation matrix
T1 = [cosd(Theta1) -sind(Theta1)  0  l2*cosd(Theta1);
      sind(Theta1)  cosd(Theta1)  0  l2*sind(Theta1);
          0             0         1         l1      ;
          0             0         0         1      ];
%2nd Joint (Theta2)-Link2 transfomation matrix
T2 = [cosd(Theta2) -sind(Theta2)  0  l3*cosd(Theta2);
      sind(Theta2)  cosd(Theta2)  0  l3*sind(Theta2);
          0             0         1         0       ;
          0             0         0         1      ];
%3rd Joint (Theta3)-Link3 transfomation matrix
T3 = [cosd(Theta3) -sind(Theta3)  0         0       ;
      sind(Theta3)  cosd(Theta3)  0         0       ;
          0             0         1         0       ;
          0             0         0         1       ];
%4th Joint (Theta4)-Link4 transfomation matrix
T4 = [    1             0         0         0       ;
          0             1         0         0       ;
          0             0         1    -l4-Theta4   ;
          0             0         0         1      ];
%Full End-Effector transformation matrix
T = T1 * T2 * T3 * T4;
%position
Px = T(1,4);
Py = T(2,4);
Pz = T(3,4);
set(handles.slider8,'value',Theta1);
set(handles.slider9,'value',Theta2);
set(handles.slider10,'value',Theta3);
set(handles.slider11,'value',Theta4);
set(handles.edit3,'string',num2str(0));
set(handles.edit4,'string',num2str(0));
set(handles.edit5,'string',num2str(0));
set(handles.edit6,'string',num2str(0));
set(handles.edit7,'string',num2str(Px));
set(handles.edit8,'string',num2str(Py));
set(handles.edit9,'string',num2str(Pz));
% stop the model
set_param(ModelName,'SimulationCommand','stop');
% Enable Start(Forward Kinematics) button
set(handles.pushbutton1,'Enable','on');
% Disable Stop button
set(handles.pushbutton5,'Enable','off');
% Disable Inverse Kinematics pushbutton
set(handles.pushbutton6,'Enable','off');

% --- Executes on slider movement.
function slider8_Callback(hObject, eventdata, handles)
% hObject    handle to slider8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

ModelName = 'SCARA_Sim';
%get the angle and display to edit
Theta1 = get(handles.slider8,'value');
set(handles.edit3,'string',num2str(Theta1));
Theta2 = get(handles.slider9,'value');
set(handles.edit4,'string',num2str(Theta2));
Theta3 = get(handles.slider10,'value');
set(handles.edit5,'string',num2str(Theta3));
Theta4 = get(handles.slider11,'value');
set(handles.edit6,'string',num2str(Theta4));
%initial variables and values
l1 = 0.45; %Joint1_to_GRN_Offset(Z): Base Length - ConnectionBase_to_Link1 
l2 = 0.45; %Joint2_to_Joint1_Offset(X): Link1 Length
l3 = 0.72; %Joint3_to_Joint2_Offset(X): Link2 Length
l4 = 0.15; %Joint3_to_EndEffector_Offset(Z): L4 = L1(0.45) - Theta4_max(0.3) - Pz(=0 for Theta4_max)
%transfer parameters to gain
set_param([ModelName '/Gain'],'Gain',num2str(Theta1));
set_param([ModelName '/Gain1'],'Gain',num2str(Theta2));
set_param([ModelName '/Gain2'],'Gain',num2str(Theta3));
set_param([ModelName '/Gain3'],'Gain',num2str(Theta4));
%1st Joint (Theta1)-Link1 transfomation matrix
T1 = [cosd(Theta1) -sind(Theta1)  0  l2*cosd(Theta1);
      sind(Theta1)  cosd(Theta1)  0  l2*sind(Theta1);
          0             0         1         l1      ;
          0             0         0         1      ];
%2nd Joint (Theta2)-Link2 transfomation matrix
T2 = [cosd(Theta2) -sind(Theta2)  0  l3*cosd(Theta2);
      sind(Theta2)  cosd(Theta2)  0  l3*sind(Theta2);
          0             0         1         0       ;
          0             0         0         1      ];
%3rd Joint (Theta3)-Link3 transfomation matrix
T3 = [cosd(Theta3) -sind(Theta3)  0         0       ;
      sind(Theta3)  cosd(Theta3)  0         0       ;
          0             0         1         0       ;
          0             0         0         1       ];
%4th Joint (Theta4)-Link4 transfomation matrix
T4 = [    1             0         0         0       ;
          0             1         0         0       ;
          0             0         1    -l4-Theta4   ;
          0             0         0         1      ];
%Full End-Effector transformation matrix
T = T1 * T2 * T3 * T4;
%position
Px = T(1,4);
Py = T(2,4);
Pz = T(3,4);
set(handles.edit7,'string',num2str(Px));
set(handles.edit8,'string',num2str(Py));
set(handles.edit9,'string',num2str(Pz));

% --- Executes during object creation, after setting all properties.
function slider8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider9_Callback(hObject, eventdata, handles)
% hObject    handle to slider9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

ModelName = 'SCARA_Sim';
%get the angle and display to edit
Theta1 = get(handles.slider8,'value');
set(handles.edit3,'string',num2str(Theta1));
Theta2 = get(handles.slider9,'value');
set(handles.edit4,'string',num2str(Theta2));
Theta3 = get(handles.slider10,'value');
set(handles.edit5,'string',num2str(Theta3));
Theta4 = get(handles.slider11,'value');
set(handles.edit6,'string',num2str(Theta4));
%initial variables and values
l1 = 0.45; %Joint1_to_GRN_Offset(Z): Base Length - ConnectionBase_to_Link1 
l2 = 0.45; %Joint2_to_Joint1_Offset(X): Link1 Length
l3 = 0.72; %Joint3_to_Joint2_Offset(X): Link2 Length
l4 = 0.15; %Joint3_to_EndEffector_Offset(Z): L4 = L1(0.45) - Theta4_max(0.3) - Pz(=0 for Theta4_max)
%transfer parameters to gain
set_param([ModelName '/Gain'],'Gain',num2str(Theta1));
set_param([ModelName '/Gain1'],'Gain',num2str(Theta2));
set_param([ModelName '/Gain2'],'Gain',num2str(Theta3));
set_param([ModelName '/Gain3'],'Gain',num2str(Theta4));
%1st Joint (Theta1)-Link1 transfomation matrix
T1 = [cosd(Theta1) -sind(Theta1)  0  l2*cosd(Theta1);
      sind(Theta1)  cosd(Theta1)  0  l2*sind(Theta1);
          0             0         1         l1      ;
          0             0         0         1      ];
%2nd Joint (Theta2)-Link2 transfomation matrix
T2 = [cosd(Theta2) -sind(Theta2)  0  l3*cosd(Theta2);
      sind(Theta2)  cosd(Theta2)  0  l3*sind(Theta2);
          0             0         1         0       ;
          0             0         0         1      ];
%3rd Joint (Theta3)-Link3 transfomation matrix
T3 = [cosd(Theta3) -sind(Theta3)  0         0       ;
      sind(Theta3)  cosd(Theta3)  0         0       ;
          0             0         1         0       ;
          0             0         0         1       ];
%4th Joint (Theta4)-Link4 transfomation matrix
T4 = [    1             0         0         0       ;
          0             1         0         0       ;
          0             0         1    -l4-Theta4   ;
          0             0         0         1      ];
%Full End-Effector transformation matrix
T = T1 * T2 * T3 * T4;
%position
Px = T(1,4);
Py = T(2,4);
Pz = T(3,4);
set(handles.edit7,'string',num2str(Px));
set(handles.edit8,'string',num2str(Py));
set(handles.edit9,'string',num2str(Pz));


% --- Executes during object creation, after setting all properties.
function slider9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider10_Callback(hObject, eventdata, handles)
% hObject    handle to slider10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

ModelName = 'SCARA_Sim';
%get the angle and display to edit
Theta1 = get(handles.slider8,'value');
set(handles.edit3,'string',num2str(Theta1));
Theta2 = get(handles.slider9,'value');
set(handles.edit4,'string',num2str(Theta2));
Theta3 = get(handles.slider10,'value');
set(handles.edit5,'string',num2str(Theta3));
Theta4 = get(handles.slider11,'value');
set(handles.edit6,'string',num2str(Theta4));
%initial variables and values
l1 = 0.45; %Joint1_to_GRN_Offset(Z): Base Length - ConnectionBase_to_Link1 
l2 = 0.45; %Joint2_to_Joint1_Offset(X): Link1 Length
l3 = 0.72; %Joint3_to_Joint2_Offset(X): Link2 Length
l4 = 0.15; %Joint3_to_EndEffector_Offset(Z): L4 = L1(0.45) - Theta4_max(0.3) - Pz(=0 for Theta4_max)
%transfer parameters to gain
set_param([ModelName '/Gain'],'Gain',num2str(Theta1));
set_param([ModelName '/Gain1'],'Gain',num2str(Theta2));
set_param([ModelName '/Gain2'],'Gain',num2str(Theta3));
set_param([ModelName '/Gain3'],'Gain',num2str(Theta4));
%1st Joint (Theta1)-Link1 transfomation matrix
T1 = [cosd(Theta1) -sind(Theta1)  0  l2*cosd(Theta1);
      sind(Theta1)  cosd(Theta1)  0  l2*sind(Theta1);
          0             0         1         l1      ;
          0             0         0         1      ];
%2nd Joint (Theta2)-Link2 transfomation matrix
T2 = [cosd(Theta2) -sind(Theta2)  0  l3*cosd(Theta2);
      sind(Theta2)  cosd(Theta2)  0  l3*sind(Theta2);
          0             0         1         0       ;
          0             0         0         1      ];
%3rd Joint (Theta3)-Link3 transfomation matrix
T3 = [cosd(Theta3) -sind(Theta3)  0         0       ;
      sind(Theta3)  cosd(Theta3)  0         0       ;
          0             0         1         0       ;
          0             0         0         1       ];
%4th Joint (Theta4)-Link4 transfomation matrix
T4 = [    1             0         0         0       ;
          0             1         0         0       ;
          0             0         1    -l4-Theta4   ;
          0             0         0         1      ];
%Full End-Effector transformation matrix
T = T1 * T2 * T3 * T4;
%position
Px = T(1,4);
Py = T(2,4);
Pz = T(3,4);
set(handles.edit7,'string',num2str(Px));
set(handles.edit8,'string',num2str(Py));
set(handles.edit9,'string',num2str(Pz));


% --- Executes during object creation, after setting all properties.
function slider10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider11_Callback(hObject, eventdata, handles)
% hObject    handle to slider11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

ModelName = 'SCARA_Sim';
%get the angle and display to edit
Theta1 = get(handles.slider8,'value');
set(handles.edit3,'string',num2str(Theta1));
Theta2 = get(handles.slider9,'value');
set(handles.edit4,'string',num2str(Theta2));
Theta3 = get(handles.slider10,'value');
set(handles.edit5,'string',num2str(Theta3));
Theta4 = get(handles.slider11,'value');
set(handles.edit6,'string',num2str(Theta4));
%initial variables and values
l1 = 0.45; %Joint1_to_GRN_Offset(Z): Base Length - ConnectionBase_to_Link1 
l2 = 0.45; %Joint2_to_Joint1_Offset(X): Link1 Length
l3 = 0.72; %Joint3_to_Joint2_Offset(X): Link2 Length
l4 = 0.15; %Joint3_to_EndEffector_Offset(Z): L4 = L1(0.45) - Theta4_max(0.3) - Pz(=0 for Theta4_max)
%transfer parameters to gain
set_param([ModelName '/Gain'],'Gain',num2str(Theta1));
set_param([ModelName '/Gain1'],'Gain',num2str(Theta2));
set_param([ModelName '/Gain2'],'Gain',num2str(Theta3));
set_param([ModelName '/Gain3'],'Gain',num2str(Theta4));
%1st Joint (Theta1)-Link1 transfomation matrix
T1 = [cosd(Theta1) -sind(Theta1)  0  l2*cosd(Theta1);
      sind(Theta1)  cosd(Theta1)  0  l2*sind(Theta1);
          0             0         1         l1      ;
          0             0         0         1      ];
%2nd Joint (Theta2)-Link2 transfomation matrix
T2 = [cosd(Theta2) -sind(Theta2)  0  l3*cosd(Theta2);
      sind(Theta2)  cosd(Theta2)  0  l3*sind(Theta2);
          0             0         1         0       ;
          0             0         0         1      ];
%3rd Joint (Theta3)-Link3 transfomation matrix
T3 = [cosd(Theta3) -sind(Theta3)  0         0       ;
      sind(Theta3)  cosd(Theta3)  0         0       ;
          0             0         1         0       ;
          0             0         0         1       ];
%4th Joint (Theta4)-Link4 transfomation matrix
T4 = [    1             0         0         0       ;
          0             1         0         0       ;
          0             0         1    -l4-Theta4   ;
          0             0         0         1      ];
%Full End-Effector transformation matrix
T = T1 * T2 * T3 * T4;
%position
Px = T(1,4);
Py = T(2,4);
Pz = T(3,4);
set(handles.edit7,'string',num2str(Px));
set(handles.edit8,'string',num2str(Py));
set(handles.edit9,'string',num2str(Pz));


% --- Executes during object creation, after setting all properties.
function slider11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider12_Callback(hObject, eventdata, handles)
% hObject    handle to slider12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ModelName = 'SCARA_Sim';
%initial variables and values
l1 = 0.45; %Joint1_to_GRN_Offset(Z): Base Length - ConnectionBase_to_Link1 
l2 = 0.45; %Joint2_to_Joint1_Offset(X): Link1 Length
l3 = 0.72; %Joint3_to_Joint2_Offset(X): Link2 Length
l4 = 0.15; %Joint3_to_EndEffector_Offset(Z): L4 = L1(0.45) - Theta4_max(0.3) - Pz(=0 for Theta4_max)
%get position values from inpunt
Px = get(handles.slider12,'value');
set(handles.edit7,'string',num2str(Px));
Py = get(handles.slider13,'value');
set(handles.edit8,'string',num2str(Py));
Pz = get(handles.slider14,'value');
set(handles.edit9,'string',num2str(Pz));
%Joints Inverse Kinematic
A  = ( Px*Px + Py*Py - l2 - l3 ) / ( 2 * l2 * l3 );
An = l3 * sqrt( 1 - A );
Ad = l2 + l3 * A;
Theta1 = atan2d(Px,Py) - atan2d(An,Ad);
Theta2 = atan2d(sqrt(1-A),A);
Theta3 = 0;
Theta4 = l1 - l4 - Pz;
%Set Joints Gain values
set_param([ModelName '/Gain'],'Gain',num2str(Theta1));
set_param([ModelName '/Gain1'],'Gain',num2str(Theta2));
set_param([ModelName '/Gain2'],'Gain',num2str(Theta3));
set_param([ModelName '/Gain3'],'Gain',num2str(Theta4));
%Set Joints Angle values
set(handles.edit3,'string',num2str(Theta1));
set(handles.edit4,'string',num2str(Theta2));
set(handles.edit5,'string',num2str(Theta3));
set(handles.edit6,'string',num2str(Theta4));
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% --- Executes during object creation, after setting all properties.
function slider12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider13_Callback(hObject, eventdata, handles)
% hObject    handle to slider13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ModelName = 'SCARA_Sim';
%initial variables and values
l1 = 0.45; %Joint1_to_GRN_Offset(Z): Base Length - ConnectionBase_to_Link1 
l2 = 0.45; %Joint2_to_Joint1_Offset(X): Link1 Length
l3 = 0.72; %Joint3_to_Joint2_Offset(X): Link2 Length
l4 = 0.15; %Joint3_to_EndEffector_Offset(Z): L4 = L1(0.45) - Theta4_max(0.3) - Pz(=0 for Theta4_max)
%get position values from inpunt
Px = get(handles.slider12,'value');
set(handles.edit7,'string',num2str(Px));
Py = get(handles.slider13,'value');
set(handles.edit8,'string',num2str(Py));
Pz = get(handles.slider14,'value');
set(handles.edit9,'string',num2str(Pz));
%Joints Inverse Kinematic
A  = ( Px*Px + Py*Py - l2 - l3 ) / ( 2 * l2 * l3 );
An = l3 * sqrt( 1 - A );
Ad = l2 + l3 * A;
Theta1 = atan2d(Px,Py) - atan2d(An,Ad);
Theta2 = atan2d(sqrt(1-A),A);
Theta3 = 0;
Theta4 = l1 - l4 - Pz;
%Set Joints Gain values
set_param([ModelName '/Gain'],'Gain',num2str(Theta1));
set_param([ModelName '/Gain1'],'Gain',num2str(Theta2));
set_param([ModelName '/Gain2'],'Gain',num2str(Theta3));
set_param([ModelName '/Gain3'],'Gain',num2str(Theta4));
%Set Joints Angle values
set(handles.edit3,'string',num2str(Theta1));
set(handles.edit4,'string',num2str(Theta2));
set(handles.edit5,'string',num2str(Theta3));
set(handles.edit6,'string',num2str(Theta4));
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider14_Callback(hObject, eventdata, handles)
% hObject    handle to slider14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ModelName = 'SCARA_Sim';
%initial variables and values
l1 = 0.45; %Joint1_to_GRN_Offset(Z): Base Length - ConnectionBase_to_Link1 
l2 = 0.45; %Joint2_to_Joint1_Offset(X): Link1 Length
l3 = 0.72; %Joint3_to_Joint2_Offset(X): Link2 Length
l4 = 0.15; %Joint3_to_EndEffector_Offset(Z): L4 = L1(0.45) - Theta4_max(0.3) - Pz(=0 for Theta4_max)
%get position values from inpunt
Px = get(handles.slider12,'value');
set(handles.edit7,'string',num2str(Px));
Py = get(handles.slider13,'value');
set(handles.edit8,'string',num2str(Py));
Pz = get(handles.slider14,'value');
set(handles.edit9,'string',num2str(Pz));
%Joints Inverse Kinematic
A  = ( Px*Px + Py*Py - l2 - l3 ) / ( 2 * l2 * l3 );
An = l3 * sqrt( 1 - A );
Ad = l2 + l3 * A;
Theta1 = atan2d(Px,Py) - atan2d(An,Ad);
Theta2 = atan2d(sqrt(1-A),A);
Theta3 = 0;
Theta4 = l1 - l4 - Pz;
%Set Joints Gain values
set_param([ModelName '/Gain'],'Gain',num2str(Theta1));
set_param([ModelName '/Gain1'],'Gain',num2str(Theta2));
set_param([ModelName '/Gain2'],'Gain',num2str(Theta3));
set_param([ModelName '/Gain3'],'Gain',num2str(Theta4));
%Set Joints Angle values
set(handles.edit3,'string',num2str(Theta1));
set(handles.edit4,'string',num2str(Theta2));
set(handles.edit5,'string',num2str(Theta3));
set(handles.edit6,'string',num2str(Theta4));
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double


% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ModelName = 'SCARA_Sim';

set(handles.slider12,'Enable','on');
set(handles.slider13,'Enable','on');
set(handles.slider14,'Enable','on');
set(handles.slider8,'Enable','off');
set(handles.slider9,'Enable','off');
set(handles.slider10,'Enable','off');
set(handles.slider11,'Enable','off');

%initial variables and values
l1 = 0.45; %Joint1_to_GRN_Offset(Z): Base Length - ConnectionBase_to_Link1 
l2 = 0.45; %Joint2_to_Joint1_Offset(X): Link1 Length
l3 = 0.72; %Joint3_to_Joint2_Offset(X): Link2 Length
l4 = 0.15; %Joint3_to_EndEffector_Offset(Z): L4 = L1(0.45) - Theta4_max(0.3) - Pz(=0 for Theta4_max)
Px = 1.17;
Py = 0;
Pz = 0.3;
set(handles.edit7,'string',num2str(Px));
set(handles.edit8,'string',num2str(Py));
set(handles.edit9,'string',num2str(Pz));
%Joints Inverse Kinematic
A  = ( Px*Px + Py*Py - l2 - l3 ) / ( 2 * l2 * l3 );
An = l3 * sqrt( 1 - A );
Ad = l2 + l3 * A;
Theta1 = atan2d(Py,Px) - atan2d(An,Ad);
Theta2 = atan2d(sqrt(1-A),A);
Theta3 = 0;
Theta4 = l1 - l4 - Pz;
%Set Joints Gain and Angle values
set_param([ModelName '/Gain'],'Gain',num2str(Theta1));
set(handles.edit3,'string',num2str(Theta1));
set_param([ModelName '/Gain1'],'Gain',num2str(Theta2));
set(handles.edit4,'string',num2str(Theta2));
set_param([ModelName '/Gain2'],'Gain',num2str(Theta3));
set(handles.edit5,'string',num2str(Theta3));
set_param([ModelName '/Gain3'],'Gain',num2str(Theta4));
set(handles.edit6,'string',num2str(Theta4));
% Disable Inverse Kinematics pushbutton
set(handles.pushbutton6,'Enable','off');
% Enable Start(Forward Kinematics) pushbutton
set(handles.pushbutton1,'Enable','on');


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ModelName = 'SCARA_Sim';
%Opens the Simulink mode
open_system(ModelName);
set_param(ModelName,'BlockReduction','off');
set_param(ModelName,'StopTime','inf');
set_param(ModelName,'simulationMode','normal');
set_param(ModelName,'StartFcn','1');
set_param(ModelName,'SimulationCommand','start');
% Enable Start(Forward Kinematics) Mode
set(handles.pushbutton1,'Enable','off');
set(handles.slider8,'Enable','on');
set(handles.slider9,'Enable','on');
set(handles.slider10,'Enable','on');
set(handles.slider11,'Enable','on');
% Disable Inverse Kinematics Mode
set(handles.pushbutton6,'Enable','on');
set(handles.slider12,'Enable','off');
set(handles.slider13,'Enable','off');
set(handles.slider14,'Enable','off');
%initial variables and values
l1 = 0.45; %Joint1_to_GRN_Offset(Z): Base Length - ConnectionBase_to_Link1 
l2 = 0.45; %Joint2_to_Joint1_Offset(X): Link1 Length
l3 = 0.72; %Joint3_to_Joint2_Offset(X): Link2 Length
l4 = 0.15; %Joint3_to_EndEffector_Offset(Z): L4 = L1(0.45) - Theta4_max(0.3) - Pz(=0 for Theta4_max)
Theta1 = 0;
Theta2 = 0;
Theta3 = 0;
Theta4 = 0;
%transfer parameters to gain
set_param([ModelName '/Gain'],'Gain',num2str(Theta1));
set_param([ModelName '/Gain1'],'Gain',num2str(Theta2));
set_param([ModelName '/Gain2'],'Gain',num2str(Theta3));
set_param([ModelName '/Gain3'],'Gain',num2str(Theta4));
%1st Joint (Theta1)-Link1 transfomation matrix
T1 = [cosd(Theta1) -sind(Theta1)  0  l2*cosd(Theta1);
      sind(Theta1)  cosd(Theta1)  0  l2*sind(Theta1);
          0             0         1         l1      ;
          0             0         0         1      ];
%2nd Joint (Theta2)-Link2 transfomation matrix
T2 = [cosd(Theta2) -sind(Theta2)  0  l3*cosd(Theta2);
      sind(Theta2)  cosd(Theta2)  0  l3*sind(Theta2);
          0             0         1         0       ;
          0             0         0         1      ];
%3rd Joint (Theta3)-Link3 transfomation matrix
T3 = [cosd(Theta3) -sind(Theta3)  0         0       ;
      sind(Theta3)  cosd(Theta3)  0         0       ;
          0             0         1         0       ;
          0             0         0         1       ];
%4th Joint (Theta4)-Link4 transfomation matrix
T4 = [    1             0         0         0       ;
          0             1         0         0       ;
          0             0         1    -l4-Theta4   ;
          0             0         0         1      ];
%Full End-Effector transformation matrix
T = T1 * T2 * T3 * T4;
%position
Px = T(1,4);
Py = T(2,4);
Pz = T(3,4);
set(handles.slider8,'value',Theta1);
set(handles.slider9,'value',Theta2);
set(handles.slider10,'value',Theta3);
set(handles.slider11,'value',Theta4);
set(handles.edit3,'string',num2str(0));
set(handles.edit4,'string',num2str(0));
set(handles.edit5,'string',num2str(0));
set(handles.edit6,'string',num2str(0));
set(handles.edit7,'string',num2str(Px));
set(handles.edit8,'string',num2str(Py));
set(handles.edit9,'string',num2str(Pz));
