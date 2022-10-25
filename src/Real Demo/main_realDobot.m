%% Cleaning the Matlab Command Window and Workspace Variables 

clear all;
clc;
close all;

%%  Initialise ROS and Dobot ======================================================================================

rosshutdown;
rosinit("http://localhost:11311");

dobot = DobotMagician();
% dobot.InitaliseRobot;

%%  Launch Description ============================================================================================

  % Python: 
    % pyenv("Version", "/usr/bin/python3.9") in command box to set python to 3.9

  % ROS: 
    % In ros: roscore
    % In ros: roslaunch realsense2_camera rs_camera.launch align_depth:=true filters:=pointcloud ordered_pc:=true

  % Matlab:
    % Launch the Peter Corke's Robotic Toolbox
    % Start the startup_rvc.m (add it to the path of this project folder)


%%  Enable RBGD Camera for Point Cloud ============================================================================

% Sub to the ideal ros topic from the camera
rgbSub = rossubscriber('/camera/aligned_depth_to_color/image_raw');
pointsSub = rossubscriber('/camera/depth/color/points'); %('/camera/depth/points');
pause(5); 

% Get the first message and plot the pointcloud data as 3D scatter plot
pointMsg = pointsSub.LatestMessage;                
pointMsg.PreserveStructureOnRead = false;  
cloudPlot_h = scatter3(pointMsg,'Parent',gca);

% The view of the camera, limited up to dobot and the workspace
xlim([-0.3 0.3]);
ylim([-0.1 0.2]);
zlim([0 0.5]);

% Only the limit the figure to cube colour
% xlim([-0.1 0.15]);
% ylim([0.02 0.048]);
% zlim([0.15 0.23]);

pcobj = pointCloud(readXYZ(pointMsg),'Color',uint8(255*readRGB(pointMsg)));

% Select the RGB data from the pointcloud
rgb = pcobj.Color(:,:,:);
red = pcobj.Color(:,1,:);
green = pcobj.Color(:,2,:);
blue = pcobj.Color(:,3,:);

% Filter the pointcloud by the colour of the blocks
resultRed   =  find(red > 180 & red < 225  & green > 60 & green < 120 & blue > 94 & blue < 120);
resultGreen =  find(red > 90   & red < 120    & green > 176 & green < 210 & blue > 178 & blue < 210);
resultBlue  =  find(red > 1   & red < 40   & green > 135  & green < 145  & blue > 178 & blue < 210);

drawnow();

cloud = readXYZ(pointMsg);
r = 0; g = 0; b = 0;          % Use to flag if the colour blocks are found


% Red ---------------------------------------------------------------------------------------
IndexR = min((resultRed))+50;           % Find the minimum of the red point but plus 50 index to get to the middle
redBlockPose = [cloud(IndexR,1,:), cloud(IndexR,2,:), cloud(IndexR,3,:)];
redRGBVal = [pcobj.Color(IndexR,1,:), pcobj.Color(IndexR,2,:), pcobj.Color(IndexR,3,:)];

if length(redBlockPose) == 0  || all(redBlockPose) == 0      % If the pose is empty
%   disp("Red not found");
  r = 1;
end  


% Green -------------------------------------------------------------------------------------
IndexG = min((resultGreen))+50;         % Find the minimum of the green point but plus 50 index to get to the middle
greenBlockPose = [cloud(IndexG,1,:), cloud(IndexG,2,:), cloud(IndexG,3,:)];
greenRGBVal = [pcobj.Color(IndexG,1,:), pcobj.Color(IndexG,2,:), pcobj.Color(IndexG,3,:)];

if length(greenBlockPose) == 0 || all(greenBlockPose) == 0    % If the pose is empty
%   disp("Green not found");
  g = 1;
end  


% Blue  -------------------------------------------------------------------------------------
IndexB = min((resultBlue))+50;         % Find the minimum of the blue point but plus 50 index to get to the middle
blueBlockPose = [cloud(IndexB,1,:), cloud(IndexB,2,:), cloud(IndexB,3,:)];
blueRGBVal = [pcobj.Color(IndexB,1,:), pcobj.Color(IndexB,2,:), pcobj.Color(IndexB,3,:)];

if length(blueBlockPose) == 0 || all(blueBlockPose) == 0     % If the pose is empty
%   disp("Blue not found");
  b = 1;
end  


% Hardcode the position of the camera ( in the frame of robot's axis)
cameraPose = [0.113, -0.240, -0.349];


%%  Loadin Simulated Dobot (For Using the Function fkine and ikcon and jtraj) =====================================

   % Create the Links and the end-effector of dobot (Using real dobot's DH parameters)
    L1 = Link('d',0,       'a',0,       'alpha',-pi/2);  % Base 
    L2 = Link('d',0,       'a',0.135,   'alpha',0);      % RearArm
    L3 = Link('d',0,       'a',0.147,   'alpha',0);      % ForeArm
    L4 = Link('d',0,       'a',0.06,    'alpha',pi/2);   % End-Effector Bracket
    L5 = Link('d',-0.06,   'a',0,       'alpha',0);      % Suction Cup

   % Specify the joint limits 
    L1.qlim = [-135 135]*pi/180;
    L2.qlim = [5 80]*pi/180;
    L3.qlim = [15 170]*pi/180;
    L4.qlim = [-90 90]*pi/180;
    L5.qlim = [-85 85]*pi/180;
    
    L2.offset = -pi/2;
    L3.offset =  pi/4;
    L4.offset = -pi/4;

    % default joint state (q)
    q = [0 pi/4 pi/4 0 0];
      
    % Use serial link to connect the links together
    DobotSim.model = SerialLink([L1 L2 L3 L4 L5],'name','Dobot');
    
    % Uncomment if want to plot the stick model dobot 
    % DobotSim.model.plot(q);
    

%%  Calculate the Target Pose of the Blocks (Robot's Frame of Reference) =======================================

blockHeight = -0.12;
Tmask = DobotSim.model.fkine(q);

% Create a empty pose if red block cannot be found --------------------------------------------------------------
    if r == 1  
        redTargetPose   = [0.1611, -0.2279, 0.1715]   % This is a pose above the cubes (Just an empty target pose cause the real one cannot be found)  
        disp("Red Target Pose Cannot be Found");
    else 
        redTargetPose   = [cameraPose(1) + redBlockPose(1)   cameraPose(2) + redBlockPose(2)   blockHeight]
    end


% Create a empty pose if red block cannot be found --------------------------------------------------------------
    if g == 1

        disp("Green Target Pose Cannot be Found");
        greenTargetPose = [0.1611, -0.2279, 0.1715]   % This is a pose above the cubes (Just an empty target pose cause the real one cannot be found) 
    else 
        greenTargetPose = [cameraPose(1) + greenBlockPose(1) cameraPose(2) + greenBlockPose(2) blockHeight]
    end
    

% Create a empty pose if red block cannot be found --------------------------------------------------------------    
    if b == 1
        disp("Blue Target Pose Cannot be Found");
        blueTargetPose  = [0.1611, -0.2279, 0.1715]   % This is a pose above the cubes (Just an empty target pose cause the real one cannot be found) 
    else 
        blueTargetPose  = [cameraPose(1) + blueBlockPose(1)  cameraPose(2) + blueBlockPose(2)  blockHeight]
    end


T0 = transl(0,0,0);
Tr = transl(double(redTargetPose));
Tg = transl(double(greenTargetPose));
Tb = transl(double(blueTargetPose));
   
input("Press Enter to Move the robot!");

%% Control the Real Dobot  ======================================================================================

q0 = [0 pi/4 pi/4 0 0];            % Dobot initial joint state (startup position)
q1 = [-0.9554 0.3962 0.1853 0 0];  % A point above the three cubes
q2 = [0.5849 0.1784 -0.0749 0 0];  % A point above the end point of three cubes (The tray) 

qC{1} = DobotSim.model.ikcon(Tb)  % Use the target pose of the blue  cube to calculate for the joint state q
qC{2} = DobotSim.model.ikcon(Tr)  % Use the target pose of the red   cube to calculate for the joint state q
qC{3} = DobotSim.model.ikcon(Tg)  % Use the target pose of the green cube to calculate for the joint state q

qE{1} = [0.7743 0.0773 0.9321 0 0]; % End Position of Blue Cube 
qE{2} = [0.5433 0.3843 0.7755 0 0]; % End Position of Red Cube
qE{3} = [0.4181 0.7555 0.5562 0 0]; % End Position of Green Cube

% Set the initial joint state to the dobot's start position
qI = q0;

% Loop through the poses to complete the picking and placing of cubes
for i = 1:1:3

    qMatrix =  jtraj(qI,q1,50);
    qMatrix2 = jtraj(q1,q2,50);
    qMatrix1 = jtraj(q1,qC{i},50);
    qMatrix3 = jtraj(q2,qE{i},50);
    qMatrix4 = jtraj(qE{i}, q2, 50);

    Movements.move(qMatrix);
    Movements.move(qMatrix1);

    dobot.PublishToolState(1,1)

    Movements.move(qMatrix);
    Movements.move(qMatrix2);
    Movements.move(qMatrix3);

    dobot.PublishToolState(0,0)

    Movements.move(qMatrix4);
   
    qI = q2;

end

%% Return to Origin ============================================================================================

q2 = [0.5849 0.1784 -0.0749 0 0];
qMatrixOrigin = jtraj(q2, q0, 50);
Movements.move(qMatrixOrigin);


%% A Function to Turn On/Off Suction Cup ======================================================================

function toolOnOff(mode)
    onOff = mode;
    openClose = mode;
    dobot.PublishToolState(onOff,openClose);
end



%% Comments for Debugging =====================================================================================


    %     redTargetPose   = [0.0459, -0.2087, -0.1064]
    %     greenTargetPose = [0.1818, -0.1921, -0.1605]
    %     blueTargetPose  = [0.1201, -0.1921, -0.1251]

% --------------- Testing the joint state of the blocks ----------------
    
    % qC{1} = [-1.3232 0.8505 1.0409 0 0]; % Start Position of blue cube
     
    % qC{2} = [-1.0192 1.0056 0.8529 0 0]; % Start Position of red cube
     
    % qC{3} = [-0.7812 1.3434 0.4944 0 0]; % Start Position of green cube
    
% ---------------------------------------------------------------------
