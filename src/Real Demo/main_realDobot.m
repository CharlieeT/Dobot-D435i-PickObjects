%% Dobot Setup
clear all;
clc;
close all;

rosshutdown;
rosinit("http://localhost:11311");

dobot = DobotMagician();
% dobot.InitaliseRobot;

% If you get the error message for python version, type this in the command window:  pyenv("Version", "/usr/bin/python3.9")


%% Enable RBGD Camera for Point Cloud

% pyenv("Version", "/usr/bin/python3.9") in command box to set python to 3.9

% In ros: roscore
% In ros: roslaunch realsense2_camera  rs_d435_camera_with_model.launch (old)
% In ros: roslaunch realsense2_camera rs_camera.launch align_depth:=true filters:=pointcloud ordered_pc:=true

%%

rgbSub = rossubscriber('/camera/aligned_depth_to_color/image_raw');
pointsSub = rossubscriber('/camera/depth/color/points'); %('/camera/depth/points');
pause(5); 

% Get the first message and plot the non coloured data
pointMsg = pointsSub.LatestMessage;                   % <-----------
pointMsg.PreserveStructureOnRead = false;  %true
cloudPlot_h = scatter3(pointMsg,'Parent',gca);

xlim([-0.3 0.3]);
ylim([-0.1 0.2]);
zlim([0 0.5]);

pcobj = pointCloud(readXYZ(pointMsg),'Color',uint8(255*readRGB(pointMsg)));

redBlock   = [209, 90, 99];
greenBlock = [0, 171, 183];
blueBlock  = [6, 140, 204];

rgb = pcobj.Color(:,:,:);
red = pcobj.Color(:,1,:);
green = pcobj.Color(:,2,:);
blue = pcobj.Color(:,3,:);

thershold = 10;

resultRed   =  find(red > 190 & red < 215  & green > 85 & green < 95 & blue > 94 & blue < 104);
resultGreen =  find(red > 0   & red < 5    & green > 166 & green < 176 & blue > 178 & blue < 188);
resultBlue  =  find(red > 1   & red < 11   & green > 135  & green < 145  & blue > 199 & blue < 209);

%if (find(red < 10) && find(green < 115) && find(green > 95) && find(blue < 190) && find(blue > 170))

drawnow();

cloud = readXYZ(pointMsg);
%  
% x = cloud(:,1,:);
% y = cloud(:,2,:);
% z = cloud(:,3,:);


% Red -------------------------------
IndexR = min((resultRed));
redBlockPose = [cloud(IndexR,1,:), cloud(IndexR,2,:), cloud(IndexR,3,:)]
redRGBVal = [pcobj.Color(IndexR,1,:), pcobj.Color(IndexR,2,:), pcobj.Color(IndexR,3,:)]
% Rx = cloud(IndexR,1,:)
% Ry = cloud(IndexR,2,:)
% Rz = cloud(IndexR,3,:)
% Rr = pcobj.Color(IndexR,1,:)
% Rg = pcobj.Color(IndexR,2,:)
% Rb = pcobj.Color(IndexR,3,:)

% Green -----------------------------
IndexG = min((resultGreen));
greenBlockPose = [cloud(IndexG,1,:), cloud(IndexG,2,:), cloud(IndexG,3,:)]
greenRGBVal = [pcobj.Color(IndexG,1,:), pcobj.Color(IndexG,2,:), pcobj.Color(IndexG,3,:)]

% Blue ------------------------------
IndexB = min((resultBlue));
blueBlockPose = [cloud(IndexB,1,:), cloud(IndexB,2,:), cloud(IndexB,3,:)]
blueRGBVal = [pcobj.Color(IndexB,1,:), pcobj.Color(IndexB,2,:), pcobj.Color(IndexB,3,:)]

cameraPose = [0.145, -0.424, 0.105];

%% Loadin simulated Dobot

    L1 = Link('d',0,       'a',0,       'alpha',-pi/2);  % Base 
    L2 = Link('d',0,       'a',0.135,   'alpha',0);      % RearArm
    L3 = Link('d',0,       'a',0.147,   'alpha',0);      % ForeArm
    L4 = Link('d',0,       'a',0.06,    'alpha',pi/2);   % End-Effector Bracket
    L5 = Link('d',-0.06,   'a',0,       'alpha',0);      % Suction Cup
    %L6 = Link('d',0,   'a',0,       'alpha',0);  % fake link for ikine

    L1.qlim = [-135 135]*pi/180;
    L2.qlim = [5 80]*pi/180;
    L3.qlim = [15 170]*pi/180;
    L4.qlim = [-90 90]*pi/180;
    L5.qlim = [-85 85]*pi/180;
    
    L2.offset = -pi/2;
    L3.offset =  pi/4;
    L4.offset = -pi/4;
    q = [0 pi/4 pi/4 0 0];
      
    DobotSim.model = SerialLink([L1 L2 L3 L4 L5],'name','Dobot');
%     DobotSim.model.plot(q);
    

     

%% Calculate the relative pose of the blocks (robot's frame of reference)

blockHeight = -0.1
Tmask = DobotSim.model.fkine(q);
% T = transl([0 0 0]) *  trotx(180, "deg");
% y is height, 
redTargetPose = [cameraPose(1) - redBlockPose(1) cameraPose(2) + redBlockPose(3) blockHeight]
greenTargetPose = [cameraPose(1) - greenBlockPose(1) cameraPose(2) + greenBlockPose(3) blockHeight]
blueTargetPose = [cameraPose(1) - blueBlockPose(1) cameraPose(2) + blueBlockPose(3) blockHeight]

% qC{1} = DobotSim.model.ikcon(T,[blueTargetPose(1), blueTargetPose(2), blueTargetPose(3)])
% qC{2} = DobotSim.model.ikcon(T,[redTargetPose(1), redTargetPose(2), redTargetPose(3)])
% qC{3} = DobotSim.model.ikcon(T,[greenTargetPose(1), greenTargetPose(2), greenTargetPose(3)])

T0 = transl(0,0,0);
Tr = transl(double(redTargetPose))
Tg = transl(double(greenTargetPose))
Tb = transl(double(blueTargetPose))

% Tred = ctraj(T0, Tr, 50); 	% compute a Cartesian path
% Tgreen = ctraj(T0, Tg, 50);
% Tblue =  ctraj(T0, Tb, 50);
   
input("Press Enter to Move the robot!");

%% Real Dobot Run

q0 = [0 pi/4 pi/4 0 0];

q1 = [-0.9554 0.3962 0.1853 0 0];

q2 = [0.5849 0.1784 -0.0749 0 0];

qC{1} = DobotSim.model.ikcon(Tb)
qC{2} = DobotSim.model.ikcon(Tr)
qC{3} = DobotSim.model.ikcon(Tg)

% qC{1} = [-1.3232 0.8505 1.0409 0 0]; % Start Position of blue cube
% 
% qC{2} = [-1.0192 1.0056 0.8529 0 0]; % Start Position of red cube
% 
% qC{3} = [-0.7812 1.3434 0.4944 0 0]; % Start Position of green cube

qE{1} = [0.7743 0.0773 0.9321 0 0]; % End Position of blue cube

qE{2} = [0.5433 0.3843 0.7755 0 0]; % End Position of red cube

qE{3} = [0.4181 0.7555 0.5562 0 0]; % End Position of green cube

qI = q0;


for i = 1:1:3

    qMatrix = jtraj(qI,q1,50);
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


%% 


% jointTarget = [qMatrix(50,1:3),0]; % Remember that the Dobot has 4 joints by default.
% 
% [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
% trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
% trajectoryPoint.Positions = jointTarget;
% targetJointTrajMsg.Points = trajectoryPoint;
% 
% send(targetJointTrajPub,targetJointTrajMsg);
% 
% end_effector_position = [0.1,0.1,0.1];
% end_effector_rotation = [0,0,0];
% dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

%% Turn on/off suction cup
function toolOnOff(mode)
    onOff = mode;
    openClose = mode;
    dobot.PublishToolState(onOff,openClose);
end

