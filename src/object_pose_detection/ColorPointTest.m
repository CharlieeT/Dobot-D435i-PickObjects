% function ColorPointTest()
close all;

% pyenv("Version", "/usr/bin/python3.9") in command box to set python to 3.9

% In ros: roscore
% In ros: roslaunch realsense2_camera  rs_d435_camera_with_model.launch (old)
% In ros: roslaunch realsense2_camera rs_camera.launch align_depth:=true filters:=pointcloud ordered_pc:=true
%%

rosshutdown;
rosinit("http://localhost:11311");

%%

% use rosbag instead ----------------------------
% bag = rosbag('sushibag.bag');
% rgbSub = select(bag,'Topic', '/camera/aligned_depth_to_color/image_raw');
% pointsSub = select(bag,'Topic', '/camera/depth/color/points');
% pointMsg = readMessages(rgbSub, 1);
% rgbMsg = readMessages(pointsSub,1);
% pointMsg.PreserveStructureOnRead = false;  %true
% cloudPlot_h = scatter3(pointMsg,'Parent',gca);

% -----------------------------------------------

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

resultRed   =  find(red > 204 & red < 214  & green > 85 & green < 95 & blue > 94 & blue < 104);
resultGreen =  find(red > 0   & red < 5    & green > 166 & green < 176 & blue > 178 & blue < 188);
resultBlue  =  find(red > 1   & red < 11   & green > 135  & green < 145  & blue > 199 & blue < 209);


%if (find(red < 10) && find(green < 115) && find(green > 95) && find(blue < 190) && find(blue > 170))

drawnow();

cloud = readXYZ(pointMsg);
%  
% x = cloud(:,1,:);
% y = cloud(:,2,:);
% z = cloud(:,3,:);
% 

% Red -------------------------------
IndexR = min((resultRed));
Rx = cloud(IndexR,1,:)
Ry = cloud(IndexR,2,:)
Rz = cloud(IndexR,3,:)
Rr = pcobj.Color(IndexR,1,:)
Rg = pcobj.Color(IndexR,2,:)
Rb = pcobj.Color(IndexR,3,:)

% Green -----------------------------
IndexG = min((resultGreen));
Gx = cloud(IndexG,1,:)
Gy = cloud(IndexG,2,:)
Gz = cloud(IndexG,3,:)
Gr = pcobj.Color(IndexG,1,:)
Gg = pcobj.Color(IndexG,2,:)
Gb = pcobj.Color(IndexG,3,:)

% Blue ------------------------------
IndexB = min((resultBlue));
Bx = cloud(IndexB,1,:)
By = cloud(IndexB,2,:)
Bz = cloud(IndexB,3,:)
Br = pcobj.Color(IndexB,1,:)
Bg = pcobj.Color(IndexB,2,:)
Bb = pcobj.Color(IndexB,3,:)


%% From Canvas ------------------------------------------------------------------
% % Loop until user breaks with ctrl+c
% while 1
%     % Get latest data and preserve structure in point cloud
%     pointMsg = pointsSub.LatestMessage;
%     pointMsg.PreserveStructureOnRead = true;             
%     
%     % Extract data from msg to matlab
%     cloud = readXYZ(pointMsg); 
% %     img = readImage(rgbSub.LatestMessage);
%     
%     % Put in format to update the scatter3 plot quickly
%  
%       x = cloud(:,1,:);
%       y = cloud(:,2,:);
%       z = cloud(:,3,:);
% %     r = img(:,:,1);
% %     g = img(:,:,2);
% %     b = img(:,:,3);
% %     
% %     % Update the plot
% %     set(cloudPlot_h,'CData',[r(:),g(:),b(:)]);
% %     set(cloudPlot_h,'XData',x(:),'YData',y(:),'ZData',z(:));
%     drawnow();
% end