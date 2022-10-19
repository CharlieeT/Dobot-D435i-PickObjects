%% Clear all ==============================================================
clc
clear all;
close all;
set(0, 'DefaultFigureWindowStyle', 'docked');

%% Enviroment =============================================================

disp('Creating Environment, Please Wait -------- ');
hold on
% 
% axis equal;

% Load Texture Image for the Ground  ------------------------------------------------------------------
surf([-2.6,-2.6; 2.6, 2.6],[-2.6, 2.6;-2.6, 2.6],[0.0,0.01;0.0,0.01],'CData',imread('Floor.jpg'),'FaceColor','texturemap');

% Load Table ------------------------------------------------------------------------------------------
disp('Loading Table.... ');
PlaceObject("table.ply",[0,0,0]);

% % Load Boxes ----------------------------------------------------------------------------------------
disp('Loading Boxes.... ');
PlaceObject("Redbox.ply",[0,0,0]);
PlaceObject("Yellowbox.ply",[0.15,0,0]);

% Load Blocks --------------------------------------------------------------------------------------

% Original ----------------------------------------------------
disp('Loading Condiments.... ');
Red = RedCondiment(transl(0.05,-0.22,0.75));
Yellow = YellowCondiment(transl(0.13,-0.22,0.75));
Bowl = Bowl(transl(0.25,0,0.78));
% -------------------------------------------------------------


hold off

% Load Dobot ----------------------------------------------------------------------------------------
disp('Loading Robot.... ');
dobot = Dobot;
qInitial = dobot.model.getpos;

% Load xArm5 ----------------------------------------------------------------------------------------
xarm = xArm5;


view([160, 28]);   % Changing the camera angle 
hold off
input('Done Loading Environment! Press Enter to Start')



%% Contorl of Robot ======================================================= 


% Moving 1
T1 = dobot.model.fkine(qInitial);
T2 = transl(T1(1:3,4))*transl(0,0,0.2);

Movements.moveikcon(dobot,T2,50);
Movements.moveikcon(dobot,Red.RedCondimentPose,50);

move1 = transl(0.05,-0.22,0.95);

Movements.moveobji(dobot,move1,Red,50);
% Resolved Motion Rate Control 1
finalPosRed = [0.25, 0, 0.8];

Movements.rmrc(dobot,finalPosRed, Red, 50);

% Moving 2

Movements.moveikcon(dobot,Yellow.YellowCondimentPose,50);

move2 = transl(0.13,-0.22,0.95);

Movements.moveobji(dobot,move2,Yellow,50);

% Resolved Motion Rate Control 2

finalPosYellow = [0.25, 0, 0.83];

Movements.rmrc(dobot,finalPosYellow, Yellow, 50);


%% Return to Original Position =========================================================================
% q1 = dobot.model.getpos;
% q2 = qInitial * transl(0,0,0.5);         % This is wrong
% 
% 
% qMatrix4 = jtraj(q1,q2,50);
% 
% for i = 1:50                                                                % Moving the robot to original pose
%     dobot.model.animate(qMatrix4(i,:));
%     drawnow();
% end
