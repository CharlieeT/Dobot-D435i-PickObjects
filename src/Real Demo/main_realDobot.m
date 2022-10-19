%% Dobot Setup
clear all;
clc;
close all;

rosshutdown;
rosinit;

dobot = DobotMagician();
% dobot.InitaliseRobot;

% If you get the error message for python version, type this in the command window:  pyenv("Version", "/usr/bin/python3.9")

%% Real Dobot Run

q0 = [0 pi/4 pi/4 0];

q1 = [-0.9554 0.3962 0.1853 0];

q2 = [0.5849 0.1784 -0.0749 0];

qC{1} = [-1.3232 0.8505 1.0409 0]; % Start Position of blue cube

qC{2} = [-1.0192 1.0056 0.8529 0]; % Start Position of red cube

qC{3} = [-0.7812 1.3434 0.4944 0]; % Start Position of green cube

qE{1} = [0.7743 0.0773 0.9321 0]; % End Position of blue cube

qE{2} = [0.5433 0.3843 0.7755 0]; % End Position of red cube

qE{3} = [0.4181 0.7555 0.5562 0]; % End Position of green cube

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

end_effector_position = [0.01,0.01,0.01];
end_effector_rotation = [0,0,0];
dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);


%% Turn on/off suction cup
function toolOnOff(mode)
    onOff = mode;
    openClose = mode;
    dobot.PublishToolState(onOff,openClose);
end