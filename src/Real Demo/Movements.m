classdef Movements < handle
    properties
        
    end
    methods(Static)%Class use static methods
        function self = Movements()
        end
        %% Resolve Motion Rate Control
        function rmrc(robot, finalPos, steps)
            deltaT = 0.05;                                        % Discrete time step
            x = zeros(3,50);
            s = lspb(0,1,50);                                 % Create interpolation scalar
            initialPos = dobot.GetCurrentJointState;
            for i = 1:50
                x(1,i) = initialPos(1)*(1-s(i)) + s(i)*finalPos(1);
                x(2,i) = initialPos(2)*(1-s(i)) + s(i)*finalPos(2);
                x(3,i) = initialPos(3)*(1-s(i)) + s(i)*finalPos(3);
                x(4,i) = 0;
                x(5,i) = 0;
            end

            qMatrix1 = nan(steps,5);

            q0 = robot.model.getpos;
            T = robot.model.fkine(q0);
            qMatrix1(1,:) = robot.model.ikcon(T,q0);                 % Solve for joint angles

            for i = 1:steps-1
                xdot = (x(:,i+1) - x(:,i))/deltaT;                             % Calculate velocity at discrete time step
                J = robot.model.jacob0(qMatrix1(i,:));            % Get the Jacobian at the current state
                J = J(1:5,1:5);
                qdot = inv(J)*xdot;                             % Solve velocitities via RMRC
                qMatrix1(i+1,:) =  qMatrix1(i,:) + deltaT*qdot';      % Update next joint state
                move(qMatrix1)
            end

            for i = 1:49
                move(qMatrix(50-i));
            end
        end
        %% Send to robot
        function move(qMatrix)     
            jointTarget = [qMatrix(50,1:3),0]; % Initial Joint Pose
            [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
            trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
            trajectoryPoint.Positions = jointTarget;
            targetJointTrajMsg.Points = trajectoryPoint;
            send(targetJointTrajPub,targetJointTrajMsg);
            pause(2);
        end
        %% Turn on/off suction cup
        function toolOnOff(mode)
            onOff = mode;
            openClose = mode;
            robot.PublishToolState(onOff,openClose);
        end

%         %% Calculate for joint state from xyz
%         function getq(x,y,z)
%         
% 
%         end

    end
end