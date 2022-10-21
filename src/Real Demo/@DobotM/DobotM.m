%% Modified UR3 Robot to use for DoBot
% Li-Ting (Charlie) Tsai
% sID: 13336209
% 20/09/2022   (Good)!

classdef DobotM < handle
    properties
        %> robot model
        model;

        %> Set workspace size
     %  workspace = [-2.6 2.6 -2.6 2.6 0.0 2.0];
        workspace = [-1.5 1.5 -1.5 1.5 0.0 2];    % Good for Demo
     %  workspace = [-1.0 1.0 -1.0 1.0 0.0 2];    % For showing Table only
       
        %> Set Robot's Default q 
        q = [0 pi/4 pi/4 0 0];
    end
    
    methods                      %% Class for Dobot simulation
function self = DobotM()
    self.GetDobot();
    self.PlotAndColourRobot();
%     self.PlotLimits();
end
%% Dobot model
function GetDobot(self)
    
    L1 = Link('d',0,       'a',0,       'alpha',-pi/2);  % Base 
    L2 = Link('d',0,       'a',0.135,   'alpha',0);      % RearArm
    L3 = Link('d',0,       'a',0.147,   'alpha',0);      % ForeArm
    L4 = Link('d',0,       'a',0.06,    'alpha',pi/2);   % End-Effector Bracket
    L5 = Link('d',-0.06,   'a',0,       'alpha',0);      % Suction Cup
    
    L1.qlim = [-135 135]*pi/180;
    L2.qlim = [5 80]*pi/180;
    L3.qlim = [15 170]*pi/180;
    L4.qlim = [-90 90]*pi/180;
    L5.qlim = [-85 85]*pi/180;
    
    L2.offset = -pi/2;
    L3.offset =  pi/4;
    L4.offset = -pi/4;
      
    self.model = SerialLink([L1 L2 L3 L4 L5],'name','Dobot');
    self.model.base = self.model.base * transl(0,0,0.865);       %0.865  // 0.865 - x = 0.75    0.615
%    self.model.plot(self.q)
end

%% PlotAndColourRobot
% Given a robot index, adding the glyphs (vertices and faces) and
% colour them in if data is available 

function PlotAndColourRobot(self)%robot,workspace)
    for linkIndex = 0:self.model.n
        [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['DobotL',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
        self.model.faces{linkIndex+1} = faceData;
        self.model.points{linkIndex+1} = vertexData;
    end

    % Display robot
    self.model.plot3d(self.q,'noarrow','workspace',self.workspace);

    % For teach!
    %$self.model.teach()

    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end  
    self.model.delay = 0.0;

    % Try to correctly colour the arm (if colours are in ply file data)
    for linkIndex = 0:self.model.n
        handles = findobj('Tag', self.model.name);
        h = get(handles,'UserData');
        try 
            h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                          , plyData{linkIndex+1}.vertex.green ...
                                                          , plyData{linkIndex+1}.vertex.blue]/255;
            h.link(linkIndex+1).Children.FaceColor = 'interp';
        catch ME_1
            disp(ME_1);
            continue;
        end
    end
end       

%% Determine the base of the robot and plot the 3D model
function DobotBaseAndPlot(self,position)
    self.model.base = transl(position);
    self.PlotAndColourRobot();
end
    end
end
