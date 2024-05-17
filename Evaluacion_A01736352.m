%% EXAMPLE: Differential drive vehicle following waypoints using the 
% Pure Pursuit algorithm
%
% Copyright 2018-2019 The MathWorks, Inc.

%% Define Vehicle
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);

%% Simulation parameters
sampleTime = 0.05;               % Sample time [s]
tVec = 0:sampleTime:200;        % Time array

initPose = [0;3;-pi/4]; % Initial pose (x y theta) %apple =  [9;5;3*pi/4]; 

pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;


waypoints = [0,3; 1.5,0; 3,3; %V
            3.5,3; 5.5,3; 4.5,3; 4.5,0; 3.5,0; 5.5,0; %I
            8,0; 6,0; 6,3; 8,3; %C
            8.5,3; 10,3; 10,0; 10,3;11.5,3; %T
            12,3; 14.5,3; 14.5,0; 12,0; 12,3; %O
            15,3; 15,0; 15,3; 17.5,3; 17.5, 1.5; 15,1.5; 17.5,0; %R
            18,0; 21,0; %_
            21.5,0; 21.5,3; 23,1.5; 24.5,3; 24.5,0; %M 
            25,0; 25,3; 27.5,3; 27.5,1.5; 25,1.5; 27.5,1.5; 27.5, 0; %A
            28,0; 28,3; 30.5,0; 30.5,3 %N
];


% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Pure Pursuit Controller

controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.2; 
controller.DesiredLinearVelocity = 0.53;  
controller.MaxAngularVelocity = 10; 

%% Simulation loop
close all
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(pose(:,idx-1));
    [wL,wR] = inverseKinematics(dd,vRef,wRef); 
  
    % Compute the velocities
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),waypoints)
    waitfor(r);
    
end