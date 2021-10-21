%% EXAMPLE: Multi-Robot Swarm Behavior
% Copyright 2018 The MathWorks, Inc.

%% Create a multi-robot environment
numRobots = 100;
env = MultiRobotEnv(numRobots);
env.robotRadius = 0.15;
env.showTrajectory = false;

%% Create robot detectors for all robots
detectors = cell(1,numRobots);
for rIdx = 1:numRobots
    detector = RobotDetector(env,rIdx);
    detector.maxDetections = numRobots;
    detector.maxRange = 10;
    detector.fieldOfView = 2*pi;
    detectors{rIdx} = detector;
end
env.plotSensorLines = false; % So the sensor lines don't dominate the visuals

%% Initialization
% Number of robot teams
% numTeams = 25;  
% env.robotColors = repmat(hsv(numTeams),[ceil(numRobots/numTeams) 1]);
% env.robotColors = env.robotColors(1:numRobots,:); % Truncate colors in case there are unequal teams

sampleTime = 0.1;              % Sample time [s]
tVec = 0:sampleTime:25;        % Time array                

% Initialize poses randomly, and add bias to each "team"
poses = [10*(rand(2,numRobots) - 0.5); ...
         pi*rand(1,numRobots)];
%% Simulation loop
vel = zeros(3,numRobots);
for idx = 2:numel(tVec)
    
    % Update the environment
    env(1:numRobots, poses);
    xlim([-10 10]);   % Without this, axis resizing can slow things down
    ylim([-10 10]); 
    
    % Read the sensor and execute the controller for each robot
    for rIdx = 1:numRobots
       detections = step(detectors{rIdx}); 
       vel(:,rIdx) = swarmLineController(poses,rIdx,detections);
    end
    
    % Discrete integration of pose
    poses = poses + vel*sampleTime;

end