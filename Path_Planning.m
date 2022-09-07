%% Mobile Robotics
% Aim- To perform path planning simulation of a mobile robot. Create a
% scenario, model a robot platform from a rigid body tree object, obtain a
% binary occupancy grid map from the scenario and plan a path for the
% mobile robot to follow using the mobileRobotPRM path planning algorithm.

%% Create Scenario with Ground Plane and Static Meshes
scenario = robotScenario(UpdateRate=5);
floorColor = [1 1 0.95];
addMesh(scenario,"Plane",Position=[20 10 0],Size=[40 20],Color=floorColor);
wallHeight = 1.5;
wallThick = 0.75;
wallThick_in = 0.5;
wallWidth = 20;
wallLength = 40;
wallColor = [0 0 0.15];
%% Outer Walls
addMesh(scenario,"Box",Position=[wallLength/2, wallThick/2, wallHeight/2], Size=[wallLength, wallThick, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[wallThick/2, wallWidth/2, wallHeight/2], Size=[wallThick, wallWidth, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[wallLength/2, wallWidth-(wallThick/2), wallHeight/2], Size=[wallLength, wallThick, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[wallLength-(wallThick/2), wallWidth/2, wallHeight/2], Size=[wallThick, wallWidth, wallHeight],Color=wallColor,IsBinaryOccupied=true);

%% Inner Walls
addMesh(scenario,"Box",Position=[6-(wallThick_in/2), 15.5, wallHeight/2], Size=[wallThick_in, 9, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[6-(wallThick_in/2), 3.5, wallHeight/2], Size=[wallThick_in, 7, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[21-(wallThick_in/2), 3, wallHeight/2], Size=[wallThick_in, 6, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[14-(wallThick/2), 13, wallHeight/2], Size=[wallThick_in, 6, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[26-(wallThick/2), 18.5, wallHeight/2], Size=[wallThick_in, 3, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[32-(wallThick/2), 15, wallHeight/2], Size=[wallThick_in, 4, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[35-(wallThick/2), 9, wallHeight/2], Size=[wallThick_in, 6, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[17.5, 6-(wallThick_in/2), wallHeight/2], Size=[7, wallThick_in, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[21, 10-(wallThick_in/2), wallHeight/2], Size=[14, wallThick_in, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[22.5, 17-(wallThick_in/2), wallHeight/2], Size=[7, wallThick_in, wallHeight],Color=wallColor,IsBinaryOccupied=true);
addMesh(scenario,"Box",Position=[37.5, 6-(wallThick_in/2), wallHeight/2], Size=[5, wallThick_in, wallHeight],Color=wallColor,IsBinaryOccupied=true);

%% Display
show3D(scenario);
lightangle(-45,30);
view(45,45);

%% Obtain Binary Occupancy Map from Scenario
map = binaryOccupancyMap(scenario,GridOriginInLocal=[-1 -1], MapSize=[41 21], MapHeightLimits=[0 3]);
inflate(map,0.01);
show(map)

%% Path Planning
startPosition = [2 18];
goalPosition = [38 2];
rng(200)
numnodes = 5000;
planner = mobileRobotPRM(map,numnodes);
planner.ConnectionDistance = 1;
waypoints = findpath(planner,startPosition,goalPosition);

%% Trajectory Generation
% Robot height from base.
robotheight = 0.12;
% Number of waypoints.
numWaypoints = size(waypoints,1);
% Robot arrival time at first waypoint.
firstInTime = 0;
% Robot arrival time at last waypoint.
lastInTime = firstInTime + (numWaypoints-1);
% Generate waypoint trajectory with waypoints from planned path.
traj = waypointTrajectory(SampleRate=10, TimeOfArrival=firstInTime:lastInTime, Waypoints=[waypoints, robotheight*ones(numWaypoints,1)], ReferenceFrame="ENU");

%% Add Robot Platform to Scenario
huskyRobot = loadrobot("clearpathHusky");
platform = robotPlatform("husky",scenario, RigidBodyTree=huskyRobot,  BaseTrajectory=traj);
[ax,plotFrames] = show3D(scenario);
lightangle(-45,30)
view(-40,50)

%% Simulate Mobile Robot
hold(ax,"on")
plot(ax,waypoints(:,1),waypoints(:,2),"-ms",LineWidth=2,MarkerSize=4,MarkerEdgeColor="b",MarkerFaceColor=[0.5 0.5 0.5]);
hold(ax,"off")
setup(scenario)

% Control simulation rate at 20 Hz.
r = rateControl(20);

% Status of robot in simulation.
robotStartMoving = false;

while advance(scenario)
    show3D(scenario,Parent=ax,FastUpdate=true);
    waitfor(r);

    currentPose = read(platform);
    if ~any(isnan(currentPose))
        % implies that robot is in the scene and performing simulation.
        robotStartMoving = true;
    end
    if any(isnan(currentPose)) && robotStartMoving
        % break, once robot reaches goal position.
        break;
    end
end