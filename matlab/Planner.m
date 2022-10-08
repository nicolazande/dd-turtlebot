%% settare nome ROS MASTER per connettersi
master_name = 'http://nicola-Precision-7730:11311'
rosinit(master_name);

%% parametri mappa
image = imread('../maps/turtlebot-map.pgm');
imageNorm = double(image)/255;
imageOccupancy = 1 - imageNorm;
occGrid = occupancyMap(imageOccupancy,1/0.05);
occGrid.LocalOriginInWorld = [-10,-10];
occGrid.OccupiedThreshold = 0.65;
occGrid.FreeThreshold = 0.196;
map = copy(occGrid);
inflate(map,0.1);

%% path planning
start = [-1.0, -2.0, -pi];
goal = [1.0, 2.0, 0];

hold on
plot(start(1), start(2), 'ro')
plot(goal(1), goal(2), 'mo')

r = 0.5;
plot([start(1), start(1) + r*cos(start(3))], [start(2), start(2) + r*sin(start(3))], 'r-' )
plot([goal(1), goal(1) + r*cos(goal(3))], [goal(2), goal(2) + r*sin(goal(3))], 'm-' )
hold off

bounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.4;

stateValidator = validatorOccupancyMap(ss); 
stateValidator.Map = map;
stateValidator.ValidationDistance = 0.05;

planner = plannerRRT(ss, stateValidator);
planner.MaxConnectionDistance = 0.1;
planner.MaxIterations = 30000;
planner.GoalReachedFcn = @exampleHelperCheckIfGoal;

rng(0,'twister')
[pthObj, solnInfo] = plan(planner, start, goal);

show(occGrid)
hold on

plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.-');

interpolate(pthObj,300)
plot(pthObj.States(:,1), pthObj.States(:,2), 'r-', 'LineWidth', 2)

plot(start(1), start(2), 'ro')
plot(goal(1), goal(2), 'mo')
hold off

%% pubblica path
[pub, path] = rospublisher("/path", "nav_msgs/Path");
pause(1);

lr = 0.1;
while (true)
    path.Header.Stamp = rostime("now");
    path.Header.FrameId = "world";

    for i = 1 : length(pthObj.States)
        item = rosmessage('geometry_msgs/PoseStamped');
        item.Header.Stamp = rostime("now");
        item.Header.FrameId = "world";
        item.Pose.Position.X = pthObj.States(i,1);
        item.Pose.Position.Y = pthObj.States(i,2);
        item.Pose.Orientation.Z = pthObj.States(i,3);
        path.Poses(i) = item;
    end
    
    send(pub,path);
    pause(1/lr);
    
end

%% Function definition
function isReached = exampleHelperCheckIfGoal(planner, goalState, newState)
    isReached = false;
    threshold = 0.001;
    if planner.StateSpace.distance(newState, goalState) < threshold
        isReached = true;
    end
end
