%% settare nome ROS MASTER per connettersi
master_name = 'http://nicola-Precision-7730:11311'
rosinit(master_name);

%% condizioni iniziali/finali + flat output
Pi = [0, 0, 0]; %posizione iniziale
Vi = 0; % velocita iniziale
Pf = [2, 1, 0]; %posizione finale
Vf = 0; %velocita finale
ti = 0; %tempo iniziale
tf = 16; %tempo finale
dt = 1;
time = [ti:dt:tf];

%------- coefficenti -------------------------------------------
a0 = Pi(1);
b0 = Pi(2);
a1 = Vi * cos(Pi(3));
b1 = Vi * sin(Pi(3));

A = [1, 0, 0, 0, 0, 0, 0, 0;
     0, 1, 0, 0, 0, 0, 0, 0;
     0, 0, tf^2, tf^3, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, tf^2, tf^3;
     0, 0, 0, 0, 1, 0, 0, 0;
     0, 0, 0, 0, 0, 1, 0, 0;
     0, 0, 2*tf, 3*tf^2, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 2*tf, 3*tf^2];
 
B = [Pi(1); Vi*cos(Pi(3)); Pf(1)-a0-a1*tf; Pf(2)-b0-b1*tf;
     Pi(2); Vi*sin(Pi(3)); Vf*cos(Pf(3))-a1; Vf*sin(Pf(3))-b1];

C = (A^-1* B);

Z1 = zeros(size(time));
pZ1 = zeros(size(time));
ppZ1 = zeros(size(time));
Z2 = zeros(size(time));
pZ2 = zeros(size(time));
ppZ2 = zeros(size(time));
V = zeros(size(time));
W = zeros(size(time));

%-------------- flat outputs ------------------------------------
for i=1:length(time)
    Z1(i) = C(1) + C(2)*time(i) + C(3)*time(i)^2 + C(4)*time(i)^3;
    pZ1(i) = C(2) + 2*C(3)*time(i) + 3*C(4)*time(i)^2;
    ppZ1(i) = 2*C(3) + 6*C(4)*time(i);
    Z2(i) = C(5) + C(6)*time(i) + C(7)*time(i)^2 + C(8)*time(i)^3;
    pZ2(i) = C(6) + 2*C(7)*time(i) + 3*C(8)*time(i)^2;
    ppZ2(i) = 2*C(7) + 6*C(8)*time(i);
end

V = sqrt(pZ1.^2 + pZ2.^2);
W = (pZ1 .* ppZ2 - pZ2 .* pZ2 .* ppZ1) ./ (pZ1.^2 + pZ2.^2);
W(isnan(W)) = 0;

theta = zeros(size(time));
tmp = atan(pZ2./pZ1);
theta(pZ1>0) = tmp(pZ1>0);
tmp = tmp + pi;
theta(pZ1<0) = tmp(pZ1<0);
tmp = sign(pZ2)*pi/2;
theta(pZ1==0) = tmp(pZ1==0);
X = Z1;
Y = Z2;

%% pubblico path
[pub, path] = rospublisher("/path", "nav_msgs/Path");
pause(1);
pause(1);

lr = 0.1;
while (true)
    path.Header.Stamp = rostime("now");
    path.Header.FrameId = "world";

    for i = 1 : length(time)
        item = rosmessage('geometry_msgs/PoseStamped');
        item.Header.Stamp = rostime("now");
        item.Header.FrameId = "world";
        item.Pose.Position.X = X(i);
        item.Pose.Position.Y = Y(i);
        item.Pose.Orientation.Z = theta(i);
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

