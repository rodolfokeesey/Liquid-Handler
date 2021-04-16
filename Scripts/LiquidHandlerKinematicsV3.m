% Liquid Handler Kinematics Test

clear all

%% Define Parameters/Dimensions

%Leg Lengths
L1 = 7;
L2 = 3;
L3 = 10;

%Origin
origin = [0,0];

%% 96 Well Plate Coordinate Generation

% The location of the plate in reference to the origin in cm is
% (6.45,-5.25)at A1. Each well is .9 cm from center to center. The plate extends 12
% wells in the positive Y direction, and 8 wells in the positive x direction.

% Generates Plate Coordinates to be mapped to angles by the inverse
% kinematics
platex = repmat([6.45:0.9:12.75],12,1);
platey = repmat([4.65:-.9:-5.25]',1,8);

platecoords = [];

for i = 1:length(platey(1,:))
    addcoords = [platex(:,i),platey(:,i)];
    platecoords = cat(1,platecoords,addcoords);
end

%Generates the alphanumeric id for each well. The x direction is from A:H,
%the Y direction from 1:12

platexid = repmat({'a','b','c','d','e','f','g','h'},12,1);
plateyid = num2cell(repmat([12:-1:1]',1,8));

plateids = {};

for i = 1:length(plateyid(1,:))
    addids = strcat(platexid(:,i),string(plateyid(:,i)));
    plateids = cat(1,plateids,addids);
end

%% Inverse Kinematics and Theta table generation

thetas = [];
p4filled = []; %Purely for visualization

for endlocation = 1:length(platecoords)
    p4 = platecoords(endlocation,:);
    [theta1,theta2,p1,p2,p3,p4] = inversekin(L1,L2,L3,origin,p4);
    

%% Plotting

figure(1)

hold on
plot(origin(1),origin(2),'o')
xlim([-7 14])
ylim([-7 14])
plot(p1(1),p1(2),'o')
plot(p2(1),p2(2),'o')
plot(p3(1),p3(2),'o')
plot(p4(1),p4(2),'o')

plot([p1(1),origin(2)],[p1(2),origin(2)])
plot([p2(1),origin(2)],[p2(2),origin(2)])
plot([p2(1),p3(1)],[p2(2),p3(2)])
plot([p1(1),p3(1)],[p1(2),p3(2)])
plot([p1(1),p4(1)],[p1(2),p4(2)])

pause(.1)
hold off

thetadd = [theta1,theta2];
thetas = cat(1,thetas,thetadd);

p4filled = cat(1,p4filled,p4);
plot(p4filled(:,1),p4filled(:,2),'o')
xlim([-7 14])
ylim([-7 14])

end

wells2angles = cat(2,plateids,thetas);

%Add inputs into the function below to manually test
%[testtheta1,testtheta2,testp1,testp2,testp3,testp4] = inversekin(L1,L2,L3,origin,[.45,-5.25]);

%% Functions

function [quadrant] = find_quadrant(point)

x = point(1);
y = point(2);

if x >= 0 && y >= 0
    quadrant = 1;
elseif x <= 0 && y >= 0
    quadrant = 2;
elseif x <= 0 && y <= 0
    quadrant = 3;
else
    quadrant = 4;
end

end

% Finds standard postion angle from the positive x axis given point
function [standardangle] = find_standard_angle(point)

x = point(1);
y = point(2);

refangle = atand(y/x);

%finds the quadrant

if x >= 0 && y >= 0
    quadrant = 1;
elseif x <= 0 && y >= 0
    quadrant = 2;
elseif x <= 0 && y <= 0
    quadrant = 3;
else
    quadrant = 4;
end

% finds the standard angle given reference angle and quadrant

if quadrant == 1 || quadrant == 4
    standardangle = 0 + refangle;
elseif quadrant == 2 || quadrant == 3
    standardangle = 180 + refangle;
end

if standardangle < 0
    standardangle = standardangle + 360;
end

end

%Inverse Kinematics Calculations
function [theta1,theta2,p1,p2,p3,p4] = inversekin(L1,L2,L3,origin,p4)

%Calculate the two possible locations of position 1
[xout,yout] = circcirc(p4(1),p4(2),L3,origin(1),origin(2),L1); %Finds the possible p1 points given L3, and the starting xy coordinates

if isempty(xout) == 1
    disp("No possible orientations")
else
    p1a = [xout(1),yout(1)];
    p1b = [xout(2),yout(2)];

    %Evaluating Conformation 1

    con1p3vector = p4-p1a;
    con1p3standangle = find_standard_angle(con1p3vector);

    con1p3 = p1a + [L2*cosd(con1p3standangle+180),L2*sind(con1p3standangle+180)];
    con1p2 = origin + [L2*cosd(con1p3standangle+180),L2*sind(con1p3standangle+180)];

    con1theta1 = find_standard_angle(p1a);
    con1theta2 = find_standard_angle(con1p2);

    %Evaluating Conformation 2

    con2p3vector = p4-p1b;
    con2p3standangle = find_standard_angle(con2p3vector);

    con2p3 = p1b + [L2*cosd(con2p3standangle+180),L2*sind(con2p3standangle+180)];
    con2p2 = origin + [L2*cosd(con2p3standangle+180),L2*sind(con2p3standangle+180)];

    con2theta1 = find_standard_angle(p1b);
    con2theta2 = find_standard_angle(con2p2);

    %Choosing Between Conformations

    conarray = [con1theta1,con1theta2;con2theta1,con2theta2];

    postheta1 = find(conarray(:,1) <= 195);
    postheta2 = find(conarray(:,2) <= 195);

    choice = intersect(postheta1,postheta2);

    if isempty(choice) == 1
        disp("Orientation violates mechanical constraints")
    elseif length(choice) > 1
        disp("Two possible orientations, first orientation selected")
        theta1 = conarray(1,1);
        theta2 = conarray(1,2);
        p1 = p1a;
        p2 = con1p2;
        p3 = con1p3;
    elseif choice == 1
        theta1 = conarray(1,1);
        theta2 = conarray(1,2);
        p1 = p1a;
        p2 = con1p2;
        p3 = con1p3;
    elseif choice == 2
        theta1 = conarray(2,1);
        theta2 = conarray(2,2);
        p1 = p1b;
        p2 = con2p2;
        p3 = con2p3;
    end

end

end

