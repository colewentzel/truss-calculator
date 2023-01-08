% Input file With all this in mind, set up a general analysis code that will accept an input file
% with the following input parameters and their values:
% 1. The connection matrix C; (i.e.: C = [1 1 0 0 ... 0; 1 0 0 0 ... 0; ... ];)
% 2. The Sx matrix of reaction forces in the x-direction;
% 3. The Sy matrix of reaction forces in the y-direction;
% 4. The joint location vectors X and Y ;
% 5. The vector of applied external loads L.

% Save these parameters to a .mat file, using the following syntax in Matlab:
% save(‘TrussDesign1_MaryJoeBob_A1.mat’,‘C’,‘Sx’,‘Sy’,‘X’,‘Y’,‘L’)

clear 

%`````````User Input``````````
j = 5; %Number of joints
m = 7; %Number of members

W = -27.2; %applied weight in oz

pj = 1; %pin joint number
rj = 3; %roller joint number

%first row is the joint, the second row is the member it is connected to
connections = [1, 1, 2, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5;
               1, 2, 2, 3, 5, 7, 6, 7, 1, 3, 4, 4, 5, 6];
           
%joint vectors, measurements in inches
X = [0, 2, 4, 1, 3];
Y = [0, 0, 0, 3, 3];

%applied loads vector, use just joint number for x direction, use (joint number + j) for y direction
appliedLoads = [3+j;  %joint
                -W]; %load converted to Newtons

%`````````Parameter Calculations``````````
    %Connection Matrix
C = zeros(j, m);

for i = 1:size(connections, 2)
    r = connections(1, i); %this takes the  joint
    c = connections(2, i); %this is the member is is connected to
    C (r, c) = 1; %set (joint, member) equal to 1
end

clear i
clear r
clear c
clear connections
    
    %Sx and Sy
Sx = zeros(j, 3);
Sy = zeros(j, 3);

Sx(pj ,1) = 1; %Sx1
Sy(pj ,2) = 1; %Sy1
Sy(rj, 3) = 1; %Sy2

clear pj
clear rj

    %applied loads vector
L = zeros(2*j, 1);

for i = 1:size(appliedLoads, 2)
    r = appliedLoads(1, i); %this takes the joint
    weight = appliedLoads(2, i); %this is the load applied to that joint
    L (r, 1) = weight; %set (joint, member) equal to 1
end

clear r
clear i
clear weight
clear W
clear appliedLoads

    %Final Save
save('TrussDesign1_TeamGoat_A2.mat','C','Sx','Sy','X','Y','L')
