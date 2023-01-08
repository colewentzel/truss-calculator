%-----Get Values from Input-------

j = size(C, 1); %get number of joints from C
m = size(C, 2); %get number of members from C

for i = 1:size(L)
    if L(i) ~= 0
        loadWeight = L(i); 
    end
end

clear i
    
%-----Structural Determination-------
    %equilibrium equations
    
    A = zeros(2*j, m+3);
    WFailure = zeros(1, m);
    
    for i = 1:3 %initializes the last three columns of the A matrix to Sx and Sy
        for k = 1:size(Sx, 1)
            A(k, m+i) = Sx(k, i);
            A(k + size(Sx, 1), m+i) = Sy(k, i);
        end
    end
    
    
    %---still need to populate A matrix with coefficients of the force
    %the c matrix determines what values in A are filled and which are zero
    %c is filled twice, once with the x directions and one with the y direction
    
    distanceVec = zeros(1,m);
    
    for i = 1:m %for each member m, determine the two joints the member is connected to
        %reset variables
        joint1 = 0;
        joint2 = 0;
        
        pointLocation = [0 0 ; 0 0];
        
        for k = 1:j %get the two joints the member is connected to and their locations
            if C(k,i) == 1
                if joint1 == 0
                    joint1 = k;
                    pointLocation(1, 1) = X(1, k);
                    pointLocation(1, 2) = Y(1, k);
                else
                    joint2 = k;
                    pointLocation(2, 1) = X(1, k);
                    pointLocation(2, 2) = Y(1, k);
                end
            end
        end
        %get the unit vectors
        distance = pdist(pointLocation,'euclidean'); %get the distance for the member
        distanceVec(i) = distance;
        
        unitVec1x = (pointLocation(2, 1) - pointLocation(1, 1))/distance;
        unitVec2x = (pointLocation(1, 1) - pointLocation(2, 1))/distance;
            
        unitVec1y = (pointLocation(2, 2) - pointLocation(1, 2))/distance;
        unitVec2y = (pointLocation(1, 2) - pointLocation(2, 2))/distance;
            
        %place the vectors into A
        A(joint1, i) = unitVec1x; %joint1x
        A(joint2, i) = unitVec2x; %joint2x
        
        A(joint1 + j, i) = unitVec1y; %joint1y
        A(joint2 + j, i) = unitVec2y; %joint2y
    end
    
    %Cost = C1 J + C2 L
    %where: C1 = $10/joint (2)
    %C2 = $1/ in., (3)
    C1 = 10;
    C2 = 1;
    
    totalDistance = sum(distanceVec, 'all');
    
    Cost = (C1 * j) + (C2 * totalDistance);
    
    T = zeros(m+3, 1); %this is the matrix we will solve the answer into
    T = inv(A)*L;
    
    %-----Load to Cost Ratio-------
    for i = 1:m %for each member m, determine the two joints the member is connected to
        %reset variables
        joint1 = 0;
        joint2 = 0;
        
        pointLocation = [0 0 ; 0 0];
        
        for k = 1:j %get the two joints the member is connected to and their locations
            if C(k,i) == 1
                if joint1 == 0
                    joint1 = k;
                    pointLocation(1, 1) = X(1, k);
                    pointLocation(1, 2) = Y(1, k);
                else
                    pointLocation(2, 1) = X(1, k);
                    pointLocation(2, 2) = Y(1, k);
                end
            end
        end
        distance = pdist(pointLocation,'euclidean'); %get the distance for the member
        
        
        Pcrit = 2945 / (distance^2); %oz weight when buckeling starts
        Rm = T(i)/loadWeight; 
        WFailure(i) = Pcrit/Rm;
    end
    
    failureMem = 0;
    failureWeight = -inf;
    
    for i = 1:length(WFailure)
        if WFailure(i) < 0 %if member in compression
            if WFailure(i) > failureWeight %member needs to be compressed less than last member
                failureWeight = WFailure(i);
            end
        end
    end
    
    weightCostRatio = -failureWeight/Cost;

%-----Print Output-------
fprintf("EK301, Section A2, Anton R., Cole W., Nick N., 11/10/2022. \n")
fprintf("Load: %.2f oz \n", loadWeight)
fprintf("Member forces in oz\n")

for i = 1:m
    multiplier = 1;
    string = " ";
    if T(i) > 0
        string = "(T)";
    elseif T(i) < 0
        multiplier = -1;
        string = "(C)";
    end
    
    fprintf("m(%d): %.3f %s\n", i, multiplier * T(i), string)
end

fprintf("Reaction forces in oz\n") 
fprintf("Sx1: %.2f \n", T(m+1))
fprintf("Sy1: %.2f \n", T(m+2))
fprintf("Sy2: %.2f \n", T(m+3))


fprintf("Cost of truss: $%.2f \n", Cost)
fprintf("Theoretical max load/cost ratio in oz/$:%.2f\n", weightCostRatio)
%load to cost ratio print

%-----Cleanup-------    
clear distance
clear i
clear k
clear joint1
clear joint2
clear pointLocation
clear unitVec1x
clear unitVec2x
clear unitVec1y
clear unitVec2y
clear string
clear multiplier

