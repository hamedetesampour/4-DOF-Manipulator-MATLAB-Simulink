%plot code reff:
% https://www.mathworks.com/matlabcentral/answers/1586134-plotting-multivariable-function-given-a-range-of-x-and-y-to-be-plotted-at-given-values-of-dependent



l1 = 0.45; %Joint1_to_GRN_Offset(Z): Base Length - ConnectionBase_to_Link1 
l2 = 0.45; %Joint2_to_Joint1_Offset(X): Link1 Length
l3 = 0.72; %Joint3_to_Joint2_Offset(X): Link2 Length
l4 = 0.15; %Joint3_to_EndEffector_Offset(Z): L4 = L1(0.45) - Theta4_max(0.3) - Pz(=0 for Theta4_max)

%{
% Define the joint angle limits
Theta1_min = -125;
Theta1_max = 125;
Theta2_min = -145;
Theta2_max = 145;
% Define the prismatic joint limits
Theta4_min = 0;
Theta4_max = 0.3;

%Full End-Effector transformation matrix
T = @(Theta1, Theta2, Theta4) [l3 * cosd(Theta1+Theta2) + l2 * cosd(Theta1);
                               l3 * sind(Theta1+Theta2) + l2 * sind(Theta1);
                               l1 - l4 - Theta4                           ];
% Initialize the minimum and maximum values
Px_min = inf;
Px_max = -inf;
Py_min = inf;
Py_max = -inf;
Pz_min = inf;
Pz_max = -inf;

% Loop through the joint limits
for Theta1 = Theta1_min:1:Theta1_max
    for Theta2 = Theta2_min:1:Theta2_max
        for Theta4 = Theta4_min:0.01:Theta4_max
            T_current = T(Theta1, Theta2, Theta4);
            Px = T_current(1);
            Py = T_current(2);
            Pz = T_current(3);
            
            % Update the minimum and maximum values
            Px_min = min(Px_min, Px);
            Px_max = max(Px_max, Px);
            Py_min = min(Py_min, Py);
            Py_max = max(Py_max, Py);
            Pz_min = min(Pz_min, Pz);
            Pz_max = max(Pz_max, Pz);
        end
    end
end

% Display the results
fprintf('Minimum Px: %.2f\n', Px_min);
fprintf('Maximum Px: %.2f\n', Px_max);
fprintf('Minimum Py: %.2f\n', Py_min);
fprintf('Maximum Py: %.2f\n', Py_max);
fprintf('Minimum Pz: %.2f\n', Pz_min);
fprintf('Maximum Pz: %.2f\n', Pz_max);
%}




%{
round4 = @(x) round(x,4);
T_2 = @(Theta1, Theta2) round4([l3 * cosd(Theta1+Theta2) + l2 * cosd(Theta1);
                                l3 * sind(Theta1+Theta2) + l2 * sind(Theta1)]);   
Px = zeros(100,100);
Py = zeros(100,100);
i = 1;
j = 1;
for Theta1 = -125:2.5:125
    for Theta2 = -145:2.9:145
        T_xy = T_2(Theta1,Theta2);
        Px(i,j) = T_xy(1);
        Py(i,j) = T_xy(2);
        j=+1;
    end
    i=+1;
end

T_mag = @(Px,Py) round4(sqrt(Px.^2 + Py.^2));
[X, Y] = meshgrid(Px,Py);
surf(X, Y, T_mag(X,Y));
xlabel('Px');
ylabel('Py');
zlabel('T');
%}



%%% Forward Kinematic Functions
syms Theta1 Theta2 Theta3 Theta4 %{real%}
%{
Theta1=0; Theta2=0; Theta3=0; Theta4 = 0; 
%}
T1 = [cos(Theta1) -sin(Theta1)  0  l2*cos(Theta1);
      sin(Theta1)  cos(Theta1)  0  l2*sin(Theta1);
          0             0         1         l1      ;
          0             0         0         1      ];
%2nd Joint (Theta2)-Link2 transfomation matrix
T2 = [cos(Theta2) -sin(Theta2)  0  l3*cos(Theta2);
      sin(Theta2)  cos(Theta2)  0  l3*sin(Theta2);
          0             0         1         0       ;
          0             0         0         1      ];
%3rd Joint (Theta3)-Link3 transfomation matrix
T3 = [cos(Theta3) -sin(Theta3)  0         0       ;
      sin(Theta3)  cos(Theta3)  0         0       ;
          0             0         1         0       ;
          0             0         0         1       ];
%4th Joint (Theta4)-Link4 transfomation matrix
T4 = [    1             0         0         0       ;
          0             1         0         0       ;
          0             0         1    -l4-Theta4   ;
          0             0         0         1      ];
%Full End-Effector transformation matrix
T = T1 * T2 * T3 * T4;
%{
Px = T(1,4);
Py = T(2,4);
Pz = T(3,4);
P = [Px; Py; Pz];
P
%}




%{
%%% Inverse Kinematic Functions
%initial variables and values
l1 = 0.45; %Joint1_to_GRN_Offset(Z): Base Length - ConnectionBase_to_Link1 
l2 = 0.45; %Joint2_to_Joint1_Offset(X): Link1 Length
l3 = 0.72; %Joint3_to_Joint2_Offset(X): Link2 Length
l4 = 0.15; %Joint3_to_EndEffector_Offset(Z): L4 = L1(0.45) - Theta4_max(0.3) - Pz(=0 for Theta4_max)
Px=1.17; Py=0; Pz=0.3; 

%Joints Inverse Kinematic
A  = ( Px*Px + Py*Py - l2 - l3 ) / ( 2 * l2 * l3 );
An = l3 * sqrt( 1 - A );
Ad = l2 + l3 * A;
Theta1 = atan2d(Py,Px) - atan2d(An,Ad);
Theta2 = atan2d(sqrt(1-A),A);
Theta3 = 0;
Theta4 = l1 - l4 - Pz;
Theta = [Theta1; Theta2; Theta3; Theta4];
Theta
%}


%{
% Desired position 
Px = 1.17; Py = 0; Pz = 0.3;
solutions = solve(T(1,4) == Px, T(2,4) == Py, T(3,4) == Pz, Theta1, Theta2, Theta4); 
% Display solutions 
fprintf('\nSolutions for inverse kinematics:\n'); 
fprintf('Theta1 = %.2f\n',solutions.Theta1); 
fprintf('Theta2 = %.2f\n',solutions.Theta2); 
fprintf('Theta4 = %.2f\n',solutions.Theta4); 
% Consistency check 
for i = 1:length(solutions) 
    Theta1_sol = double(solutions.Theta1(i)); 
    Theta2_sol = double(solutions.Theta2(i)); 
    Theta4_sol = double(solutions.Theta4(i)); 
    % Recompute forward kinematics 
    T_sol = subs(T, {Theta1, Theta2, Theta4}, {Theta1_sol, Theta2_sol, Theta4_sol}); 
    Px_sol = double(T_sol(1,4)); 
    Py_sol = double(T_sol(2,4)); 
    Pz_sol = double(T_sol(3,4)); 
    % Display consistency check results 
    fprintf('\nSolution %d:\n Theta1 = %.2f, Theta2 = %.2f, Theta4 = %.2f\n', i, Theta1_sol, Theta2_sol, Theta4_sol); 
    fprintf('\nRecomputed Vector P:\nPx = %.2f, Py = %.2f, Pz = %.2f\n', Px_sol, Py_sol, Pz_sol); 
end
%}
%Forward Point Test
p_test = subs(T, {Theta1, Theta2, Theta3, Theta4}, {deg2rad(-92.5), deg2rad(-52.2), deg2rad(0), 0.24});
double(p_test)
%Inverse Point Test
Px = 0.61662; Py = -0.95259; Pz = 0.18;
solutions = solve(T(1,4) == Px, T(2,4) == Py, T(3,4) == Pz, Theta1, Theta2, Theta4); 
t11 = double(solutions.Theta1);
rad2deg(t11)