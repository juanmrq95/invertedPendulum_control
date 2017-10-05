%% Prelab 6b set up
Mc = .94; % kg Mass of cart
r = 6.36e-3; % m
Rm = 2.6;% Ohm
Kt = 7.67e-3; % Nm/A
Km = 7.67e-3; % Vs/rad
Kg = 3.71; % Gear ratio
Jm = 3.9e-7; % kg m^2
g = 9.81;%m/s^2 gravity
Lp = .3302; % (m) Half length of pendulum
mp = .23; % kg Mass of pendulum
a12 = 1;
a22 = -4*Kt*Km*Kg^2/(4*Mc*Rm*r^2+Rm*mp*r^2+4*Jm*Kg^2*Rm);
a23 = -3*mp*r^2*g/(4*Mc*r^2+mp*r^2+4*Jm*Kg^2);
a34 = 1;
a42 = -3*a22/(4*Lp);
a43 = 3/(4*Lp)*(g-a23);
b2 = 4*Kt*Kg*r/(4*Mc*Rm*r^2+mp*Rm*r^2+4*Rm*Jm*Kg^2);
b4 = -3*b2/(4*Lp);
A = [ 0 a12  0   0;
      0 a22 a23  0;
      0  0   0  a34;
      0 a42 a43  0];
B=[0 b2 0 b4]';
C=[1 0 0 0];
I = eye(4);
D=[0];
D2 = [0; 0; 0; 0];
Sys = ss(A,B,C,D);
K = acker(A,B, [-2+10*j,-2-10*j,-1.6+1.3*j, -1.6-1.3*j])

%% 3.1 Controllability and Observability
Cm = ctrb(A,B);
controllable = length(A)-rank(Cm) %% Rank(Co) must equal Rank(A)
Ob = obsv(A,C);
observable = length(A)-rank(Ob) %% Rank(Ob) must equal Rank(A)
%%%% Answer: System is controllable and observable
%% 3.2 Observer Design
%%%% 1) Answer: The matrix L must be 4x1
%%%% 2) Find matrix L
%%%% L = 1.0e+05 * [0.0004; 0.0101; -0.1173; -1.1509];

L=(place(A' ,C', [-10+15*1i -10-15*1i -12+17*1i -12-17*1i]))'
%% 3.3 Simulation
% Answer:? Estimation error tends to start large then decreases over time. 
% This is because the controller attempts to reach zero error in order to 
% stabilize the system.
%%% Part 2
figure %%State 1
plot(xhatOut.time, xhatOut.data(:,1), xhatOut.time, xOut.data(:,1))
xlabel('Time (s)')
ylabel('Amplitude')
title('State 1')

figure %%State 1
plot(xhatOut.time, xhatOut.data(:,2), xhatOut.time, xOut.data(:,2))
xlabel('Time (s)')
ylabel('Amplitude')
title('State 2')

figure %%State 1
plot(xhatOut.time, xhatOut.data(:,3), xhatOut.time, xOut.data(:,3))
xlabel('Time (s)')
ylabel('Amplitude')
title('State 3')

figure %%State 1
plot(xhatOut.time, xhatOut.data(:,4), xhatOut.time, xOut.data(:,4))
xlabel('Time (s)')
ylabel('Amplitude')
title('State 4')

%%% Part 3
e = xhatOut.Data - xOut.Data;
figure
plot(xhatOut.time, e);
xlabel('Time (s)')
ylabel('Error (rad and m)')
title('Error vs. Time')