clc
close all
clear all

%% code description: mpc trajectory tracking (states: position and velocity) on simple spring mass system (mass on flat ground, mass one side attached to fixed end spring, then one horizontal force acted on mass)

%% MPC parameters
ts = 0.01;          % time step
T = 5;              % receding horizon can be tuned
iter = 200;         % iteration times, thus we have total_time = 200*0.01

%% spring-mass model description
% dynamic model in states space: x(t+1) = Ax(t) + Bu(t) + w(t)
mass = 1;           % mass of model
k = 10;             % spring stiffness 
n = 2;              % number of states
m = 1;              % number of inputs
A = [1,        ts;
    -(k/m)*ts, 1];  % dynamic matrix
B = [0;     ts/m];

%% initialization
x = 0.1;            % state_1_begin - position
xf = 0.2;           % state_1_end   - position (this is for trajectory generation)
xdot = 0;           % state_2_begin - velocity
X = [x; xdot];      % states initialization
U = zeros(1, T);    % inputs initialization

%% cubic trajectory generation
t0 = 0;
tf = ts*iter;
y0 = x;             % position start position
yf = xf;            % position end position

a0 = (yf*t0*t0*(3*tf-t0) + y0*tf*tf*(tf-3*t0))/((tf-t0)*(tf-t0)*(tf-t0));
a1 = 6*t0*tf*(y0-yf)/((tf-t0)*(tf-t0)*(tf-t0));
a2 = 3*(t0+tf)*(yf-y0)/((tf-t0)*(tf-t0)*(tf-t0));
a3 = 2*(y0-yf)/((tf-t0)*(tf-t0)*(tf-t0));
yhis = [];
ydothis = [];

%% cost function - quadratic optimization problrm
Q = diag([1000 1]); % this can be tuned
R = zeros(m);       % inputs cost matrix set to zero --- since we don't have the target input trajectory to optimize then we set zero
J = 0;              % initialize cost function

%% data saving purpose
xhis = [];
xdothis = [];

%% MPC main loop
for i = 1 : iter
    Aieq = [];      % since no ineuqality constraint
    bieq = [];      % since no inequality constraint
    Aeq = [];       % since no equality constraint
    beq = [];       % since no equality constraint
    lb = 0*ones(1, T);   % low boundary for controls
    ub = 100*ones(1, T); % up boundary for controls

    % cubic trajectory profile
    y = a0 + a1*(i*ts) + a2*(i*ts)*(i*ts) + a3*(i*ts)*(i*ts)*(i*ts);
    ydot = a1 + 2*a2*(i*ts) + 3*a3*(i*ts)*(i*ts);
    yhis(i) = y;
    ydothis(i) = ydot;
    Xref = [y; ydot];

    % core of the mpc
    u = fmincon(@(U)cost_fun(X,U,Xref,A,B,Q,R),U,Aieq,bieq,Aeq,beq,lb,ub);
    X = A*X + B*u(1);

    % data saving
    xhis(i) = X(1);
    xdothis(i) = X(2); 
end
subplot(1,2,1)
plot(1:length(xhis), xhis); hold on;
plot(1:length(yhis), yhis)

subplot(1,2,2)
plot(1:length(xdothis), xdothis); hold on;
plot(1:length(ydothis), ydothis)