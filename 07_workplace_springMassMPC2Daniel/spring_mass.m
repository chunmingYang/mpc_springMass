clc
clear all
close all

% model description
n = 2; % state
m = 1; % input
mass = 1;
k = 10;
g = 9.8;

% dynamic matrixs
A = [0, 1;
     -k/mass, 0];
B = [1/mass];

% % convert to discrete-time system
% ts = 0.5;       % sampling time
% A = expm(ts*A);
% B = (A\(expm(ts*A)-eye(n)))*B;

% objective matrixs
Q = eye(n);      
R = eye(m);  
% R = zeros(m);

% inequality constraints
xmin = -10*ones(n,1);
xmax = 10*ones(n,1);
umin = -0*ones(m,1);
umax = 100*ones(m,1);

% initial state
x0 = zeros(n,1);
% x0 = [0.1;0.1];

% system description
sys.A = A ;
sys.B = B;
sys.xmax = xmax;
sys.xmin = xmin;
sys.umax = umax;
sys.umin = umin;
sys.n = n;
sys.m = m;
sys.Q = Q;
sys.R = R;

% fast MPC parameters
nsteps = 100;
T = 10;                % time horizon
params.T = T; 
params.Qf = Q;         % final state cost
params.kappa = 0.01;   % barrier parameter
params.niters = 5;     % number of newton steps
params.quiet = false;
Xhist = zeros(n,nsteps);  % state
Uhist = zeros(m,nsteps);  % input

% set up initial state and input trajectories
X = zeros(n, T);  % warm start X trajectory (reference)
% X = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
%      0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
U = zeros(m, T);    % warm start U trajectory (reference)
% U = 8*ones(m,T);  % warm start U trajectory
% U
x = x0;  % initial states

% MPC
for i = 1:nsteps

    [X,U,telapsed] = fmpc_step(sys,params,X,U,x);

    % control update
    U
    u = U(:,1);
    u

    % recording data
%     Xhist(:,i) = x; 
%     Uhist(:,i) = u;
    

%     % control update
%     u = U(:,1);

    % state update
    x = A*x + B*u;
%     x
%     X
    
    % reference trajectory update
    X = [];
%     X = [X(:,2:T),zeros(n,1)];
%     U = 9.8*ones(m,T);
%     X = zeros(n, T);  % warm start X trajectory (reference)
%     U = zeros(m, T);    % warm start U trajectory (reference)
end
% Xhist
% Uhist
% Uhist


% main iteration loop
% for i = 1:nsteps
%     [X,U,telapsed] = fmpc_step(sys,params,X,U,x);
%     u = U(:,1);
%     record state, input, stage cost, and fmpc run time
%     Xhist(:,i) = x; Uhist(:,i) = u;
%     Jhist(i) = x'*Q*x+u'*R*u;
%     thist(i) = telapsed;
%    
%     state update
%     x = A*x+B*u+w(:,i);
% 
%     shift previous state and input trajectories 
%     for warm start in next step
%     X = [X(:,2:T),zeros(n,1)];
%     U = [U(:,2:T),zeros(m,1)];
% end

