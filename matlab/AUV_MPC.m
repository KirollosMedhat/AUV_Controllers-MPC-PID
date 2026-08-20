clear 
close all
clc
addpath('D:\ASMarine\casadi-windows-matlabR2016a-v3.5.5')
import casadi.*

T = 0.1;
N = 30;
rob_dim = 0.2;
AUV_radius = 0.37/2;      %distance between CG and thruster

force_max_T1 = 12; force_min_T1 = - force_max_T1;
force_max_T2 = 12; force_min_T2 = -force_max_T2;

m = 25.3;   %mass
Xu_dot = 55.6; %added mass coeff
Xuu = 222.416;    %drag coeff
Nrr = 3.16723;    %rotational drag coeff
Izz = 5.1456;    %Inertia around Z axis
Nr_dot = 4.0; %added inertia coeff around z axis
x = SX.sym('x'); y = SX.sym('y'); psi = SX.sym('psi');  %global positions
u = SX.sym('u'); v = SX.sym('v'); r = SX.sym('r');      %velocities

states = [x;
    y;
    psi;
    u;
    v;
    r];   

n_states = length(states);

T1 = SX.sym('T1');
T2 = SX.sym('T2');

T_total = T1*cos(pi/4) + T2*cos(pi/4);  %resultant in forward direction

% kan feh 8alteteen f 7esbt el torque
%1) el sign kont 3amelha plus w hya el mafrood minus
%2) el radius kan r zyo zy el yaw fa 8yart esmo le AUV_radius
Torque = T1*AUV_radius - T2*AUV_radius; %Torque generated from thrusters

controls = [T1;T2];  n_controls = length(controls);

Xddot = (( -Xuu*u*abs(u)*cos(psi) + T_total * cos(psi) )/(m + Xu_dot)); 
Yddot = (( -Xuu*u*abs(u)*sin(psi) + T_total * sin(psi) )/(m + Xu_dot)); 
psiddot = (-Nrr * r*abs(r) + Torque)/(Izz + Nr_dot);


rhs = [ u*cos(psi);
        u*sin(psi);
        r;
        Xddot;
        Yddot;
        psiddot];


%da el rhs b3d edit mina 
% rhs = [ u*cos(psi) + T*Xddot;
%         u*sin(psi) + T*Yddot;
%         r + T*psiddot;
%         Xddot;
%         Yddot;
%         psiddot];


% da el rhs el awalany

% rhs = [ u + T*Xddot;
%     v + T*Yddot;
%     r + T*psiddot;
%     Xddot;
%     Yddot;
%     psiddot];

f = Function('f', {states,controls}, {rhs});

U = SX.sym('U',n_controls,N);
P = SX.sym('P',n_states + n_states);
X = SX.sym('X', n_states,(N+1));

X(:,1) = P(1:6);

for k = 1:N
    st = X(:,k); con = U(:,k);
    f_value = f(st,con);
    st_next = st + (T*f_value);
    X(:,k+1) = st_next;
end


%reference el tuning so far:
%Q(1,1) = 59000;Q(2,2) = 31000;Q(3,3) = 50000;
%Q(1,1) = 70000;Q(2,2) = 70000;Q(3,3) = 500;
%Q = zeros(3,3); Q(1,1) = 2500;Q(2,2) = 1500;Q(3,3) = 20;
%Q = zeros(3,3); Q(1,1) = 9500;Q(2,2) = 9500;Q(3,3) = 20;

%a7sn 7aga so far:
%Q = zeros(3,3); Q(1,1) = 25000;Q(2,2) = 25000;Q(3,3) = 80;


%3ayzeen nekamel mn el arkam de:
%Q = zeros(3,3); Q(1,1) = 25000;Q(2,2) = 25000;Q(3,3) = 8000;
ff=Function('ff',{U,P},{X});

obj = 0;
g = [];
display(g)  
Q = zeros(3,3); Q(1,1) = 25000;Q(2,2) = 25000;Q(3,3) = 100;
%Q(4,4) = 4; Q(5,5) = 4;
%Q(6,6) = 4;
R = zeros(2,2); R(1,1) = 0.5; R(2,2) = 0.5;

for k=1:N
st = X(1:3,k);
obj = obj+(st-P(7:9))'*Q*(st-P(7:9)) ; % calculate obj
end

for k = 1:N+1 % box constraints due to the map margins
g = [g ; X(1,k)]; %state x
g = [g ; X(2,k)]; %state y
display(g)
end

OPT_variables = reshape(U,2*N,1);

nlp = struct ('x', OPT_variables, 'f',obj,'g',g,'p',P);

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level = 0;
opts.print_time = 0;
opts.ipopt.acceptable_tol = 1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt',nlp,opts);

args = struct;


args.lbg = -50;
args.ubg = 50;

args.lbx(1:2:2*N-1,1) = force_min_T1 ; args.lbx (2:2:2*N,1) = force_min_T2;
args.ubx(1:2:2*N-1,1) = force_max_T1 ; args.ubx (2:2:2*N,1) = force_max_T2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


t0 = 0;
x0 = [0 ; 0 ; 0; 0.0; 0.0; 0.0];    % initial condition.
xs = [1.5;1.5;0; 0; 0; 0]; % Reference posture.

xx(:,1) = x0; % xx contains the history of states
t(1) = t0;

u0 = zeros(N,2);  % two control inputs 

sim_tim = 30; % Maximum simulation time

%settling = 17.3 sec
%steady state error approx. = 0
%overshoot:
%       x = 1.41
%       y = 1.653
%       theta = 2.6607
%steady state:
%       x = 1.5001
%       y = 1.497
%       theta = 0.0009



% Start MPC
mpciter = 0;
xx1 = [];
u_cl=[];


% the main simulaton loop... it works as long as the error is greater
% than 10^-2 and the number of mpc steps is less than its maximum
% value.
main_loop = tic;
while(norm((x0-xs),2) > 1e-2 && mpciter < sim_tim / T)
    args.p   = [x0;xs]; % set the values of the parameters vector
    args.x0 = reshape(u0',2*N,1); % initial value of the optimization variables
    %tic
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);    
    %toc
    u = reshape(full(sol.x)',2,N)';
    ff_value = ff(u',args.p); % compute OPTIMAL solution TRAJECTORY
    xx1(:,1:6,mpciter+1)= full(ff_value)';
    
    u_cl= [u_cl ; u(1,:)];
    t(mpciter+1) = t0;
    [t0, x0, u0] = shift(T, t0, x0, u,f); % get the initialization of the next optimization step
    
    xx(:,mpciter+2) = x0;
    display(mpciter);
    mpciter = mpciter + 1;
end

Draw_MPC_point_stabilization_v1 (t,xx,xx1,u_cl,xs,N,rob_dim)

