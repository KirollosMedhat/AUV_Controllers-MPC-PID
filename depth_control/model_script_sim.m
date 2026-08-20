clear;clc;
global E_old;
global E_int;
E_int = 0;
E_old = 0;

m = 25.3;
added_mass = 269;
g = 9.81;

samp_time = 0.01;
sim_time = 150;

timeline = linspace(0, sim_time, (sim_time/samp_time)+1);
z = zeros(1, (sim_time/samp_time)+1);
z_dot = zeros(1, (sim_time/samp_time)+1);

setpoint = 3;
Ft = zeros(1, (sim_time/samp_time)+1);

for i = 1:(sim_time/samp_time)
    z_ddot = (0.2 - Ft(i))/(m + added_mass);
    z_dot(i+1) = z_dot(i) + z_ddot*samp_time;
    z(i+1) = z(i) + z_dot(i)*samp_time;
    Ft(i+1) = Discrete_PID(54.6, 0, 286.4, z(i) - setpoint, samp_time);
    
end

plot(timeline, z);
