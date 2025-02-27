close all
clear

%% CPG kuramoto phase

%The models used in this file are from the following papers:
%CPG Driven Locomotion Control of Quadruped Robot (Liu et al. 2009)
%A CPG-based gait planning and motion performance analysis for quadruped robot (Wei et al. 2021)

dt = 0.01;
Sim_time = 5;


%Oscillator parameters
w = 10;
lambda = 4;
mu = 10;
R = 2;
%Walking gait
phi = [0 pi pi/2 3*pi/2;
      -pi 0 -pi/2 pi/2;
      -pi/2 pi/2 0 pi;
      -pi/2 -pi/2 -pi 0];



theta = zeros(4,Sim_time/dt);
r = zeros(4,Sim_time/dt);
u = zeros(4,Sim_time/dt);
x = zeros(4,Sim_time/dt);
y = zeros(4,Sim_time/dt);


%initial values
theta(1,1) = 1e-3 * randn(1);
theta(2,1) = 1e-3 * randn(1);
theta(3,1) = 1e-3 * randn(1);
theta(4,1) = 1e-3 * randn(1);

r(1,1) = 1e-3 * randn(1);
r(2,1) = 1e-3 * randn(1);
r(3,1) = 1e-3 * randn(1);
r(4,1) = 1e-3 * randn(1);

%Simulating models
for i = 2:Sim_time/dt
    for j = 1:4
        sine_sum = 0;
        for k = 1:4
            sine_sum = sine_sum + sin(theta(k,i-1) - theta(j,i-1) - phi(j,k));
        end
        theta(j,i) = theta(j,i-1) + dt*(w + lambda*sine_sum);
        r(j,i) = r(j,i-1) + dt*(u(j,i-1));
        u(j,i) = u(j,i-1) + dt*(mu^2 * (R - r(j,i-1)) - 3/2 * mu*u(j,i-1));
        x(j,i-1) = r(j,i-1).*(sin(theta(j,i-1)));
        y(j,i-1) = r(j,i-1).*(cos(theta(j,i-1)));
    end
end

t = linspace(0,Sim_time,Sim_time/dt);

%Plot variables

figure()
subplot(4,1,1)
plot(t,x(1,:))
xlabel('t')
ylabel('x_1')

subplot(4,1,2)
plot(t,x(2,:))
xlabel('t')
ylabel('x_2')

subplot(4,1,3)
plot(t,x(3,:))
xlabel('t')
ylabel('x_3')

subplot(4,1,4)
plot(t,x(4,:))
xlabel('t')
ylabel('x_4')

figure()
hold on
plot(t,x(1,:))
plot(t,x(2,:))
plot(t,x(3,:))
plot(t,x(4,:))
legend('x_1', 'x_2', 'x_3', 'x_4', Location='southeast')

figure()
subplot(4,1,1)
plot(t,y(1,:))
xlabel('t')
ylabel('y_1')

subplot(4,1,2)
plot(t,y(2,:))
xlabel('t')
ylabel('y_2')

subplot(4,1,3)
plot(t,y(3,:))
xlabel('t')
ylabel('y_3')

subplot(4,1,4)
plot(t,y(4,:))
xlabel('t')
ylabel('y_4')

figure()
hold on
plot(t,y(1,:))
plot(t,y(2,:))
plot(t,y(3,:))
plot(t,y(4,:))
legend('y_1', 'y_2', 'y_3', 'y_4', Location='southeast')

