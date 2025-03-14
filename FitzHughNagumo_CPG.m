
close all
clear

%% FitzHugh-Nagumo

% Time
dt = 0.01;
len_time = 1000;
tsteps = 0:dt:(dt*(len_time-1));

% Parameters
c = 0.75;
a = 0.1;
b = 0.5;
fa = 0;
fb = 1;
k1 = 0;
k2 = 0;
Lambda = [ 0.0  -0.2  0.0  0.0;
           0.0  0.0  0.0  -0.2;
          -0.2  0.0  0.0  0.0;
           0.0  0.0  -0.2  0];

% Preallocate
num_legs = 4;
x_ = zeros(num_legs,len_time);
y_ = zeros(num_legs,len_time);

% Initialize 
x_kmin1 = 1e-3*randn(num_legs,1);
y_kmin1 = 1e-3*randn(num_legs,1);

for k = 1:len_time

    for i = 1:num_legs

        % Coupling effect from other oscillators
        coupling = Lambda(i, :) * x_kmin1;

        % Driving signal (fixed or sinusoidal drive)
        fci = fa + fb * (k1 * sin(k2 * tsteps(k)) + coupling);

        % FitzHugh-Nagumo Equations
        x_(i, k) = x_kmin1(i) + dt * (c * (y_kmin1(i) + x_kmin1(i) + (x_kmin1(i)^3) / 3 + fci));
        y_(i, k) = y_kmin1(i) - dt * ((x_kmin1(i) - a + b * y_kmin1(i)) / c);
    end

    % Update previous step values
    x_kmin1 = x_(:, k);
    y_kmin1 = y_(:, k);
end

%% Plot all oscillators over time

figure(1)
set(gcf, 'Position', [100 200 1500 500])

subplot(121)
plot(tsteps, x_')
xlabel('time (s)')
ylabel('x_i(t)')

subplot(122)
plot(tsteps, y_')
xlabel('time (s)')
ylabel('y_i(t)')

%% Plot each oscillator over space

% figure(2)
% set(gcf, 'Position', [200 300 1000 800])
% 
% for i = 1:num_legs
%     
%     subplot(2,2,i)
%     scatter(x_(i,:), y_(i,:), 'k.')
%     xlabel(['x_' num2str(i) ' (s)'])
%     ylabel(['y_' num2str(i) ' (s)'])
% 
% end

%% Attempt with MATLAB's ODE solver

t0 = 0;
tfinal = 10;
z0 = [0.5, 0];
[t,z] = ode45(@FHN, [t0 tfinal], z0);

plot(t,z(:,1));
