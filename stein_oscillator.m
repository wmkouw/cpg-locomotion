%% Stein Model for Quadrupedal Locomotion
close all; clear;

%% Parameters for the Stein model
p = 10.0;       % Controls the rate of adaptation
b = -2000.0;     % Adaptation parameter (corrected to be positive)
q = 30.0;       % Controls the rate of adaptation
Lambda = -0.2; % Inhibitory coupling strength

%% WALK GAIT
% a = 10.0;
% f = 40.0;
% k1 = 0.0;
% k2 = 0.0;

%% TROT GAIT 
% a = 12.0;
% f = 40.0;
% k1 = 0.1;
% k2 = 57.0;

%% BOUND GAIT
a = 16.0;
f = 50.0;
k1 = 0.1;
k2 = 59.0;

%% Time parameters
T = 10;          % Total simulation time
dt = 0.005;       % Time step
time = 0:dt:T;  % Time vector


%% Initalization 
num_neurons = 4;
x = zeros(num_neurons, length(time)); % Membrane potential for neurons
y = zeros(num_neurons, length(time)); % Adaptation variable y
z = zeros(num_neurons, length(time)); % Adaptation variable z
% Initial conditions: 
x(:, 1) = [0.1; 0.15; 0.2; 0.25];
y(:, 1) = [0.1; 0.12; 0.14; 0.16];
z(:, 1) = [0.1; 0.13; 0.17; 0.19];


% Define inhibitory connections (Z4 symmetry - ring network)
 inhibition_matrix = [
    0, Lambda, 0, 0;       % Neuron 1 inhibits 2
    0, 0, 0, Lambda;       % Neuron 2 inhibits 4
    Lambda, 0, 0, 0;       % Neuron 3 inhibits 1
    0, 0, Lambda, 0];      % Neuron 4 inhibits 3

%% Simulate the Stein model
%% Simulate the Stein model
for t = 2:length(time)
    for i = 1:num_neurons
        % Compute the driving signal fi with INCOMING inhibitory coupling
        incoming_coupling = sum(inhibition_matrix(:, i) .* x(:, t-1));
        fi = f * (1 + k1 * sin(k2 * time(t))) + incoming_coupling;
        
        % Logistic function for adaptation
        adaptation = 1 / (1 + exp(-fi - b * y(i, t-1) + b * z(i, t-1)));

        % Update equations
        dx = dt * (a * (-x(i, t-1) + adaptation));
        dy = dt * (x(i, t-1) - p * y(i, t-1));
        dz = dt * (x(i, t-1) - q * z(i, t-1));

        % Store updated values
        x(i, t) = x(i, t-1) + dx;
        y(i, t) = y(i, t-1) + dy;
        z(i, t) = z(i, t-1) + dz;
    end
end
%% Plot individual neuron activities
figure()
for i = 1:num_neurons
    subplot(4,1,i)
    plot(time, x(i, :));
    xlabel('Time (s)');
    ylabel(['x_', num2str(i)]);
    legend(['Neuron ', num2str(i)]);
end

%% Plot all neuron activities together
figure()
hold on
for i = 1:num_neurons
    plot(time, x(i, :));
end
xlabel('Time (s)');
ylabel('Neuron Activity');
title('Stein Model Neural Oscillators');
legend('Neuron 1', 'Neuron 2', 'Neuron 3', 'Neuron 4');
grid on;
hold off;
