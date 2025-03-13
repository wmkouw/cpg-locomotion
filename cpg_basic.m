%% Clear

close all;
clearvars;

%% Initialization
% Time
dt = 0.01;
len_time = 1000;
tsteps = 0:dt:(dt*(len_time-1));

% Parameters
alpha = 1.0;
beta = 1.0;
mu = 0.5;
b = 100;
K = [ 0 -1  1 -1;
     -1  0 -1  1;
     -1  1  0 -1;
      1 -1 -1  0];
omega_stance = 3;
omega_swing  = 3;

% Preallocate
num_legs = 4;
x_ = zeros(num_legs,len_time);
y_ = zeros(num_legs,len_time);

% Initialize output x and y
x_kmin1 = 1e-3*randn(num_legs,1);
y_kmin1 = 1e-3*randn(num_legs,1);


%% computing
for k = 1:len_time

    for i = 1:num_legs

        ri2 = x_kmin1(i)^2 + y_kmin1(i)^2;
        omega_i  = omega_stance / (exp(-b*y_kmin1(i)) + 1) + omega_swing / (exp(-b*y_kmin1(i)) + 1);

        x_(i,k) = x_kmin1(i) + dt*( alpha*(mu - ri2)*x_kmin1(i) - omega_i*y_kmin1(i) );
        y_(i,k) = y_kmin1(i) + dt*( beta*(mu - ri2)*y_kmin1(i) + omega_i*y_kmin1(i) + K(i,:)*y_kmin1 );

    end

    x_kmin1 = x_(:,k);
    y_kmin1 = y_(:,k);
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
figure(2)
set(gcf, 'Position', [200 300 1000 800])

for i = 1:num_legs
    
    subplot(2,2,i)
    scatter(x_(i,:), y_(i,:), 'k.')
    xlabel(['x_' num2str(i) ' (s)'])
    ylabel(['y_' num2str(i) ' (s)'])

end