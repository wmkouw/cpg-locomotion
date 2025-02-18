%% 4 Oscillators(Hopf)


dt = 0.01;
Sim_time = 10;


%Oscillator parameters
alpha = 5;
beta = 3;
u = 2;
b = 100;
k = [0 -1 1 -1;
    -1  0 -1 1;
    -1 1  0 -1;
     1 -1 -1 0;];

w_stance = 8;
w_swing = 3;

x = zeros(4,Sim_time/dt);
y = zeros(4,Sim_time/dt);

%initial values
x(1,1) = 0.1;
x(2,1) = 0.2;
x(3,1) = 0.3;
x(4,1) = 0.4;

y(1,1) = 0;
y(2,1) = 0.1;
y(3,1) = 0.2;
y(4,1) = 0.3;

%Simulating models
for i = 2:Sim_time/dt
    for j = 1:4
        w = w_stance/(exp(-b*y(j,i-1))+1) + w_swing/(exp(b*y(j,i-1))+1);
        r = sqrt(x(j,i-1).^2 + y(j,i-1).^2);
        x(j,i) = x(j,i-1) + dt.*(alpha.*(u - r.^2).*x(j,i-1) - w.*y(j,i-1));
    
        
        y(j,i) = y(j,i-1) + dt.*(beta.*(u - r^2).*y(j,i-1) + w.*x(j,i-1) + k(j,:)*y(:,i-1));
    end
end

t = linspace(0,Sim_time,Sim_time/dt);

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