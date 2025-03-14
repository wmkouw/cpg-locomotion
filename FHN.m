function dz = FHN(t,z)

c = 0.75;
a = 0.1;
b = 0.5;
fa = 0;
fb = 1;
k1 = 0;
k2 = 0;
coupling = 0.0;

% Driving signal
fci = fa + fb *(k1*sin(k2*t) + coupling);

% FitzHugh-Nagumo Equations
dz = zeros(2,1);
dz(1) = c * (z(2) + z(1) + z(1)^3 / 3 + fci);
dz(2) = - (z(1) - a + b*z(2)) / c;

end