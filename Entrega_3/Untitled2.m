syms s t
X1 = 1/s;
K = 1;
p = 1;
Kp = 1;
taud = 10;
taui = 20;
t=0:0.01:100;
scaler = 1;
num = (Kp*K*taud)*(s^2 + s/taud + 1/(taud*taui));
den = s^2 *(s+p) + K*Kp*taud * (s^2 + s/taud+ 1/(taud*taui));
%H = (Kp*K)/(s^2 + (p+Kp*K*taud)*s + Kp*K);  % CONTROLADOR P-D
H = num/den;                               % CONTROLADOR PID
Y1 = H* X1;
yaux =(ilaplace(Y1));
y = matlabFunction(yaux);

plot(t/scaler,y(t/scaler));


