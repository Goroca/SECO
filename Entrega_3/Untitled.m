%% P
close all
clear
K = 1166.89;
p = 68;
Kp = [1,2,4,8,16];
t=0:0.001:5; % set the time you want to plot
u1=t; %set the funtion ramp X(s)= 1/s^2
u2=t.^2; %set the funtion parabole X(s)= 1/s^3
scaler = 10;    % Para reducir el tiempo de la señal

plotHandlesy = zeros(1,length(Kp));
plotLabelsy = cell(1,length(Kp));
plotHandlese1 = zeros(1,length(Kp));
plotLabelse1 = cell(1,length(Kp));
plotHandlese2 = zeros(1,length(Kp));
plotLabelse2 = cell(1,length(Kp));
plotHandlese3 = zeros(1,length(Kp));
plotLabelse3 = cell(1,length(Kp));

for i=1:length(Kp)
    Hnum = Kp(i)*K; % vector [a*s^0   b*s^1    c*s^3,....]
    Hden = [1  p  Kp(i)*K]; % vector [a*s^0    b*s^1    c*s^3,....]
    HeNum = [1 p 0];
    HeDen = Hden;
    
    y = step(Hnum,Hden,t/scaler); %step solution -> X(s)= 1/s
    e0 = step(HeNum,HeDen,t/scaler);
    e1 =lsim(HeNum,HeDen,u1,t);
    e2 =lsim(HeNum,HeDen,u2,t);
    
    figure(1);
    title("Output");
    hold on;
    plotHandlesy(i) = plot(t/scaler,y);
    plotLabelsy{i} = ['Kp = ',num2str(Kp(i))];

    figure(2);
    title("Error a Función Escalón");
    hold on;
    plotHandlese1(i) = plot(t/scaler,e0);
    plotLabelse1{i} = ['Kp = ',num2str(Kp(i))];

    figure(3);
    title("Error a Función Rampa");
    hold on;
    plotHandlese2(i) = plot(t,e1);
    plotLabelse2{i} = ['Kp = ',num2str(Kp(i))];

    figure(4);
    title("Error a Función Parábola");
    hold on;
    plotHandlese3(i) = plot(t,e2);
    plotLabelse3{i} = ['Kp = ',num2str(Kp(i))];
end
legend(plotHandlesy,plotLabelsy);
legend(plotHandlese1,plotLabelse1);
legend(plotHandlese2,plotLabelse2);
legend(plotHandlese3,plotLabelse3);

hold off
%% P Regimen Permanente
close all
clear
syms s t
X1 = 1/s;
X2 = 1/s^2;
X3 = 1/s^3;

syms K p Kp

He = (s^2 +s*p)/(s^2 + p*s + Kp*K);

Ye1 = He*X1;
Ye2 = He*X2;
Ye3 = He*X3;

e0= limit(s*Ye1,s,0);
e1= limit(s*Ye2,s,0);
e2= limit(s*Ye3,s,0);

%% PD
close all
clear
K = 1166.89;
p = 68;
Kp = [1,2,4,8,16];
taud = [0.005,1];
t=0:0.01:20;
graphics = 4;
u1=t;
u2=t.^2;
scaler = 20;     % Para reducir el tiempo de la señales

plotHandlesy = zeros(1,length(Kp));
plotLabelsy = cell(1,length(Kp));
plotHandlese1 = zeros(1,length(Kp));
plotLabelse1 = cell(1,length(Kp));
plotHandlese2 = zeros(1,length(Kp));
plotLabelse2 = cell(1,length(Kp));
plotHandlese3 = zeros(1,length(Kp));
plotLabelse3 = cell(1,length(Kp));

for m=1:length(taud)
    for i=1:length(Kp)
        Hnum = [Kp(i)*K*taud(m) Kp(i)*K];
        Hden = [1 (p+Kp(i)*K*taud(m)) Kp(i)*K];
        HeNum = [1 p 0];
        HeDen = Hden;
    
        y = step(Hnum,Hden,t/scaler);
        e0 = step(HeNum,HeDen,t/scaler);
        e1 = lsim(HeNum,HeDen,u1,t);
        e2 = lsim(HeNum,HeDen,u2,t);
        
        figure(1 + (m-1)*graphics);
        hold on;
        title(['Output - \tau_D = ', num2str(taud(m))]);
        plotHandlesy(i) = plot(t/scaler,y);
        plotLabelsy{i} = ['Kp = ',num2str(Kp(i))];

        figure(2 + (m-1)*graphics);
        title(['Error a Función Escalón - \tau_D = ', num2str(taud(m))]);
        hold on;
        plotHandlese1(i) = plot(t/scaler,e0);
        plotLabelse1{i} = ['Kp = ',num2str(Kp(i))];

        figure(3 + (m-1)*graphics);
        title(['Error a Función Rampa - \tau_D = ', num2str(taud(m))]);
        hold on;
        plotHandlese2(i) = plot(t,e1);
        plotLabelse2{i} = ['Kp = ',num2str(Kp(i))];

        figure(4 + (m-1)*graphics);
        title(['Error a Función Parábola - \tau_D = ', num2str(taud(m))]);
        hold on;
        plotHandlese3(i) = plot(t,e2);
        plotLabelse3{i} = ['Kp = ',num2str(Kp(i))];
    end
    legend(plotHandlesy,plotLabelsy);
    legend(plotHandlese1,plotLabelse1);
    legend(plotHandlese2,plotLabelse2);
    legend(plotHandlese3,plotLabelse3);
end
hold off
%% PD Régimen Permanente
close all
clear
syms s t
X1 = 1/s;
X2 = 1/s^2;
X3 = 1/s^3;
syms K p Kp taud

He = (s^2+p*s)/(s^2 + (p+Kp*K*taud)*s + Kp*K);

E1 = He* X1;
E2 = He* X2;
E3 = He* X3;

e0= limit(s*E1,s,0); %#OK
e1= limit(s*E2,s,0); %#OK
e2= limit(s*E3,s,0);%#OK

%% P-D
close all
clear
K = 1166.89;
p = 68;
Kp = [10,20,40,80,160,320];
taud = [0.02,0.001];
t=0:0.01:20;
graphics = 4;
u1=t;
u2=t.^2;
scaler = 100;    % Para reducir el tiempo de la señal

plotHandlesy = zeros(1,length(Kp));
plotLabelsy = cell(1,length(Kp));
plotHandlese1 = zeros(1,length(Kp));
plotLabelse1 = cell(1,length(Kp));
plotHandlese2 = zeros(1,length(Kp));
plotLabelse2 = cell(1,length(Kp));
plotHandlese3 = zeros(1,length(Kp));
plotLabelse3 = cell(1,length(Kp));

for m=1:length(taud)
    for i=1:length(Kp)
        Hnum = Kp(i)*K;
        Hden = [1 (p+Kp(i)*K*taud(m)) Kp(i)*K];
        HeNum = [1 (p+Kp(i)*K*taud(m)) 0];
        HeDen = Hden;
    
        y = step(Hnum,Hden,t/scaler);
        e0 = step(HeNum,HeDen,t/scaler);
        e1 = lsim(HeNum,HeDen,u1,t);
        e2 = lsim(HeNum,HeDen,u2,t);
        
        figure(1 + (m-1)*graphics);
        hold on;
        title(['Output - \tau_D = ', num2str(taud(m))]);
        plotHandlesy(i) = plot(t/scaler,y);
        plotLabelsy{i} = ['Kp = ',num2str(Kp(i))];

        figure(2 + (m-1)*graphics);
        title(['Error a Función Escalón - \tau_D = ', num2str(taud(m))]);
        hold on;
        plotHandlese1(i) = plot(t/scaler,e0);
        plotLabelse1{i} = ['Kp = ',num2str(Kp(i))];

        figure(3 + (m-1)*graphics);
        title(['Error a Función Rampa - \tau_D = ', num2str(taud(m))]);
        hold on;
        plotHandlese2(i) = plot(t,e1);
        plotLabelse2{i} = ['Kp = ',num2str(Kp(i))];

        figure(4 + (m-1)*graphics);
        title(['Error a Función Parábola - \tau_D = ', num2str(taud(m))]);
        hold on;
        plotHandlese3(i) = plot(t,e2);
        plotLabelse3{i} = ['Kp = ',num2str(Kp(i))];
    end
    legend(plotHandlesy,plotLabelsy);
    legend(plotHandlese1,plotLabelse1);
    legend(plotHandlese2,plotLabelse2);
    legend(plotHandlese3,plotLabelse3);
end
hold off
%% P-D Régimen Permanente
close all
clear
syms s t
X1 = 1/s;
X2 = 1/s^2;
X3 = 1/s^3;
syms K p Kp taud

He = (s^2+(p+Kp*taud)*s)/(s^2 + (p+Kp*K*taud)*s + Kp*K);

E1 = He* X1;
E2 = He* X2;
E3 = He* X3;

e0= limit(s*E1,s,0); %#OK
e1= limit(s*E2,s,0); %#OK
e2= limit(s*E3,s,0);%#OK
%% PI
close all
clear
K = 1166.89;
p = 68;
Kp = [0.75,1.5,3,6,12];
taui = [0.5,1];
t=0:0.001:8;
graphics = 4;
u1=t;
u2=t.^2;
scaler = 5;     % Para reducir el tiempo de la señales

plotHandlesy = zeros(1,length(Kp));
plotLabelsy = cell(1,length(Kp));
plotHandlese1 = zeros(1,length(Kp));
plotLabelse1 = cell(1,length(Kp));
plotHandlese2 = zeros(1,length(Kp));
plotLabelse2 = cell(1,length(Kp));
plotHandlese3 = zeros(1,length(Kp));
plotLabelse3 = cell(1,length(Kp));

for m=1:length(taui)
    for i=1:length(Kp)
        Hnum = [Kp(i)*K   Kp(i)*K/taui(m)];
        Hden = [1 p Kp(i)*K*taui(m) Kp(i)*K/taui(m)];
        HeNum = [1 p 0 0];
        HeDen = Hden;
    
        y = step(Hnum,Hden,t/scaler);
        e0 = step(HeNum,HeDen,t/scaler);
        e1 = lsim(HeNum,HeDen,u1,t);
        e2 = lsim(HeNum,HeDen,u2,t);
        
        figure(1 + (m-1)*graphics);
        hold on;
        title(['Output - \tau_I = ', num2str(taui(m))]);
        plotHandlesy(i) = plot(t/scaler,y);
        plotLabelsy{i} = ['Kp = ',num2str(Kp(i))];

        figure(2 + (m-1)*graphics);
        title(['Error a Función Escalón - \tau_I = ', num2str(taui(m))]);
        hold on;
        plotHandlese1(i) = plot(t/scaler,e0);
        plotLabelse1{i} = ['Kp = ',num2str(Kp(i))];

        figure(3 + (m-1)*graphics);
        title(['Error a Función Rampa - \tau_I = ', num2str(taui(m))]);
        hold on;
        plotHandlese2(i) = plot(t,e1);
        plotLabelse2{i} = ['Kp = ',num2str(Kp(i))];

        figure(4 + (m-1)*graphics);
        title(['Error a Función Parábola - \tau_I = ', num2str(taui(m))]);
        hold on;
        plotHandlese3(i) = plot(t,e2);
        plotLabelse3{i} = ['Kp = ',num2str(Kp(i))];
    end
    legend(plotHandlesy,plotLabelsy);
    legend(plotHandlese1,plotLabelse1);
    legend(plotHandlese2,plotLabelse2);
    legend(plotHandlese3,plotLabelse3);
end
hold off
%% PI Régimen Permanente
close all
clear
syms s t
X1 = 1/s;
X2 = 1/s^2;
X3 = 1/s^3;
syms K p Kp taui

He = (s^2+p*s)/(s^2 + (p+Kp*K*taui)*s + Kp*K);

E1 = He* X1;
E2 = He* X2;
E3 = He* X3;

e0= limit(s*E1,s,0); %#OK
e1= limit(s*E2,s,0); %#OK
e2= limit(s*E3,s,0);%#OK


%% PID
close all
clear
K = 1166.89;
p = 68;
Kp = [4,8,16,32,64];
taud = [0.005,0.02];
taui = [0.05,0.5];

% SOLO PINTA LAS FIGURAS DISTINTAS DE 0
FIGURA_OUT = 0;
FIGURA_ESCALON = 0;
FIGURA_RAMPA = 0;
FIGURA_PARABOLA = 1;


t=0:0.01:2.5;
u1=t;
u2=t.^2;
graphics = 4;
scaler = 10;     % Para reducir el tiempo de la señal

plotHandlesy = zeros(1,length(Kp));
plotLabelsy = cell(1,length(Kp));
plotHandlese1 = zeros(1,length(Kp));
plotLabelse1 = cell(1,length(Kp));
plotHandlese2 = zeros(1,length(Kp));
plotLabelse2 = cell(1,length(Kp));
plotHandlese3 = zeros(1,length(Kp));
plotLabelse3 = cell(1,length(Kp));

for n=1:length(taui)
    for m=1:length(taud)
        for i=1:length(Kp)
            Hnum = [(Kp(i)*K*taud(m))   Kp(i)*K  (Kp(i)*K/taui(n))];
            Hden = [ 1  (p+(Kp(i)*K*taud(m))) K*Kp(i) K*Kp(i)/taui(n)];
            HeNum = [1 p 0 0];
            HeDen = Hden;
    
            y = step(Hnum,Hden,t/scaler);
            e0 = step(HeNum,HeDen,t);
            e1 = lsim(HeNum,HeDen,u1,t);
            e2 = lsim(HeNum,HeDen,u2,t*2);
            
            if FIGURA_OUT ~= 0
            figure(1 + (m-1)*graphics + (n-1)*length(taud)*graphics);
            hold on;
            title(['Output - \tau_D = ', num2str(taud(m)),' y \tau_I = ',num2str(taui(n))]);
            plotHandlesy(i) = plot(t/scaler,y);
            plotLabelsy{i} = ['Kp = ',num2str(Kp(i))];
            end
            if FIGURA_ESCALON ~= 0
            figure(2 + (m-1)*graphics + (n-1)*length(taud)*graphics);
            title(['Error a Función Escalón - \tau_D = ', num2str(taud(m)),' y \tau_I = ',num2str(taui(n))]);
            hold on;
            plotHandlese1(i) = plot(t,e0);
            plotLabelse1{i} = ['Kp = ',num2str(Kp(i))];
            end
            if FIGURA_RAMPA ~= 0
            figure(3 + (m-1)*graphics + (n-1)*length(taud)*graphics);
            title(['Error a Función Rampa - \tau_D = ', num2str(taud(m)),' y \tau_I = ',num2str(taui(n))]);
            hold on;
            plotHandlese2(i) = plot(t,e1);
            plotLabelse2{i} = ['Kp = ',num2str(Kp(i))];
            end
            if FIGURA_PARABOLA ~= 0
            figure(4 + (m-1)*graphics + (n-1)*length(taud)*graphics);
            title(['Error a Función Parábola - \tau_D = ', num2str(taud(m)),' y \tau_I = ',num2str(taui(n))]);
            hold on;
            plotHandlese3(i) = plot(t*2,e2);
            plotLabelse3{i} = ['Kp = ',num2str(Kp(i))];
            end
        end
        if FIGURA_OUT ~= 0
        legend(plotHandlesy,plotLabelsy);
        end
        if FIGURA_ESCALON ~= 0
        legend(plotHandlese1,plotLabelse1);
        end
        if FIGURA_RAMPA ~= 0
        legend(plotHandlese2,plotLabelse2);
        end
        if FIGURA_PARABOLA ~= 0
        legend(plotHandlese3,plotLabelse3);
        end  
    end
end

hold off

%% PID Régimen Permanente
close all
clear
syms s t
X1 = 1/s;
X2 = 1/s^2;
X3 = 1/s^3;
syms K p Kp taud taui

HeNum = (s^3 + s^2*p);
HeDen =s^2 *(s+p) + K*Kp*taud * (s^2 + s/taud+ 1/(taud*taui));
He = HeNum/HeDen;

E1 = He* X1;
E2 = He* X2;
E3 = He* X3;

e0= limit(s*E1,s,0); %#OK
e1= limit(s*E2,s,0); %#OK
e2= limit(s*E3,s,0);%#OK
%% PI-D
close all
clear
K = 1166.89;
p = 68;
Kp = [4,8,16,32,64];
taud = [0.005,0.02];
taui = [0.05,0.5];
% SOLO PINTA LAS FIGURAS DISTINTAS DE 0
FIGURA_OUT = 0;
FIGURA_ESCALON = 0;
FIGURA_RAMPA = 0;
FIGURA_PARABOLA = 1;


t=0:0.01:2.5;
u1=t;
u2=t.^2;
graphics = 4;
scaler = 10;     % Para reducir el tiempo de la señal

plotHandlesy = zeros(1,length(Kp));
plotLabelsy = cell(1,length(Kp));
plotHandlese1 = zeros(1,length(Kp));
plotLabelse1 = cell(1,length(Kp));
plotHandlese2 = zeros(1,length(Kp));
plotLabelse2 = cell(1,length(Kp));
plotHandlese3 = zeros(1,length(Kp));
plotLabelse3 = cell(1,length(Kp));

for n=1:length(taui)
    for m=1:length(taud)
        for i=1:length(Kp)
            Hnum = [Kp(i)*K   (Kp(i)*K/taui(n))];
            Hden = [ 1  (p+(Kp(i)*K*taud(m))) K*Kp(i) K*Kp(i)/taui(n)];
            HeNum = [1 (p+K*Kp(i)*taud(m)) 0 0];
            HeDen = Hden;
    
            y = step(Hnum,Hden,t/scaler);
            e0 = step(HeNum,HeDen,t);
            e1 = lsim(HeNum,HeDen,u1,t);
            e2 = lsim(HeNum,HeDen,u2,t*2);
            
            if FIGURA_OUT ~= 0
            figure(1 + (m-1)*graphics + (n-1)*length(taud)*graphics);
            hold on;
            title(['Output - \tau_D = ', num2str(taud(m)),' y \tau_I = ',num2str(taui(n))]);
            plotHandlesy(i) = plot(t/scaler,y);
            plotLabelsy{i} = ['Kp = ',num2str(Kp(i))];
            end
            if FIGURA_ESCALON ~= 0
            figure(2 + (m-1)*graphics + (n-1)*length(taud)*graphics);
            title(['Error a Función Escalón - \tau_D = ', num2str(taud(m)),' y \tau_I = ',num2str(taui(n))]);
            hold on;
            plotHandlese1(i) = plot(t,e0);
            plotLabelse1{i} = ['Kp = ',num2str(Kp(i))];
            end
            if FIGURA_RAMPA ~= 0
            figure(3 + (m-1)*graphics + (n-1)*length(taud)*graphics);
            title(['Error a Función Rampa - \tau_D = ', num2str(taud(m)),' y \tau_I = ',num2str(taui(n))]);
            hold on;
            plotHandlese2(i) = plot(t,e1);
            plotLabelse2{i} = ['Kp = ',num2str(Kp(i))];
            end
            if FIGURA_PARABOLA ~= 0
            figure(4 + (m-1)*graphics + (n-1)*length(taud)*graphics);
            title(['Error a Función Parábola - \tau_D = ', num2str(taud(m)),' y \tau_I = ',num2str(taui(n))]);
            hold on;
            plotHandlese3(i) = plot(t*2,e2);
            plotLabelse3{i} = ['Kp = ',num2str(Kp(i))];
            end
        end
        if FIGURA_OUT ~= 0
        legend(plotHandlesy,plotLabelsy);
        end
        if FIGURA_ESCALON ~= 0
        legend(plotHandlese1,plotLabelse1);
        end
        if FIGURA_RAMPA ~= 0
        legend(plotHandlese2,plotLabelse2);
        end
        if FIGURA_PARABOLA ~= 0
        legend(plotHandlese3,plotLabelse3);
        end  
    end
end
hold off
%% PI-D Régimen Permanente
close all
clear
syms s t
X1 = 1/s;
X2 = 1/s^2;
X3 = 1/s^3;
syms K p Kp taud taui

HeNum = (s^3 + s^2*(p+K*Kp*taud));
HeDen =s^2 *(s+p) + K*Kp*taud * (s^2 + s/taud+ 1/(taud*taui));
He = HeNum/HeDen;

E1 = He* X1;
E2 = He* X2;
E3 = He* X3;

e0= limit(s*E1,s,0); %#OK
e1= limit(s*E2,s,0); %#OK
e2= limit(s*E3,s,0);%#OK
%% PID-D
close all
clear
K = 1166.89;
p = 68;
taud2 = [-0.01,-0.05,-0.1,-1];%Debe ser negativo!
taud1 = [0.01,1];
taui = [1.5,4];
% SOLO PINTA LAS FIGURAS DISTINTAS DE 0
FIGURA_OUT = 0;
FIGURA_ESCALON = 0;
FIGURA_RAMPA = 0;
FIGURA_PARABOLA = 2;

t=0:0.01:40;
u1=t;
u2=t.^2;
graphics = 4;
scaler = 1.6;     % Para reducir el tiempo de la señal
Kp = zeros(1,length(taud2));

plotHandlesy = zeros(1,length(taud2));
plotLabelsy = cell(1,length(taud2));
plotHandlese1 = zeros(1,length(taud2));
plotLabelse1 = cell(1,length(taud2));
plotHandlese2 = zeros(1,length(taud2));
plotLabelse2 = cell(1,length(taud2));
plotHandlese3 = zeros(1,length(taud2));
plotLabelse3 = cell(1,length(taud2));

for n=1:length(taui)
    for m1=1:length(taud1)
        for m2=1:length(taud2)
            Kp(m2) = round(-p*taud2(m2)/K,3);
            Hnum = [(Kp(m2)*K*taud1(m1))   Kp(m2)*K  (Kp(m2)*K/taui(n))];
            Hden = [ 1  (Kp(m2)*K) K*Kp(m2) K*Kp(m2)/taui(n)];
            HeNum = [1 0 0 0];
            HeDen = Hden;
            y = step(Hnum,Hden,t/scaler);
            e0 = step(HeNum,HeDen,t);
            e1 = lsim(HeNum,HeDen,u1,t);
            e2 = lsim(HeNum,HeDen,u2,t);
                
            if FIGURA_OUT ~= 0
            figure(1 + (n-1)*length(taud1)*graphics + (m1-1)*graphics);
            hold on;
            title(['Output - \tau_{D1} = ', num2str(taud1(m1)),' y \tau_I = ',num2str(taui(n))]);
            plotHandlesy(m2) = plot(t/scaler,y);
            plotLabelsy{m2} = ['\tau_{D2} = ',num2str(taud2(m2)),' y Kp = ',num2str(Kp(m2))];
            end
            if FIGURA_ESCALON ~= 0
            figure(2 + (n-1)*length(taud1)*graphics + (m1-1)*graphics);
            title(['Error a Función Escalón - \tau_{D1} = ', num2str(taud1(m1)),' y \tau_I = ',num2str(taui(n))]);
            hold on;
            plotHandlese1(m2) = plot(t,e0);
            plotLabelse1{m2} = ['\tau_{D2} = ',num2str(taud2(m2)),' y Kp = ',num2str(Kp(m2))];
            end
            if FIGURA_RAMPA ~= 0
            figure(3 + (n-1)*length(taud1)*graphics + (m1-1)*graphics);
            title(['Error a Función Rampa - \tau_{D1} = ', num2str(taud1(m1)),' y \tau_I = ',num2str(taui(n))]);
            hold on;
            plotHandlese2(m2) = plot(t,e1);
            plotLabelse2{m2} = ['\tau_{D2} = ',num2str(taud2(m2)),' y Kp = ',num2str(Kp(m2))];
            end
            if FIGURA_PARABOLA ~= 0
            figure(4 + (n-1)*length(taud1)*graphics + (m1-1)*graphics);
            title(['Error a Función Parábola - \tau_{D1} =', num2str(taud1(m1)),' y \tau_I =',num2str(taui(n))]);
            hold on;
            plotHandlese3(m2) = plot(t,e2);
            plotLabelse3{m2} = ['\tau_{D2} = ',num2str(taud2(m2)),' y Kp = ',num2str(Kp(m2))];
            end     
        end
        if FIGURA_OUT ~= 0
        legend(plotHandlesy,plotLabelsy);
        end
        if FIGURA_ESCALON ~= 0
        legend(plotHandlese1,plotLabelse1);
        end
        if FIGURA_RAMPA ~= 0
        legend(plotHandlese2,plotLabelse2);
        end
        if FIGURA_PARABOLA ~= 0
        legend(plotHandlese3,plotLabelse3);
        end       
    end
end
hold off

%% PID-D Régimen Permanente
close all
clear
syms s t
X1 = 1/s;
X2 = 1/s^2;
X3 = 1/s^3;
syms K p Kp taud1 taud2 taui
taud = taud1+taud2;
HeNum = s^2*(s+p+K*Kp*taud2);
HeDen =s^2*(p+s) + K*Kp*taud*(s^2 + s/taud + 1/(taud*taui));
He = HeNum/HeDen;
E1 = He* X1;
E2 = He* X2;
E3 = He* X3;

e0= limit(s*E1,s,0); %#OK
e1= limit(s*E2,s,0); %#OK
e2= limit(s*E3,s,0);%#OK

%% D|PID
close all
clear
K = 1166.89;
p = 68;
taud1 = [0.01,0.5,2];
taud2 = [0.01,0.05,0.1,0.5,2]; %Debe ser negativo!
taui = [0.5,2];
% SOLO PINTA LAS FIGURAS DISTINTAS DE 0
FIGURA_OUT = 1;
FIGURA_ESCALON = 0;
FIGURA_RAMPA = 0;
FIGURA_PARABOLA = 0;

t=0:0.01:10;
u1=t;
u2=t.^2;
graphics = 4;
scaler = 8;     % Para reducir el tiempo de la señal

Kp = zeros(1,length(taud2));
plotHandlesy = zeros(1,length(taud2));
plotLabelsy = cell(1,length(taud2));
plotHandlese1 = zeros(1,length(taud2));
plotLabelse1 = cell(1,length(taud2));
plotHandlese2 = zeros(1,length(taud2));
plotLabelse2 = cell(1,length(taud2));
plotHandlese3 = zeros(1,length(taud2));
plotLabelse3 = cell(1,length(taud2));

for n=1:length(taui)
    for m1=1:length(taud1)
        for m2=1:length(taud2)
            Kp(m2) = round(p*taud2(m2)/K,3);
            Hnum = [(p+(Kp(m2)*K*taud1(m1)))   Kp(m2)*K  (Kp(m2)*K/taui(n))];
            Hden = [ 1  (p+(Kp(m2)*K*taud1(m1))) K*Kp(m2)  K*Kp(m2)/taui(n)];
            HeNum = [1 0 0 0];
            HeDen = Hden;
            y = step(Hnum,Hden,t/scaler);
            e0 = step(HeNum,HeDen,t);
            e1 = lsim(HeNum,HeDen,u1,t);
            e2 = lsim(HeNum,HeDen,u2,t);
                
            if FIGURA_OUT ~= 0
            figure(1 + (n-1)*length(taud1)*graphics + (m1-1)*graphics);
            hold on;
            title(['Output - \tau_{D1} = ', num2str(taud1(m1)),' y \tau_I = ',num2str(taui(n))]);
            plotHandlesy(m2) = plot(t/scaler,y);
            plotLabelsy{m2} = ['\tau_{D2} = ',num2str(taud2(m2)),' y Kp = ',num2str(Kp(m2))];
            end
            if FIGURA_ESCALON ~= 0
            figure(2 + (n-1)*length(taud1)*graphics + (m1-1)*graphics);
            title(['Error a Función Escalón - \tau_{D1} = ', num2str(taud1(m1)),' y \tau_I = ',num2str(taui(n))]);
            hold on;
            plotHandlese1(m2) = plot(t,e0);
            plotLabelse1{m2} = ['\tau_{D2} = ',num2str(taud2(m2)),' y Kp = ',num2str(Kp(m2))];
            end
            if FIGURA_RAMPA ~= 0
            figure(3 + (n-1)*length(taud1)*graphics + (m1-1)*graphics);
            title(['Error a Función Rampa - \tau_{D1} = ', num2str(taud1(m1)),' y \tau_I = ',num2str(taui(n))]);
            hold on;
            plotHandlese2(m2) = plot(t,e1);
            plotLabelse2{m2} = ['\tau_{D2} = ',num2str(taud2(m2)),' y Kp = ',num2str(Kp(m2))];
            end
            if FIGURA_PARABOLA ~= 0
            figure(4 + (n-1)*length(taud1)*graphics + (m1-1)*graphics);
            title(['Error a Función Parábola - \tau_{D1} =', num2str(taud1(m1)),' y \tau_I =',num2str(taui(n))]);
            hold on;
            plotHandlese3(m2) = plot(t,e2);
            plotLabelse3{m2} = ['\tau_{D2} = ',num2str(taud2(m2)),' y Kp = ',num2str(Kp(m2))];
            end     
        end
        if FIGURA_OUT ~= 0
        legend(plotHandlesy,plotLabelsy);
        end
        if FIGURA_ESCALON ~= 0
        legend(plotHandlese1,plotLabelse1);
        end
        if FIGURA_RAMPA ~= 0
        legend(plotHandlese2,plotLabelse2);
        end
        if FIGURA_PARABOLA ~= 0
        legend(plotHandlese3,plotLabelse3);
        end       
    end
end
hold off

%% D|PID Regimen Permanente
close all
clear
syms s t
X1 = 1/s;
X2 = 1/s^2;
X3 = 1/s^3;
syms K p Kp taud1 taud2 taui
taud = taud1 + taud2;
HeNum = s^2*(s+p-K*Kp*taud2);
HeDen =s^2 *(s+p) + K*Kp*taud*(s^2 + s/taud + 1/(taud*taui));

He = HeNum/HeDen;
E1 = He* X1;
E2 = He* X2;
E3 = He* X3;

e0= limit(s*E1,s,0); %#OK
e1= limit(s*E2,s,0); %#OK
e2= limit(s*E3,s,0);%#OK