clear
hold on

%% D|PID
close all
clear
K = 2652.28;
p = 68;
taud1 = [0.5,2];
taud2 = [0.1,0.5,2,]; %Debe ser negativo!
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
