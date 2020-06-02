clear
hold on

%% D|PID
close all
clear
K = 2652.28;
p = 64.986;
taud1 = 0.03;%[0.5,2];
taud2 = 10;%[0.1,0.5,2,]; %Debe ser >0!
taui = 20;%[0.5,2];
% SOLO PINTA LAS FIGURAS DISTINTAS DE 0
FIGURA_OUT = 1;
FIGURA_ERROR_ESCALON = 1;
FIGURA_ERROR_RAMPA = 1;
FIGURA_ERROR_PARABOLA = 1;
period = 0.01;
t=0:period:100;
u1=t;
u2=t.^2;
graphics = 4;
scaler = 80;     % Para reducir el tiempo de la señal

yPeriod = period/scaler;

Kp = zeros(1,length(taud2));
plotHandlesy = zeros(1,length(taud2));
plotLabelsy = cell(1,length(taud2));
plotHandlese1 = zeros(1,length(taud2));
plotLabelse1 = cell(1,length(taud2));
plotHandlese2 = zeros(1,length(taud2));
plotLabelse2 = cell(1,length(taud2));
plotHandlese3 = zeros(1,length(taud2));
plotLabelse3 = cell(1,length(taud2));


% EVALUACION
subida = 1;
tEstablecimiento = 0.4;
tEstablecimientoReal = 0;
establecimiento = 2/100;
sobreElongacion = 6/100;
sobreElongacionReal = 0;
tSubida = 0.25;
tSubidaReal=0;
for n=1:length(taui)
    for m1=1:length(taud1)
        for m2=1:length(taud2)
            Kp(m2) = round(p*taud2(m2)/K,3);
            Hnum = [(p+(Kp(m2)*K*taud1(m1)))   Kp(m2)*K  (Kp(m2)*K/taui(n))];
            Hden = [ 1  (p+(Kp(m2)*K*taud1(m1))) K*Kp(m2)  K*Kp(m2)/taui(n)];
            HeNum = [1 0 0 0];
            HeDen = Hden;
            y = step(Hnum,Hden,t/scaler);
            e0 = step(HeNum,HeDen,t/scaler);
            e1 = lsim(HeNum,HeDen,u1,t/scaler);
            e2 = lsim(HeNum,HeDen,u2,t);
            
            
            for a = 1:length(y)
                if y(a)  > subida && tSubidaReal == 0
                    tSubidaReal = (a-1)*yPeriod;
                    S=['Tiempo de Subida (',num2str(tSubidaReal),')- tau_D1 = ', num2str(taud1(m1)),' tau_D2 = ',num2str(taud2(m2)),' y tau_I = ',num2str(taui(n))];
                    disp(S)
                end
                if tSubidaReal ~=0
                    if abs(y(a)- 1) > sobreElongacionReal
                        sobreElongacionReal = abs(y(a)-1);
                    end
                end
                
                if tEstablecimientoReal == 0 && tSubidaReal ~=0
                    next = 0;
                    for b = a:length(y)
                        if abs(y(b)-1) > establecimiento
                            next=1;
                        end
                    end
                    if next==0
                        tEstablecimientoReal = a*yPeriod;
                    end
                end
            end
            S=['Tiempo de Establecimiento  (',num2str(tEstablecimientoReal),') - tau_D1 = ', num2str(taud1(m1)),' tau_D2 = ',num2str(taud2(m2)),' y tau_I = ',num2str(taui(n))];
            disp(S)           
            
            S=['SobreElongación Máxima  (',num2str(sobreElongacionReal*100),' %) - tau_D1 = ', num2str(taud1(m1)),' tau_D2 = ',num2str(taud2(m2)),' y tau_I = ',num2str(taui(n))];
            disp(S)

            if FIGURA_OUT ~= 0
                figure(1 + (n-1)*length(taud1)*graphics + (m1-1)*graphics);
                hold on;
                title(['Output - \tau_{D1} = ', num2str(taud1(m1)),' y \tau_I = ',num2str(taui(n))]);
                plotHandlesy(m2) = plot(t/scaler,y);
                plotLabelsy{m2} = ['\tau_{D2} = ',num2str(taud2(m2)),' y Kp = ',num2str(Kp(m2))];
            end
            if FIGURA_ERROR_ESCALON ~= 0
                figure(2 + (n-1)*length(taud1)*graphics + (m1-1)*graphics);
                title(['Error a Función Escalón - \tau_{D1} = ', num2str(taud1(m1)),' y \tau_I = ',num2str(taui(n))]);
                hold on;
                plotHandlese1(m2) = plot(t/scaler,e0);
                plotLabelse1{m2} = ['\tau_{D2} = ',num2str(taud2(m2)),' y Kp = ',num2str(Kp(m2))];
            end
            if FIGURA_ERROR_RAMPA ~= 0
                figure(3 + (n-1)*length(taud1)*graphics + (m1-1)*graphics);
                title(['Error a Función Rampa - \tau_{D1} = ', num2str(taud1(m1)),' y \tau_I = ',num2str(taui(n))]);
                hold on;
                plotHandlese2(m2) = plot(t/scaler,e1);
                plotLabelse2{m2} = ['\tau_{D2} = ',num2str(taud2(m2)),' y Kp = ',num2str(Kp(m2))];
            end
            if FIGURA_ERROR_PARABOLA ~= 0
                figure(4 + (n-1)*length(taud1)*graphics + (m1-1)*graphics);
                title(['Error a Función Parábola - \tau_{D1} =', num2str(taud1(m1)),' y \tau_I =',num2str(taui(n))]);
                hold on;
                plotHandlese3(m2) = plot(t,e2);
                plotLabelse3{m2} = ['\tau_{D2} = ',num2str(taud2(m2)),' y Kp = ',num2str(Kp(m2))];
            end
            tSubidaReal = 0;
            tEstablecimientoReal = 0;
            sobreElongacionReal = 0;
            
        end
        if FIGURA_OUT ~= 0
            legend(plotHandlesy,plotLabelsy);
        end
        if FIGURA_ERROR_ESCALON ~= 0
            legend(plotHandlese1,plotLabelse1);
        end
        if FIGURA_ERROR_RAMPA ~= 0
            legend(plotHandlese2,plotLabelse2);
        end
        if FIGURA_ERROR_PARABOLA ~= 0
            legend(plotHandlese3,plotLabelse3);
        end
    end
end
hold off
