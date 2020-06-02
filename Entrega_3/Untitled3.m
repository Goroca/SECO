%% P
K = 1;
p = 1;
Kp = [0.5,1,2,4,20];
t=0:0.01:100;
u=t;
u1=t.^2;
scaler = 5;    % Para que no salgan tantas muestras

plotHandlesy = zeros(1,length(Kp));
plotLabelsy = cell(1,length(Kp));

plotHandlese1 = zeros(1,length(Kp));
plotLabelse1 = cell(1,length(Kp));

plotHandlese2 = zeros(1,length(Kp));
plotLabelse2 = cell(1,length(Kp));

plotHandlese3 = zeros(1,length(Kp));
plotLabelse3 = cell(1,length(Kp));

for i=1:length(Kp)

    Hnum = Kp(i)*K;
    Hden = [1  p  Kp(i)*K];
    HeNum = [1 0 p];
    HeDen = Hden;
    
    y1 = step(Hnum,Hden,t/scaler);
    e1 = step(HeNum,HeDen,t/scaler);
    e2 =lsim(HeNum,HeDen,u,t);
    e3 =lsim(HeNum,HeDen,u1,t);
    
    figure(1);
    title("Output");
    hold on;
    plotHandlesy(i) = plot(t/scaler,y1);
    plotLabelsy{i} = ['Kp = ',num2str(Kp(i))];

    figure(2);
    title("Error a Función Escalón");
    hold on;
    plotHandlese1(i) = plot(t/scaler,e1);
    plotLabelse1{i} = ['Kp = ',num2str(Kp(i))];

    figure(3);
    title("Error a Función Rampa");
    hold on;
    plotHandlese2(i) = plot(t,e2);
    plotLabelse2{i} = ['Kp = ',num2str(Kp(i))];

    figure(4);
    title("Error a Función Parábola");
    hold on;
    plotHandlese3(i) = plot(t,e3);
    plotLabelse3{i} = ['Kp = ',num2str(Kp(i))];
end
legend(plotHandlesy,plotLabelsy);
legend(plotHandlese1,plotLabelse1);
legend(plotHandlese2,plotLabelse2);
legend(plotHandlese3,plotLabelse3);

hold off