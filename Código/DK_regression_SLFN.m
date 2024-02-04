x=table2array(readtable('MotorsPositionOld.csv'))';
t=table2array(readtable('EndPositionOld.csv'))';

trainFcn = 'trainlm';

last_total=1000;
for i=(50:50:200)
    hiddenLayerSize = i;
    net = fitnet(hiddenLayerSize,trainFcn);

     net.divideParam.trainRatio = 80/100;
    net.divideParam.valRatio = 10/100;
    net.divideParam.testRatio = 10/100;

    net.trainParam.epochs = 5000;

    net.performFcn='mse';
    net.trainParam.max_fail=10;

    [net,tr] = train(net,x,t);

    y = net(x);
    Error = gsubtract(t,y)';
    performance = perform(net,t,y);
    ECM=mean(Error.^2);
    Root=ECM.^0.5;
    total=sum(Root);

    fprintf("-------For %d neurons-------\n", i);
    fprintf("Errors:\n%f %f %f %f %f %f %f\n", Root);
    fprintf("Sum:%f\n", total);
    fprintf("Max Errors\n%f %f %f %f %f %f %f\n", max(abs(Error)));
    fprintf("Min Errors\n%f %f %f %f %f %f %f\n", min(abs(Error)));

    if(total<=last_total)
        last_total=total;
        BestNet=net;
    end
    
end
genFunction(BestNet, 'DK_No_g');
