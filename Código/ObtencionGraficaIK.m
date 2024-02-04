clear;
CRReal=table2array(readtable('MTest_Net.csv'));
Test=table2array(readtable('ETest_Net.csv'));
CRPredecida=IK_SLFN(Test')';

for i=1:size(CRReal,2)
    switch i
        case 1
        c='r';
        case 2
        c='m';
        case 3
        c='g';
        case 4
        c='y';
        case 5
        c='b';
        case 6
        c='k';
    end
    plot(CRReal(:,i), strcat(c, 'x'), 'MarkerSize', 12);
    hold on
    plot(CRPredecida(:,i), strcat(c, 'o'), 'MarkerSize', 12);
    hold on
end
grid on;
ax = gca;
ax.GridColor='k';
set(gca, 'Color', 'w');
ylabel('Rack Extension [mm]','FontSize', 30);
%xlabel('Medición','FontSize', 30);
lgd = legend('Measured Extension for Rack 1','Calculated Extension for Rack 1','Measured Extension for Rack 2','Calculated Extension for Rack 2','Measured Extension for Rack 3','Calculated Extension for Rack 3','Measured Extension for Rack 4','Calculated Extension for Rack 4','Measured Extension for Rack 5','Calculated Extension for Rack 5','Measured Extension for Rack 6','Calculated Extension for Rack 6');
lgd.Location = 'southwest';
lgd.FontSize = 12;
lgd.NumColumns = 2;