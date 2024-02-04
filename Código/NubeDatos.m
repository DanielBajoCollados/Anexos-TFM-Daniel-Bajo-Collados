clear;
EndPosition=table2array(readtable('EndPosition.csv'));
EndPositionSort=sortrows(EndPosition, 7);
% EndPositionSort=sortrows([newEnd;EndPosition], 7);
% EndPositionSort=sortrows(y, 7);
X=EndPositionSort(:,5)*1000;
Y=EndPositionSort(:,6)*1000;
Z=EndPositionSort(:,7)*1000;
G=zeros(size(X,1),1);
B=(1/size(X,1):1/size(X,1):1)';
R=1-B;
Colors=[R G B];
scatter3(X, Y, Z, '.', 'CData', Colors);
xlim([-70, 70]);
ylim([-70, 70]);
zlim([0, 140]);
xlabel('X[mm]', 'FontSize', 30)
ylabel('Y[mm]', 'FontSize', 30)
zlabel('Z[mm]', 'FontSize', 30)
pbaspect([1 1 1]);