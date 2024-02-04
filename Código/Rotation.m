clear;
x=[1 0 0];
y=[0 1 0];
EndPosition=table2array(readtable('ETest.csv'));
EndPosition=[EndPosition(3,:);EndPosition(5,:);EndPosition(7,:);EndPosition(9,:);EndPosition(11,:);EndPosition(13,:);EndPosition(15,:)];
quat = EndPosition(:,1:4);
quat=quaternion(quat);
Rotations = [rotateframe(quat,x);rotateframe(quat,y)];
starts=[EndPosition(:,5:7)*1000;EndPosition(:,5:7)*1000];
MotorsPosition=table2array(readtable('MTest.csv'));
MotorsPosition=[MotorsPosition(3,:);MotorsPosition(5,:);MotorsPosition(7,:);MotorsPosition(9,:);MotorsPosition(11,:);MotorsPosition(13,:);MotorsPosition(15,:)];
EndPredict=DK_SLFN(MotorsPosition')';
quatP=EndPredict(:,1:4);
quatP=quaternion(quatP);
RotationsP=[rotateframe(quatP,x);rotateframe(quatP,y)];
startsP=[EndPredict(:,5:7)*1000;EndPredict(:,5:7)*1000];

for i=(0:1) 
    RotsRed(i+1:2:i+3,:)=[Rotations((i*size(RotationsP,1)/2)+1,:);RotationsP((i*size(RotationsP,1)/2)+1,:)];
    RotsGreen(i+1:2:i+3,:)=[Rotations((i*size(RotationsP,1)/2)+2,:);RotationsP((i*size(RotationsP,1)/2)+2,:)];
    RotsBlue(i+1:2:i+3,:)=[Rotations((i*size(RotationsP,1)/2)+3,:);RotationsP((i*size(RotationsP,1)/2)+3,:)];
    RotsCyan(i+1:2:i+3,:)=[Rotations((i*size(RotationsP,1)/2)+4,:);RotationsP((i*size(RotationsP,1)/2)+4,:)];
    RotsMagenta(i+1:2:i+3,:)=[Rotations((i*size(RotationsP,1)/2)+5,:);RotationsP((i*size(RotationsP,1)/2)+5,:)];
    RotsYellow(i+1:2:i+3,:)=[Rotations((i*size(RotationsP,1)/2)+6,:);RotationsP((i*size(RotationsP,1)/2)+6,:)];
    RotsBlack(i+1:2:i+3,:)=[Rotations((i*size(RotationsP,1)/2)+7,:);RotationsP((i*size(RotationsP,1)/2)+7,:)];
    StartsRed(i+1:2:i+3,:)=[starts((i*size(startsP,1)/2)+1,:);startsP((i*size(startsP,1)/2)+1,:)];
    StartsGreen(i+1:2:i+3,:)=[starts((i*size(startsP,1)/2)+2,:);startsP((i*size(startsP,1)/2)+2,:)];
    StartsBlue(i+1:2:i+3,:)=[starts((i*size(startsP,1)/2)+3,:);startsP((i*size(startsP,1)/2)+3,:)];
    StartsCyan(i+1:2:i+3,:)=[starts((i*size(startsP,1)/2)+4,:);startsP((i*size(startsP,1)/2)+4,:)];
    StartsMagenta(i+1:2:i+3,:)=[starts((i*size(startsP,1)/2)+5,:);startsP((i*size(startsP,1)/2)+5,:)];
    StartsYellow(i+1:2:i+3,:)=[starts((i*size(startsP,1)/2)+6,:);startsP((i*size(startsP,1)/2)+6,:)];
    StartsBlack(i+1:2:i+3,:)=[starts((i*size(startsP,1)/2)+7,:);startsP((i*size(startsP,1)/2)+7,:)];
end

% Rotations = rotateframe(quat,x);
% starts=newEnd(:,5:7)*1000;
quiver3(StartsRed(1:2,1), StartsRed(1:2,2), StartsRed(1:2,3), RotsRed(1:2,1), RotsRed(1:2,2), RotsRed(1:2,2), 10, ['-', '+' , 'r']);
hold on
quiver3(StartsGreen(1:2,1), StartsGreen(1:2,2), StartsGreen(1:2,3), RotsGreen(1:2,1), RotsGreen(1:2,2), RotsGreen(1:2,2), 10, ['-', '+' , 'g']);
hold on
quiver3(StartsBlue(1:2,1), StartsBlue(1:2,2), StartsBlue(1:2,3), RotsBlue(1:2,1), RotsBlue(1:2,2), RotsBlue(1:2,2), 10, ['-', '+' , 'b']);
hold on
quiver3(StartsCyan(1:2,1), StartsCyan(1:2,2), StartsCyan(1:2,3), RotsCyan(1:2,1), RotsCyan(1:2,2), RotsCyan(1:2,2), 10, ['-', '+' , 'c']);
hold on
quiver3(StartsMagenta(1:2,1), StartsMagenta(1:2,2), StartsMagenta(1:2,3), RotsMagenta(1:2,1), RotsMagenta(1:2,2), RotsMagenta(1:2,2), 10, ['-', '+' , 'm']);
hold on
quiver3(StartsYellow(1:2,1), StartsYellow(1:2,2), StartsYellow(1:2,3), RotsYellow(1:2,1), RotsYellow(1:2,2), RotsYellow(1:2,2), 10, ['-', '+' , 'y']);
hold on
quiver3(StartsBlack(1:2,1), StartsBlack(1:2,2), StartsBlack(1:2,3), RotsBlack(1:2,1), RotsBlack(1:2,2), RotsBlack(1:2,2), 10, ['-', '+' , 'k']);
hold on
quiver3(StartsRed(3:4,1), StartsRed(3:4,2), StartsRed(3:4,3), RotsRed(3:4,1), RotsRed(3:4,2), RotsRed(3:4,2), 10, ['-', '^' , 'r']);
hold on
quiver3(StartsGreen(3:4,1), StartsGreen(3:4,2), StartsGreen(3:4,3), RotsGreen(3:4,1), RotsGreen(3:4,2), RotsGreen(3:4,2), 10, ['-', '^' , 'g']);
hold on
quiver3(StartsBlue(3:4,1), StartsBlue(3:4,2), StartsBlue(3:4,3), RotsBlue(3:4,1), RotsBlue(3:4,2), RotsBlue(3:4,2), 10, ['-', '^' , 'b']);
hold on
quiver3(StartsCyan(3:4,1), StartsCyan(3:4,2), StartsCyan(3:4,3), RotsCyan(3:4,1), RotsCyan(3:4,2), RotsCyan(3:4,2), 10, ['-', '^' , 'c']);
hold on
quiver3(StartsMagenta(3:4,1), StartsMagenta(3:4,2), StartsMagenta(3:4,3), RotsMagenta(3:4,1), RotsMagenta(3:4,2), RotsMagenta(3:4,2), 10, ['-', '^' , 'm']);
hold on
quiver3(StartsYellow(3:4,1), StartsYellow(3:4,2), StartsYellow(3:4,3), RotsYellow(3:4,1), RotsYellow(3:4,2), RotsYellow(3:4,2), 10, ['-', '^' , 'y']);
hold on
quiver3(StartsBlack(3:4,1), StartsBlack(3:4,2), StartsBlack(3:4,3), RotsBlack(3:4,1), RotsBlack(3:4,2), RotsBlack(3:4,2), 10, ['-', '^' , 'k']);
% axis equal

xlim([-20, 20]);
ylim([-70, 70]);
zlim([0, 140]);
xlabel('X[mm]', 'FontSize', 30)
ylabel('Y[mm]', 'FontSize', 30)
zlabel('Z[mm]', 'FontSize', 30)
pbaspect([1 1 1]);