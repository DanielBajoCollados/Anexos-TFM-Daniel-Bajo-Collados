clear;
List = readtable('../Data File All.csv');

%%The matrix to save the values of all motors positions
MotorsTab = List(:, 13:18);
MotorsMat=table2array(MotorsTab);

%%The matrix to save the values of all quaternions and traslations

%%First, get all values of the transformation matrices
TTab = List(:,1:12);
TMat=table2array(TTab);

%%Transform all arrays storing the information into matrices
TMats = zeros(4, 4, size(TMat, 1));

for k=(1:size(TMat, 1))%%Loop for each matrix
    for i=(1:3)%%Loop for each row
        for j=(1:4)%%Loop for each column
            TMats(i, j, k)=TMat(k, (4*(i-1))+j);
        end
    end
end

for k=(1:size(TMat, 1))%%Loop for each matrix
    for j=(1:3)%%Loop for each column
        TMats(4, j, k)=0;
    end
    TMats(4, 4, k)=1;
end

%%Get all values on two disticnt matrices. One for the motors positions and
%%the other for the end of the arm positions

%%For the motors
%%Transform the integer to the length of the rack on mm
%%This is done by transforming the integer of the rotation into radians
%%(2pi rads/4095) and multiplying it by the radius (7 mm)
MotorsPosition = MotorsMat * 7 * 2*pi / 4095;

%%For the end of the arm
%%Transform all rotation matrices into quaternions and concatenate them
%%with al traslation vectors
RMats = zeros(3, 3, size(TMat, 1));
for k=(1:size(TMat, 1))%%Loop for each matrix
    RMats(1:3,1:3,k) = TMats(1:3,1:3,k);
end

QuatMats = zeros(size(TMat, 1), 4);
for k=(1:size(TMat, 1))%%Loop for each quaternion
    QuatMats(k, :)=rotm2quat(RMats(:, :, k));
end

EndPosition = zeros(size(TMat, 1), 7);
EndPosition(1:size(TMat, 1),1:4) = QuatMats;
for k=(1:size(TMat, 1))
    for i=(1:3)
        EndPosition(k,4+i) = TMats(i,4,k);
    end
end

%%Check for NaN or infinite values and remove if found
Test = [(~isfinite(EndPosition))|isnan(EndPosition), (~isfinite(MotorsPosition))|isnan(MotorsPosition)];
Check = any(Test);
for k=(1:13)
    EndPosition((Test(:,k)), :)=[];
    MotorsPosition((Test(:,k)), :)=[];
end

%%Orginize all data in four matrices. Two for input and output on the
%%forward kinematics, and the other two for inverse kinematics.

OutputDK = EndPosition;
OutputIK = MotorsPosition;
InputDK = zeros(size(MotorsPosition, 1), size(MotorsPosition, 2)+1+4);
InputIK = zeros(size(EndPosition, 1), size(EndPosition, 2)+1+4);
weight=161;%Weight of end of arm in grams
%In this case, all data is taken with the ROBOMINERS module sideways and
%with only the coupler attached at the end
InputDK(:,1:6)=MotorsPosition;
for k=(1:size(InputDK, 1))
    InputDK(k,7:10)=rotm2quat(rotvec2mat3d([pi/2, 0, 0]));
    InputDK(k, 11)=weight;
end

InputIK(:,1:7)=EndPosition;
for k=(1:size(InputDK, 1))
    InputIK(k,8:11)=rotm2quat(rotvec2mat3d([pi/2, 0, 0]));
    InputIK(k, 12)=weight;
end

EndPosition2 = EndPosition;
EndPosition2(:, 5:7)=EndPosition2(:, 5:7)*1000;

MotorsPosition2=MotorsPosition;
MotorsPosition2(:,2)=MotorsPosition2(:,2)-MotorsPosition2(:,1);
MotorsPosition2(:,4)=MotorsPosition2(:,4)-MotorsPosition2(:,3);
MotorsPosition2(:,6)=MotorsPosition2(:,6)-MotorsPosition2(:,5);
writematrix(MotorsPosition2, 'MotorsPosition_Net.csv');
writematrix(EndPosition2, 'EndPosition_Net.csv');

%%Save all lists
writematrix(MotorsPosition, 'MotorsPosition.csv');
writematrix(EndPosition, 'EndPosition.csv');