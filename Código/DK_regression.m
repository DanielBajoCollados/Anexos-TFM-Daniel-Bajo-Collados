clear;
X=table2array(readtable('MotorsPosition_No0_Net.csv'));
Y=table2array(readtable('EndPosition_No0_Net.csv'));

cv=cvpartition(size(Y,1), 'HoldOut', 0.3);
X_train = X(~cv.test, :);
Y_train = Y(~cv.test, :);
X_test = X(cv.test, :);
Y_test = Y(cv.test, :);

% Scale data
mean_x = mean(X_train);
std_x = std(X_train);

mean_y = mean(Y_train);
std_y = std(Y_train);

X_train_scale = (X_train - mean_x) ./ std_x;
X_test_scale = (X_test - mean_x) ./ std_x;

Y_train_scale = (Y_train - mean_y) ./ std_y;
Y_test_scale = (Y_test - mean_y) ./ std_y;

%% Train model

elm = extreme_learning_machine_regressor(X_train_scale, Y_train_scale, 'hidden', 150, 'activation', 'fastsigmoid'); % Train ELM

for i=(1:size(elm.Win, 1))
    b((i-1)*size(elm.Win, 2)+1:i*size(elm.Win, 2))=elm.Win(i, :);
end
for i=(1:size(elm.Wout, 1))
    beta((i-1)*size(elm.Wout, 2)+1:i*size(elm.Wout, 2))=elm.Wout(i, :);
end
writematrix(b, 'DKbarray.csv');
writematrix(beta, 'DKbetaarray.csv');

%%Testing of model
y = elm.predict(X_test_scale); % Predict

% Rescale target
y = y .* std_y + mean_y;

TMeasured=Y_test;
TPredicted=y;

TErr=TMeasured-TPredicted;

fprintf("----------------------------\n");
fprintf("Traslation Errors\n")
fprintf("%f %f %f %f %f %f %f\n", mean(TErr));
fprintf("%f %f %f %f %f %f %f\n", mean(TErr.^2));
fprintf("%f %f %f %f %f %f %f\n", mean(TErr.^2).^0.5);
fprintf("Max Errors\n");
fprintf("%f %f %f %f %f %f %f\n", max(abs(TErr)));
fprintf("Min Errors\n");
fprintf("%f %f %f %f %f %f %f\n", min(abs(TErr)));
fprintf("----------------------------\n");