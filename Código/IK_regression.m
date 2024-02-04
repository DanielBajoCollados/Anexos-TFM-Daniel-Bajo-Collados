clear;
X=table2array(readtable('EndPositionOld.csv'));
Y=table2array(readtable('MotorsPositionOld.csv'));

cv=cvpartition(size(Y,1), 'HoldOut', 0.2);
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
last_total=1000;
for i=(50:10:300)

    elm = extreme_learning_machine_regressor(X_train_scale, Y_train_scale, 'hidden', i, 'activation', 'fastsigmoid'); % Train ELM

    %%Testing of model
    y = elm.predict(X_test_scale); % Predict

    % Rescale target
    y = y .* std_y + mean_y;

    TMeasured=Y_test;
    TPredicted=y;

    TErr=TMeasured-TPredicted;

    fprintf("-------For %d neurons-------\n", i);
    fprintf("Traslation Errors\n")
    fprintf("%f %f %f %f %f %f\n", (mean(TErr.^2)).^0.5);
    fprintf("Max Errors\n");
    fprintf("%f %f %f %f %f %f\n", max(abs(TErr)));
    fprintf("Min Errors\n");
    fprintf("%f %f %f %f %f %f\n", min(abs(TErr)));

    m=sum((mean(TErr.^2)).^0.5);
    if m<last_total
        last_total=m;
        bestElm=elm;
    end
end