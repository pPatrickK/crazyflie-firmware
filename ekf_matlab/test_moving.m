fname = 'moving7.mat';
load(fname);
process;

P = load('Pinit_imuvis.mat');
P = P.Pinit(1:15, 1:15);

% TEMP disabled for synthetic data
acc_imu = [acc_imu(2,:); -acc_imu(1,:); acc_imu(3,:)];
gyro_imu = [gyro_imu(2,:); -gyro_imu(1,:); gyro_imu(3,:)];

acc_unbias = [acc_unbias(2,:); -acc_unbias(1,:); acc_unbias(3,:)];
gyro_unbias = [gyro_unbias(2,:); -gyro_unbias(1,:); gyro_unbias(3,:)];

NPTS = 1000;
NPTS = size(pos_vicon, 2);
[p_ekf, v_ekf, a_ekf, q_ekf, bw_ekf, ba_ekf] = ...
    ekf_full(acc_unbias, gyro_unbias, pos_vicon, quat_vicon, dt);
if any(isnan(p_ekf(:))) || any(isnan(v_ekf(:))) || any(isnan(q_ekf(:)))
    error('NaN output');
end
[p_c, v_c, q_c, ~, ~, ~] = ...
	ekfmex(t', acc_unbias, gyro_unbias, pos_vicon, quat_vicon, P);

rmserr = @(a, b) sqrt(sum((a-b).^2, 1));
%CLOSE = 0.01;
%assert(~any(rmserr(p_ekf, p_c) > CLOSE));
%assert(~any(rmserr(v_ekf, v_c) > CLOSE));
%assert(~any(rmserr(q_ekf, q_c) > CLOSE));

clf; hold on;
p_ekf = p_ekf(:,1:NPTS);
pos_vicon = pos_vicon(:,1:NPTS);
plot3n(p_c);
%plot3n(p_ekf);
plot3n(pos_vicon);

axis equal;
p_err = p_c - pos_vicon;
p_rmse = sqrt(sum(p_err .^ 2, 1));
[max_rmse, iworst] = max(p_rmse);
fprintf('\nmax rmse: %f\n', max_rmse);
if ~exist('best_rmse', 'var') || max_rmse < best_rmse
    best_rmse = max_rmse;
    fprintf('new high score!\n');
end
plot3(p_ekf(1,iworst), p_ekf(2,iworst), p_ekf(3,iworst), 'b*');
plot3(pos_vicon(1,iworst), pos_vicon(2,iworst), pos_vicon(3,iworst), 'r*');
ba_final = ba_ekf(:,end)
