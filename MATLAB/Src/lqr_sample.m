%% 倒立振子のLQR制御設計
% 状態方程式: dx/dt = Ax + Bp
% 状態変数: [x, x_dot, theta, theta_dot]'
% 制御入力: p (モータへの入力)

clear; clc; close all;

%% パラメータ設定
% 物理パラメータ
M = 0.698;        % 台車質量 [kg]
m = 0.0303;        % 振子質量 [kg]
l = 0.30 / 2;        % 振子長さ [m]
g = 9.81;       % 重力加速度 [m/s^2]

v_base = 12; % 定格電圧
v_ref = 8; % 印加電圧
v_ratio = v_ref / v_base; % 定格電圧と印加電圧の比率

% モータ・機構パラメータ
r_omega = 0.0255; % 車輪半径 [m]
omega_max = 387.46 * v_ratio; % 最大回転数 [rad/s]
u_max = 0.0078 * v_ratio;     % 最大トルク [N・m]
G = 6.667;         % ギア比

%% 状態空間行列の計算
% A行列の要素計算
a22 = -4/(4*M + m) * G^2 * u_max / (r_omega^2 * omega_max);
a23 = -3*m*g/(4*M + m);
a42 = -3/(l*(4*M + m)) * G^2 * u_max / (r_omega^2 * omega_max);
a43 = -3*(M + m)*g / (l*(4*M + m));

% A行列
A = [0,   1,   0,   0;
     0,  a22, a23,  0;
     0,   0,   0,   1;
     0,  a42, a43,  0];

% B行列の要素計算
b2 = 4/(4*M + m) * G * u_max / (r_omega * omega_max);
b4 = 3/(l*(4*M + m)) * G * u_max / (r_omega * omega_max);

% B行列
B = [0; b2; 0; b4];

% C行列（全状態観測可能と仮定）
C = eye(4);
D = 0;

% 10, 1, 50000, 10000

%% 重み行列の設定
% Q行列（状態の重み）
Q = diag([1, 100, 1000, 5000]); % [x位置, x速度, 角度, 角速度]の重み

% R行列（制御入力の重み）
R = 1;

%% LQR制御器設計
[K, S, P] = lqr(A, B, Q, R);

%% 結果表示
fprintf('=== 倒立振子LQR制御設計結果 ===\n');
fprintf('パラメータ:\n');
fprintf('  台車質量 M = %.2f kg\n', M);
fprintf('  振子質量 m = %.2f kg\n', m);
fprintf('  振子長さ l = %.2f m\n', l);
fprintf('  ギア比 G = %.1f\n', G);
fprintf('\n');

fprintf('状態フィードバックゲイン K:\n');
fprintf('  K = [%.4f, %.4f, %.4f, %.4f]\n', K);
fprintf('\n');

fprintf('制御則: u = -K * x\n');
fprintf('v_ref = -%.4f*x - %.4f*dx - %.4f*theta - %.4f*dtheta;\n', K);
fprintf('\n');

%% 閉ループ系の安定性確認
A_cl = A - B*K;
poles = eig(A_cl);

fprintf('閉ループ系の極:\n');
for i = 1:length(poles)
    if imag(poles(i)) == 0
        fprintf('  極%d: %.4f\n', i, real(poles(i)));
    else
        fprintf('  極%d: %.4f ± %.4fj\n', i, real(poles(i)), abs(imag(poles(i))));
    end
end

% 安定性判定
if all(real(poles) < 0)
    fprintf('-> 閉ループ系は安定です\n');
else
    fprintf('-> 警告: 閉ループ系が不安定です\n');
end

%% 状態空間モデルの作成
sys_ol = ss(A, B, C, D);  % 開ループ系
sys_cl = ss(A_cl, B, C, D); % 閉ループ系

%% シミュレーション
t = 0:0.01:5; % 時間ベクトル

% 初期条件（振子が少し傾いた状態）
x0 = [0; 0; 0.1; 0]; % [x, x_dot, theta, theta_dot]

% 閉ループ系の応答
[y, t_sim] = initial(sys_cl, x0, t);

%% 結果のプロット
figure('Position', [100, 100, 1200, 800]);

% 位置応答
subplot(2,2,1);
plot(t_sim, y(:,1), 'b-', 'LineWidth', 2);
xlabel('時間 [s]');
ylabel('台車位置 x [m]');
title('台車位置応答');
grid on;

% 角度応答
subplot(2,2,2);
plot(t_sim, y(:,3)*180/pi, 'r-', 'LineWidth', 2);
xlabel('時間 [s]');
ylabel('振子角度 θ [deg]');
title('振子角度応答');
grid on;

% 制御入力
u_sim = -K * y';
subplot(2,2,3);
plot(t_sim, u_sim, 'g-', 'LineWidth', 2);
xlabel('時間 [s]');
ylabel('制御入力 p');
title('制御入力');
grid on;

% 相平面図（角度 vs 角速度）
subplot(2,2,4);
plot(y(:,3)*180/pi, y(:,4)*180/pi, 'k-', 'LineWidth', 2);
xlabel('角度 θ [deg]');
ylabel('角速度 θ̇ [deg/s]');
title('相平面図（角度-角速度）');
grid on;

sgtitle('倒立振子LQR制御シミュレーション結果');

%% ボード線図（開ループ特性）
figure;
bode(sys_ol);
title('開ループ系のボード線図');
grid on;

%% ステップ応答比較
figure;
step(sys_cl, 10);
title('閉ループ系のステップ応答');
grid on;

fprintf('\n=== シミュレーション完了 ===\n');
fprintf('初期角度: %.1f度から制御開始\n', x0(3)*180/pi);
fprintf('最終角度: %.4f度\n', y(end,3)*180/pi);
fprintf('整定時間: 約%.2f秒\n', t_sim(end));