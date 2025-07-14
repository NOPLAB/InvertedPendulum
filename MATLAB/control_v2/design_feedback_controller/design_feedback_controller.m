% LQRを用いた状態フィードバックゲインの導出
clear; close all; clc;

%% パラメータの読み込み
load("estimate_parameters\parameters.mat");

p_M = 0.57;
p_bx = 0.0;

params = [
    p_M;     % M: 台車質量 [kg]
    p_m;     % m: 振子質量 [kg]
    p_g;     % g: 重力加速度 [m/s^2]
    p_l;     % l: 振子長さ [m]
    p_r;     % r: タイヤ半径 [m]
    p_Iw;    % Iw: タイヤ慣性モーメント [kg*m^2]
    p_G;     % G: ギア比
    p_Jp;    % Jp: 振子慣性モーメント [kg*m^2]
    p_bx;    % bx: 台車の粘性摩擦係数 [N*s/m]
    p_btheta;% btheta: 振子の粘性摩擦係数 [N*m*s/rad]
    p_Kt;    % Kt: トルク定数 [N*m/A]
    p_Ke;    % Ke: 逆起電力定数 [V*s/rad]
    p_Ra;    % Ra: 電機子抵抗 [Ω]
    p_La     % La: 電機子インダクタンス [H]
];

%% 状態方程式の読み込み
[A, B, C] = cart_pendulum_system(params);
D = [0; 0];

% システムの可制御性・可観測性確認
Wc = ctrb(A, B);
Wo = obsv(A, C);
fprintf('可制御性: rank(Wc) = %d (期待値: %d)\n', rank(Wc), size(A,1));
fprintf('可観測性: rank(Wo) = %d (期待値: %d)\n', rank(Wo), size(A,1));

%% LQRによる状態フィードバックゲインの導出
fprintf('\n=== LQR設計 ===\n');

% Q行列（状態の重み）
Q = diag([100, 0, 50, 100]); % [x位置, x速度, 角度, 角速度]の重み

% R行列（制御入力の重み）
R = 10;

% LQR制御器設計
[K, S, P] = lqr(A, B, Q, R);

% 状態フィードバックゲインの表示
fprintf('LQRゲイン: K = [%.4f, %.4f, %.4f, %.4f]\n', K);

%% 閉ループ系の安定性確認
fprintf('\n--- LQR閉ループ系の安定性 ---\n');
A_cl = A - B*K;
poles_lqr = eig(A_cl);

fprintf('閉ループ系の極:\n');
for i = 1:length(poles_lqr)
    if abs(imag(poles_lqr(i))) < 1e-10
        fprintf('  極%d: %.6f\n', i, real(poles_lqr(i)));
    else
        fprintf('  極%d: %.6f ± %.6fj\n', i, real(poles_lqr(i)), abs(imag(poles_lqr(i))));
    end
end

% 安定性判定
if all(real(poles_lqr) < -1e-10)
    fprintf('-> 閉ループ系は安定です\n');
else
    fprintf('-> 警告: 閉ループ系が不安定です\n');
end
