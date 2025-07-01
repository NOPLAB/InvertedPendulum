% 電流フィードバックとタイヤ速度フィードバックを含む制御システム

clear; clc; close all;

load("estimate_parameters/parameters.mat")

%% パラメータの設定
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

%% 拡張状態空間モデルの構築
% 状態変数: X = [x; x_dot; phi; phi_dot; i; x_int; i_int]
% x_int: 位置の積分（定常偏差除去用）
% i_int: 電流の積分（定常偏差除去用）

fprintf('=== 拡張状態空間モデルの構築 ===\n');

% 基本の状態空間行列を取得
[A_linear, B_linear] = linearized_matrices(params);

% 拡張状態空間行列（積分器を追加）
n = size(A_linear, 1);  % 元の状態数 (5)
A_ext = zeros(n+2, n+2);
A_ext(1:n, 1:n) = A_linear;
A_ext(n+1, 1) = 1;      % dx_int/dt = x
A_ext(n+2, 5) = 1;      % di_int/dt = i

B_ext = zeros(n+2, 1);
B_ext(1:n, 1) = B_linear;

% 出力行列（測定可能な状態）
% y = [x; x_dot; phi; phi_dot; i; x_int; i_int]
C_ext = eye(7);

fprintf('拡張システムの次元: %d\n', size(A_ext, 1));

%% 階層型制御器の設計
% Level 1: 電流制御（最内側ループ）
% Level 2: 速度制御（中間ループ）
% Level 3: 位置・角度制御（最外側ループ）

%% 1. 電流制御器（PIコントローラ）
fprintf('\n=== Level 1: 電流制御器の設計 ===\n');

% 電流制御の時定数目標
tau_i_desired = params(14)/params(13) * 100;  % La/Ra
fprintf('目標電流応答時定数: %.4f s\n', tau_i_desired);

% PI制御器のパラメータ（極配置法）
Kp_i = params(14) / tau_i_desired - params(13);  % 比例ゲイン
Ki_i = params(13) / tau_i_desired;               % 積分ゲイン

fprintf('電流PI制御器:\n');
fprintf('  Kp_i = %.4f\n', Kp_i);
fprintf('  Ki_i = %.4f\n', Ki_i);

%% 2. 速度制御器（PIコントローラ）
fprintf('\n=== Level 2: 速度制御器の設計 ===\n');

% タイヤ角速度の計算
% omega_wheel = x_dot / r
% モータ角速度: omega_motor = G * omega_wheel = G * x_dot / r

% 速度制御の時定数目標
tau_v_desired = tau_i_desired * 1000;
fprintf('目標速度応答時定数: %.4f s\n', tau_v_desired);

% 機械系の簡略化モデルから速度制御ゲインを設計
M_eq = params(1) + 2*params(6)/params(5)^2 + params(2);  % 等価質量
Kp_v = M_eq / tau_v_desired - params(9);  % 比例ゲイン（摩擦補償含む）
Ki_v = params(9) / tau_v_desired;          % 積分ゲイン

fprintf('速度PI制御器:\n');
fprintf('  Kp_v = %.4f\n', Kp_v);
fprintf('  Ki_v = %.4f\n', Ki_v);

%% 3. 位置・角度制御器（状態フィードバック）
fprintf('\n=== Level 3: 位置・角度制御器の設計 ===\n');

% 台車・振子システムの状態空間行列
[A_cart, B_cart, C_cart] = cart_pendulum_system(params);

% LQR設計
Q_lqr = diag([100, 10, 1000, 10]);  % [x, x_dot, phi, phi_dot]
R_lqr = 1;
[K_lqr, S_lqr, E_lqr] = lqr(A_cart, B_cart, Q_lqr, R_lqr);

fprintf('LQRゲイン:\n');
fprintf('  K = [%.4f, %.4f, %.4f, %.4f]\n', K_lqr);



%% シミュレーション関数
function [t, X, U, Refs] = simulate_feedback_system(controller, x0, x_ref, tf)
    % フィードバック制御システムのシミュレーション
    
    dt = controller.dt;
    t = 0:dt:tf;
    n = length(t);
    
    % 状態とコマンドの記録用配列
    X = zeros(n, 5);      % [x, x_dot, phi, phi_dot, i]
    U = zeros(n, 1);      % 電圧指令
    Refs = zeros(n, 3);   % [F_ref, v_ref, i_ref]
    
    % 初期状態
    X(1, :) = x0;
    
    % シミュレーションループ
    for k = 1:n-1
        % 現在の状態
        x_current = X(k, :)';
        
        % センサノイズの追加（実際的なシミュレーション）
        x_meas = x_current;
        x_meas(1) = x_meas(1) + 0.0001 * randn();  % 位置ノイズ ±0.1mm
        x_meas(2) = x_meas(2) + 0.001 * randn();   % 速度ノイズ ±1mm/s
        x_meas(3) = x_meas(3) + 0.0001 * randn();  % 角度ノイズ ±0.01deg
        x_meas(4) = x_meas(4) + 0.001 * randn();   % 角速度ノイズ
        x_meas(5) = x_meas(5) + 0.01 * randn();    % 電流ノイズ ±10mA
        
        % 制御計算
        [V, i_ref, v_ref, F_ref] = controller.compute_control(x_meas, x_ref);
        
        % 記録
        U(k) = V;
        Refs(k, :) = [F_ref, v_ref, i_ref];
        
        % 非線形ダイナミクスの更新（オイラー法）
        dX = nonlinear_dynamics(x_current, V, controller.params);
        X(k+1, :) = x_current + dt * dX;
    end
end

%% メインシミュレーション
fprintf('\n=== シミュレーション実行 ===\n');

% コントローラの作成
controller = sample2_class(params, Kp_i, Ki_i, Kp_v, Ki_v, K_lqr);

% 初期条件と目標値
x0 = [0; 0; 0.05; 0; 0];    % 初期角度0.1rad
x_ref = [0; 0; 0; 0];     % 目標位置0.5m、振子直立

% シミュレーション実行
tf = 5;  % シミュレーション時間
[t, X, U, Refs] = simulate_feedback_system(controller, x0, x_ref, tf);

%% 結果のプロット
figure('Position', [100, 100, 1200, 900]);

% 状態変数のプロット
subplot(3,3,1);
plot(t, X(:,1), 'b-', 'LineWidth', 2);
hold on;
plot(t, x_ref(1)*ones(size(t)), 'r--', 'LineWidth', 1);
xlabel('時間 [s]');
ylabel('位置 x [m]');
title('台車位置');
grid on;
legend('実際', '目標');

subplot(3,3,2);
plot(t, X(:,2), 'b-', 'LineWidth', 2);
hold on;
plot(t, Refs(:,2), 'g--', 'LineWidth', 1);
xlabel('時間 [s]');
ylabel('速度 [m/s]');
title('台車速度');
grid on;
legend('実際', '速度指令');

subplot(3,3,3);
plot(t, X(:,3)*180/pi, 'b-', 'LineWidth', 2);
hold on;
plot(t, zeros(size(t)), 'r--', 'LineWidth', 1);
xlabel('時間 [s]');
ylabel('角度 [deg]');
title('振子角度');
grid on;

subplot(3,3,4);
plot(t, X(:,4)*180/pi, 'b-', 'LineWidth', 2);
xlabel('時間 [s]');
ylabel('角速度 [deg/s]');
title('振子角速度');
grid on;

subplot(3,3,5);
plot(t, X(:,5), 'b-', 'LineWidth', 2);
hold on;
plot(t, Refs(:,3), 'g--', 'LineWidth', 1);
xlabel('時間 [s]');
ylabel('電流 [A]');
title('モータ電流');
grid on;
legend('実際', '電流指令');

subplot(3,3,6);
plot(t, U, 'b-', 'LineWidth', 2);
xlabel('時間 [s]');
ylabel('電圧 [V]');
title('印加電圧');
grid on;

% 制御性能の解析
subplot(3,3,7);
plot(t, Refs(:,1), 'b-', 'LineWidth', 2);
xlabel('時間 [s]');
ylabel('推力 [N]');
title('推力指令');
grid on;

% エラーのプロット
subplot(3,3,8);
e_pos = X(:,1) - x_ref(1);
e_angle = X(:,3);
plot(t, e_pos, 'b-', 'LineWidth', 2);
hold on;
plot(t, e_angle*10, 'r-', 'LineWidth', 2);  % 角度誤差を10倍表示
xlabel('時間 [s]');
ylabel('誤差');
title('追従誤差');
legend('位置誤差 [m]', '角度誤差×10 [rad]');
grid on;

% 電力消費
subplot(3,3,9);
power = U .* X(:,5);
energy = cumtrapz(t, abs(power));
plot(t, energy, 'b-', 'LineWidth', 2);
xlabel('時間 [s]');
ylabel('エネルギー [J]');
title('累積消費エネルギー');
grid on;

sgtitle('電流・速度フィードバック制御システムの応答');

%% 制御性能の評価
fprintf('\n=== 制御性能評価 ===\n');
fprintf('最終位置誤差: %.4f mm\n', abs(X(end,1) - x_ref(1))*1000);
fprintf('最終角度誤差: %.4f deg\n', abs(X(end,3))*180/pi);
fprintf('最大電流: %.2f A\n', max(abs(X(:,5))));
fprintf('最大電圧: %.2f V\n', max(abs(U)));
fprintf('総消費エネルギー: %.2f J\n', energy(end));

% 整定時間の計算
idx_pos = find(abs(X(:,1) - x_ref(1)) > 0.02*abs(x_ref(1)), 1, 'last');
idx_angle = find(abs(X(:,3)) > 0.01, 1, 'last');
fprintf('位置整定時間（2%%）: %.2f s\n', t(idx_pos));
fprintf('角度整定時間（0.01rad）: %.2f s\n', t(idx_angle));

%% 周波数応答解析（オプション）
fprintf('\n=== 周波数応答解析 ===\n');

% 閉ループシステムの構築
% 簡略化のため、線形化モデルで解析
s = tf('s');

% 電流制御ループ
G_current = 1/(params(13) + params(14)*s);  % モータの電気系
C_current = Kp_i + Ki_i/s;                   % PI制御器
T_current = feedback(C_current * G_current, 1);

% 速度制御ループ（簡略化モデル）
G_velocity = params(11)*params(7)/(params(5)*M_eq*s);  % 力から速度
C_velocity = Kp_v + Ki_v/s;
T_velocity = feedback(C_velocity * T_current * G_velocity, 1);

figure('Position', [100, 100, 800, 600]);
subplot(2,1,1);
bode(T_current);
title('電流制御ループの周波数応答');
grid on;

subplot(2,1,2);
bode(T_velocity);
title('速度制御ループの周波数応答');
grid on;

fprintf('\n=== シミュレーション完了 ===\n');