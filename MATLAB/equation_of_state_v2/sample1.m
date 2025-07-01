%% use_separated_systems.m
% 分離されたシステムの使用例

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

%% 1. 分離されたシステムの行列を取得
fprintf('=== システム行列の計算 ===\n');

% DCモータシステム
[A_motor, B_motor, C_motor, E_motor] = dc_motor_system(params);
fprintf('\nDCモータシステム:\n');
fprintf('A_motor = %.4f\n', A_motor);
fprintf('B_motor = %.4f\n', B_motor);
fprintf('C_motor = %.4f\n', C_motor);
fprintf('E_motor = %.4f\n', E_motor);

% 台車・振子システム
[A_cart, B_cart, C_cart] = cart_pendulum_system(params);
fprintf('\n台車・振子システム:\n');
fprintf('A_cart:\n'); disp(A_cart);
fprintf('B_cart:\n'); disp(B_cart);
fprintf('C_cart:\n'); disp(C_cart);

%% 2. システムの特性解析
fprintf('\n=== システム特性解析 ===\n');

% 台車・振子システムの固有値
eig_cart = eig(A_cart);
fprintf('\n台車・振子システムの固有値:\n');
disp(eig_cart);
if any(real(eig_cart) > 0)
    fprintf('→ 不安定システム（安定化制御が必要）\n');
else
    fprintf('→ 安定システム\n');
end

% 可制御性
P_cart = ctrb(A_cart, B_cart);
if rank(P_cart) == size(A_cart, 1)
    fprintf('\n台車・振子システム: 完全可制御\n');
else
    fprintf('\n台車・振子システム: 可制御でない\n');
end

% 可観測性
Q_cart = obsv(A_cart, C_cart);
if rank(Q_cart) == size(A_cart, 1)
    fprintf('台車・振子システム: 完全可観測\n');
else
    fprintf('台車・振子システム: 可観測でない\n');
end

%% 3. LQR制御器の設計（台車・振子システム）
fprintf('\n=== LQR制御器の設計 ===\n');

% 重み行列
Q_lqr = diag([10, 1, 100, 10]);  % [x, x_dot, phi, phi_dot]の重み
R_lqr = 1;  % 入力の重み

% LQRゲインの計算
[K_lqr, S_lqr, E_lqr] = lqr(A_cart, B_cart, Q_lqr, R_lqr);
fprintf('\nLQRゲイン K:\n');
disp(K_lqr);

% 閉ループシステムの固有値
A_cl = A_cart - B_cart * K_lqr;
eig_cl = eig(A_cl);
fprintf('\n閉ループシステムの固有値:\n');
disp(eig_cl);

%% 4. カスケード制御系の構成関数
function [V, F_motor, i] = cascade_controller(x_cart, x_ref, K_lqr, K_motor, params)
    % カスケード制御器
    % 入力:
    %   x_cart: 台車・振子の状態 [x; x_dot; phi; phi_dot]
    %   x_ref: 目標状態
    %   K_lqr: LQRゲイン
    %   K_motor: モータ制御ゲイン
    %   params: システムパラメータ
    % 出力:
    %   V: モータ印加電圧
    %   F_motor: モータ推力
    %   i: モータ電流
    
    % パラメータの展開
    Kt = params(11);
    Ke = params(12);
    Ra = params(13);
    G = params(7);
    r = params(5);
    
    % 外側ループ：LQR制御による目標推力の計算
    e = x_ref - x_cart;
    F_motor_ref = -K_lqr * e;
    
    % 目標電流の計算
    i_ref = F_motor_ref * r / (2 * G * Kt);
    
    % 内側ループ：電流制御（簡易版）
    % 実際の実装では電流フィードバックが必要
    i = i_ref;  % 理想的な電流追従を仮定
    
    % 必要な電圧の計算（逆起電力補償付き）
    x_dot = x_cart(2);
    V = Ra * i + Ke * G * x_dot / r + K_motor * (i_ref - i);
    
    % 実際のモータ推力
    F_motor = 2 * G * Kt * i / r;
end

%% 5. シミュレーション用の統合システム
function dX = integrated_system(t, X, K_lqr, K_motor, x_ref, params)
    % 統合システムの状態方程式
    % 状態: X = [x; x_dot; phi; phi_dot; i]
    
    % 状態の分離
    x_cart = X(1:4);
    i = X(5);
    
    % カスケード制御
    [V, ~, ~] = cascade_controller(x_cart, x_ref, K_lqr, K_motor, params);
    
    % 非線形動力学の呼び出し
    dX = nonlinear_dynamics(X, V, params);
end

%% 6. シミュレーション例
fprintf('\n=== シミュレーション例 ===\n');

% 初期条件
x0 = [0; 0; 0.05; 0; 0];  % [x, x_dot, phi, phi_dot, i]
x_ref = [0.5; 0; 0; 0];   % 目標位置0.5m、振子直立

% シミュレーション時間
tspan = [0, 5];

% モータ制御ゲイン
K_motor = 10;

% ODEソルバーの呼び出し
[t, X] = ode45(@(t, X) integrated_system(t, X, K_lqr, K_motor, x_ref, params), ...
               tspan, x0);

% 結果のプロット
figure('Position', [100, 100, 1000, 800]);

subplot(3,2,1);
plot(t, X(:,1), 'b-', 'LineWidth', 2);
hold on;
plot(t, x_ref(1)*ones(size(t)), 'r--', 'LineWidth', 1);
xlabel('時間 [s]');
ylabel('位置 x [m]');
title('台車位置');
grid on;
legend('実際', '目標');

subplot(3,2,2);
plot(t, X(:,2), 'b-', 'LineWidth', 2);
xlabel('時間 [s]');
ylabel('速度 ẋ [m/s]');
title('台車速度');
grid on;

subplot(3,2,3);
plot(t, X(:,3)*180/pi, 'b-', 'LineWidth', 2);
hold on;
plot(t, zeros(size(t)), 'r--', 'LineWidth', 1);
xlabel('時間 [s]');
ylabel('角度 φ [deg]');
title('振子角度');
grid on;
legend('実際', '目標');

subplot(3,2,4);
plot(t, X(:,4)*180/pi, 'b-', 'LineWidth', 2);
xlabel('時間 [s]');
ylabel('角速度 φ̇ [deg/s]');
title('振子角速度');
grid on;

subplot(3,2,5);
plot(t, X(:,5), 'b-', 'LineWidth', 2);
xlabel('時間 [s]');
ylabel('電流 i [A]');
title('モータ電流');
grid on;

% モータ推力の計算
F_motor = zeros(length(t), 1);
for k = 1:length(t)
    F_motor(k) = C_motor * X(k,5);
end

subplot(3,2,6);
plot(t, F_motor, 'b-', 'LineWidth', 2);
xlabel('時間 [s]');
ylabel('推力 F [N]');
title('モータ推力');
grid on;

sgtitle('2モータ対称制御倒立振子システムの応答');

fprintf('\nシミュレーション完了\n');
fprintf('最終位置誤差: %.4f m\n', abs(X(end,1) - x_ref(1)));
fprintf('最終角度誤差: %.4f deg\n', abs(X(end,3)*180/pi));

%% 7. 周波数応答解析
fprintf('\n=== 周波数応答解析 ===\n');

% 台車・振子システムの伝達関数
sys_cart = ss(A_cart, B_cart, C_cart, zeros(2,1));

% ボード線図
figure('Position', [100, 100, 800, 600]);

% 推力から位置への伝達関数
subplot(2,1,1);
bode(sys_cart(1,1), {10^-1, 10^2});
title('推力から位置への周波数応答');
grid on;

% 推力から角度への伝達関数
subplot(2,1,2);
bode(sys_cart(2,1), {10^-1, 10^2});
title('推力から角度への周波数応答');
grid on;

%% 8. 制御性能の評価関数
function performance = evaluate_controller(K_lqr, params, x0, x_ref, tf)
    % 制御性能の評価
    % 入力:
    %   K_lqr: LQRゲイン
    %   params: システムパラメータ
    %   x0: 初期状態
    %   x_ref: 目標状態
    %   tf: 評価時間
    % 出力:
    %   performance: 性能指標構造体
    
    K_motor = 10;
    [t, X] = ode45(@(t, X) integrated_system(t, X, K_lqr, K_motor, x_ref, params), ...
                   [0, tf], x0);
    
    % 性能指標の計算
    performance.settling_time_x = find_settling_time(t, X(:,1), x_ref(1), 0.02);
    performance.settling_time_phi = find_settling_time(t, X(:,3), 0, 0.01);
    performance.overshoot_x = max(abs(X(:,1) - x_ref(1))) / abs(x_ref(1)) * 100;
    performance.max_angle = max(abs(X(:,3))) * 180/pi;
    performance.max_current = max(abs(X(:,5)));
    performance.energy = trapz(t, X(:,5).^2);  % 消費エネルギー（電流の2乗積分）
end

function ts = find_settling_time(t, y, y_ref, threshold)
    % 整定時間の計算
    error = abs(y - y_ref);
    idx = find(error > threshold * abs(y_ref), 1, 'last');
    if isempty(idx)
        ts = 0;
    else
        ts = t(idx);
    end
end

%% 9. パラメータ感度解析
fprintf('\n=== パラメータ感度解析 ===\n');

% 基準パラメータでの性能
perf_base = evaluate_controller(K_lqr, params, x0, x_ref, 5);
fprintf('\n基準性能:\n');
fprintf('  位置整定時間: %.2f s\n', perf_base.settling_time_x);
fprintf('  角度整定時間: %.2f s\n', perf_base.settling_time_phi);
fprintf('  最大角度偏差: %.2f deg\n', perf_base.max_angle);
fprintf('  消費エネルギー: %.2f J\n', perf_base.energy);

% 振子質量の変化に対する感度
m_variations = [0.08, 0.09, 0.1, 0.11, 0.12];  % ±20%
settling_times = zeros(length(m_variations), 1);

for i = 1:length(m_variations)
    params_temp = params;
    params_temp(2) = m_variations(i);
    
    % 新しいシステム行列
    [A_cart_temp, B_cart_temp, ~] = cart_pendulum_system(params_temp);
    
    % 同じLQRゲインを使用（ロバスト性の評価）
    perf_temp = evaluate_controller(K_lqr, params_temp, x0, x_ref, 5);
    settling_times(i) = perf_temp.settling_time_phi;
end

figure('Position', [100, 100, 600, 400]);
plot(m_variations, settling_times, 'bo-', 'LineWidth', 2);
xlabel('振子質量 m [kg]');
ylabel('角度整定時間 [s]');
title('振子質量に対する制御性能の感度');
grid on;

%% 10. 最適なLQR重みの探索（オプション）
fprintf('\n=== LQR重み最適化（簡易版）===\n');

% 重み行列のパラメータ
q_x = logspace(0, 2, 5);      % 位置の重み
q_phi = logspace(1, 3, 5);    % 角度の重み

best_perf = inf;
best_Q = [];
best_K = [];

for i = 1:length(q_x)
    for j = 1:length(q_phi)
        Q_test = diag([q_x(i), 1, q_phi(j), 10]);
        R_test = 1;
        
        try
            [K_test, ~, ~] = lqr(A_cart, B_cart, Q_test, R_test);
            perf_test = evaluate_controller(K_test, params, x0, x_ref, 5);
            
            % 評価関数（整定時間と消費エネルギーのトレードオフ）
            cost = perf_test.settling_time_x + perf_test.settling_time_phi + ...
                   0.1 * perf_test.energy;
            
            if cost < best_perf
                best_perf = cost;
                best_Q = Q_test;
                best_K = K_test;
            end
        catch
            % LQR設計が失敗した場合はスキップ
            continue;
        end
    end
end

fprintf('\n最適なLQR重み:\n');
fprintf('Q = diag([%.1f, %.1f, %.1f, %.1f])\n', diag(best_Q));
fprintf('最適なLQRゲイン:\n');
disp(best_K);

fprintf('\n=== すべての解析が完了しました ===\n');