%% derive_equations_of_motion.m
% ラグランジュ法による2モータ対称制御の倒立振子カートシステムの運動方程式導出

clear; clc;

%% シンボリック変数の定義
syms x x_dot x_ddot phi phi_dot phi_ddot real
syms M m g l r Iw G Jp bx btheta Kt Ke Ra La real
syms i i_dot V real
syms t real

%% 1. 運動エネルギーの計算
fprintf('=== 運動エネルギーの計算 ===\n');

% 台車の運動エネルギー
T_cart = (1/2) * M * x_dot^2;
fprintf('台車の運動エネルギー T_cart:\n');
pretty(T_cart);

% タイヤの運動エネルギー（2つのタイヤ）
omega_wheel = x_dot / r;  % タイヤの角速度
T_wheel = (1/2) * 2 * Iw * omega_wheel^2;
T_wheel = simplify(T_wheel);
fprintf('\nタイヤの運動エネルギー T_wheel:\n');
pretty(T_wheel);

% 振子重心の位置
xp = x + l * sin(phi);
yp = l * cos(phi);

% 振子重心の速度（時間微分）
xp_dot = diff(xp, x) * x_dot + diff(xp, phi) * phi_dot;
yp_dot = diff(yp, x) * x_dot + diff(yp, phi) * phi_dot;
xp_dot = simplify(xp_dot);
yp_dot = simplify(yp_dot);

fprintf('\n振子重心の速度:\n');
fprintf('ẋp = '); pretty(xp_dot);
fprintf('ẏp = '); pretty(yp_dot);

% 振子の並進運動エネルギー
T_pendulum_trans = (1/2) * m * (xp_dot^2 + yp_dot^2);
T_pendulum_trans = simplify(T_pendulum_trans);
fprintf('\n振子の並進運動エネルギー:\n');
pretty(T_pendulum_trans);

% 振子の回転運動エネルギー
T_pendulum_rot = (1/2) * Jp * phi_dot^2;

% 全運動エネルギー
T = T_cart + T_wheel + T_pendulum_trans + T_pendulum_rot;
T = simplify(T);
fprintf('\n全運動エネルギー T:\n');
pretty(T);

%% 2. 位置エネルギーの計算
V_potential = m * g * l * cos(phi);
fprintf('\n位置エネルギー V:\n');
pretty(V_potential);

%% 3. ラグランジアン
L = T - V_potential;
L = simplify(L);
fprintf('\nラグランジアン L = T - V:\n');
pretty(L);

%% 4. ラグランジュの運動方程式
% d/dt(∂L/∂q̇) - ∂L/∂q = Q

% x方向の運動方程式
dL_dx_dot = diff(L, x_dot);
fprintf('\n∂L/∂ẋ:\n');
pretty(dL_dx_dot);

% 時間微分 d/dt(∂L/∂ẋ)
dt_dL_dx_dot = diff(dL_dx_dot, x) * x_dot + ...
               diff(dL_dx_dot, x_dot) * x_ddot + ...
               diff(dL_dx_dot, phi) * phi_dot + ...
               diff(dL_dx_dot, phi_dot) * phi_ddot;
dt_dL_dx_dot = simplify(dt_dL_dx_dot);

dL_dx = diff(L, x);
fprintf('\n∂L/∂x:\n');
pretty(dL_dx);

% φ方向の運動方程式
dL_dphi_dot = diff(L, phi_dot);
fprintf('\n∂L/∂φ̇:\n');
pretty(dL_dphi_dot);

% 時間微分 d/dt(∂L/∂φ̇)
dt_dL_dphi_dot = diff(dL_dphi_dot, x) * x_dot + ...
                 diff(dL_dphi_dot, x_dot) * x_ddot + ...
                 diff(dL_dphi_dot, phi) * phi_dot + ...
                 diff(dL_dphi_dot, phi_dot) * phi_ddot;
dt_dL_dphi_dot = simplify(dt_dL_dphi_dot);

dL_dphi = diff(L, phi);
fprintf('\n∂L/∂φ:\n');
pretty(dL_dphi);

%% 5. 一般化力
% 2モータ対称制御
F_motor = 2 * G * Kt * i / r;  % 2つのモータの合計推力
F_damping_x = -bx * x_dot;     % 台車の粘性摩擦
tau_damping_phi = -btheta * phi_dot;  % 振子の粘性摩擦

Q_x = F_motor + F_damping_x;
Q_phi = tau_damping_phi;

fprintf('\n一般化力:\n');
fprintf('Qx = '); pretty(Q_x);
fprintf('Qφ = '); pretty(Q_phi);

%% 6. 運動方程式
eq1 = dt_dL_dx_dot - dL_dx == Q_x;
eq2 = dt_dL_dphi_dot - dL_dphi == Q_phi;

eq1 = simplify(eq1);
eq2 = simplify(eq2);

fprintf('\n運動方程式:\n');
fprintf('x方向: '); pretty(eq1);
fprintf('\nφ方向: '); pretty(eq2);

%% 7. 行列形式に整理
[M_matrix, rhs] = equationsToMatrix([eq1, eq2], [x_ddot, phi_ddot]);
M_matrix = simplify(M_matrix);
rhs = simplify(rhs);

fprintf('\n慣性行列 M(q):\n');
pretty(M_matrix);
fprintf('\n右辺ベクトル:\n');
pretty(rhs);

%% 8. 状態方程式の構築
% 状態変数: X = [x; x_dot; phi; phi_dot; i]

% 加速度の解
accelerations = simplify(M_matrix \ rhs);
x_ddot_expr = accelerations(1);
phi_ddot_expr = accelerations(2);

% 電気系の方程式
i_dot_expr = (V - Ra * i - Ke * G * x_dot / r) / La;

% 非線形状態方程式
f_nonlinear = [x_dot; 
               x_ddot_expr; 
               phi_dot; 
               phi_ddot_expr;
               i_dot_expr];

fprintf('\n非線形状態方程式 dX/dt = f(X, V):\n');
for k = 1:length(f_nonlinear)
    fprintf('f%d = ', k);
    pretty(f_nonlinear(k));
end

%% 9. 平衡点での線形化
fprintf('\n=== 平衡点での線形化 ===\n');
% 平衡点: x = 0, x_dot = 0, phi = 0, phi_dot = 0, i = 0

% 状態変数と入力
state_vars = [x; x_dot; phi; phi_dot; i];
input_var = V;

% ヤコビアンの計算
A_nonlinear = jacobian(f_nonlinear, state_vars);
B_nonlinear = jacobian(f_nonlinear, input_var);

% 平衡点での評価
equilibrium = [0; 0; 0; 0; 0];
V_eq = 0;

A_linear = subs(A_nonlinear, [state_vars; input_var], [equilibrium; V_eq]);
B_linear = subs(B_nonlinear, [state_vars; input_var], [equilibrium; V_eq]);

A_linear = simplify(A_linear);
B_linear = simplify(B_linear);

fprintf('線形化されたA行列:\n');
pretty(A_linear);
fprintf('\n線形化されたB行列:\n');
pretty(B_linear);

%% 10. 関数の保存
% フォルダの作成
if ~exist('dynamics_functions', 'dir')
    mkdir('dynamics_functions');
end

% パラメータリスト
params = [M; m; g; l; r; Iw; G; Jp; bx; btheta; Kt; Ke; Ra; La];

% 非線形動力学関数
matlabFunction(f_nonlinear, 'File', 'equation_of_state_v2/dynamics_functions/nonlinear_dynamics', ...
    'Vars', {state_vars, input_var, params}, ...
    'Outputs', {'dXdt'});

% 線形化行列関数
matlabFunction(A_linear, B_linear, 'File', 'equation_of_state_v2/dynamics_functions/linearized_matrices', ...
    'Vars', {params}, ...
    'Outputs', {'A', 'B'});

% 慣性行列と右辺ベクトル（運動方程式用）
matlabFunction(M_matrix, rhs, 'File', 'equation_of_state_v2/dynamics_functions/equations_of_motion', ...
    'Vars', {[x; phi], [x_dot; phi_dot], i, params}, ...
    'Outputs', {'M', 'rhs'});