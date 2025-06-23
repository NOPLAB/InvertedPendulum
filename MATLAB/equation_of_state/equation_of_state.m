% シンボリック変数の定義
syms x x_dot x_ddot phi phi_dot phi_ddot i i_dot V real
syms M m g l r Iw n G Jp bx btheta Kt Ke Ra La real

% === ラグランジュの運動方程式の導出 ===
% 座標系定義：φ=0で振子が上向き（倒立状態）

% 1. 運動エネルギーの計算
% 台車の運動エネルギー
T_cart = (1/2) * M * x_dot^2;

% タイヤの運動エネルギー（回転）
omega_wheel = x_dot / r;  % タイヤの角速度
T_wheel = (1/2) * n * Iw * omega_wheel^2;

% 振子の運動エネルギー
% 振子重心の位置
xp = x + l * sin(phi);
yp = l * cos(phi);

% 振子重心の速度
xp_dot = diff(xp, x) * x_dot + diff(xp, phi) * phi_dot;
yp_dot = diff(yp, x) * x_dot + diff(yp, phi) * phi_dot;

% 簡単化
xp_dot = x_dot + l * phi_dot * cos(phi);
yp_dot = -l * phi_dot * sin(phi);

% 振子の並進運動エネルギー
T_pendulum_trans = (1/2) * m * (xp_dot^2 + yp_dot^2);

% 振子の回転運動エネルギー
T_pendulum_rot = (1/2) * Jp * phi_dot^2;

% 全運動エネルギー
T = T_cart + T_wheel + T_pendulum_trans + T_pendulum_rot;
T = simplify(T);

% 2. 位置エネルギーの計算
V_potential = m * g * yp;
V_potential = m * g * l * cos(phi);

% 3. ラグランジアン
L = T - V_potential;

% 4. 一般化座標と一般化速度
q = [x; phi];
q_dot = [x_dot; phi_dot];
q_ddot = [x_ddot; phi_ddot];

% 5. ラグランジュの運動方程式
% d/dt(∂L/∂q_dot) - ∂L/∂q = Q

% ∂L/∂q_dot の計算
dL_dx_dot = diff(L, x_dot);
dL_dphi_dot = diff(L, phi_dot);

% d/dt(∂L/∂q_dot) の計算
% 時間微分には連鎖律を使用
dt_dL_dx_dot = diff(dL_dx_dot, x) * x_dot + diff(dL_dx_dot, x_dot) * x_ddot + ...
               diff(dL_dx_dot, phi) * phi_dot + diff(dL_dx_dot, phi_dot) * phi_ddot;

dt_dL_dphi_dot = diff(dL_dphi_dot, x) * x_dot + diff(dL_dphi_dot, x_dot) * x_ddot + ...
                 diff(dL_dphi_dot, phi) * phi_dot + diff(dL_dphi_dot, phi_dot) * phi_ddot;

% ∂L/∂q の計算
dL_dx = diff(L, x);
dL_dphi = diff(L, phi);

% 一般化力の計算
% モータによる力
F_motor = G * Kt * i / r;
tau_motor = 0;  % 振子には直接作用しない

% 減衰力
F_damping_x = -bx * x_dot;
tau_damping_phi = -btheta * phi_dot;

% 一般化力
Q_x = F_motor + F_damping_x;
Q_phi = tau_motor + tau_damping_phi;

% ラグランジュの運動方程式
eq1 = dt_dL_dx_dot - dL_dx == Q_x;
eq2 = dt_dL_dphi_dot - dL_dphi == Q_phi;

% 運動方程式を整理して行列形式にする
% M(q) * q_ddot + C(q, q_dot) * q_dot + G(q) = Q

% eq1とeq2を x_ddot と phi_ddot について整理
[M_matrix, rhs] = equationsToMatrix([eq1, eq2], [x_ddot, phi_ddot]);
M_matrix = simplify(M_matrix);
rhs = simplify(rhs);

% === 非線形運動方程式の保存 ===
% 状態変数: X = [x; x_dot; phi; phi_dot; i]
% 状態方程式: dX/dt = f(X, V)

% 加速度を求める（非線形）
% [x_ddot; phi_ddot] = M_matrix^(-1) * rhs
M_inv = inv(M_matrix);
accelerations = M_inv * rhs;
x_ddot_expr = simplify(accelerations(1));
phi_ddot_expr = simplify(accelerations(2));

% 電気系の方程式
% V = Ra * i + La * di/dt + Ke * omega_motor
% omega_motor = G * x_dot / r
i_dot_expr = (V - Ra * i - Ke * G * x_dot / r) / La;

% 非線形状態方程式 dX/dt = f(X, V)
% X = [x; x_dot; phi; phi_dot; i]
f_nonlinear = [x_dot; 
               x_ddot_expr; 
               phi_dot; 
               phi_ddot_expr; 
               i_dot_expr];

% 非線形関数の保存
% 入力引数の順序を明示的に指定
state_vars = [x; x_dot; phi; phi_dot; i];
input_var = V;
params = [M; m; g; l; r; Iw; n; G; Jp; bx; btheta; Kt; Ke; Ra; La];

% 非線形状態方程式を関数として保存
matlabFunction(f_nonlinear, 'File', 'equation_of_state/nonlinear_dynamics', ...
    'Vars', {state_vars, input_var, params}, ...
    'Outputs', {'dXdt'});

% 慣性行列M(q)と右辺項を別々に保存（解析用）
% matlabFunction(M_matrix, 'File', 'equation_of_state/mass_matrix', ...
%     'Vars', {phi, params}, ...
%     'Outputs', {'M'});

% matlabFunction(rhs, 'File', 'equation_of_state/rhs_vector', ...
%     'Vars', {[x; x_dot; phi; phi_dot; i], params}, ...
%     'Outputs', {'rhs'});

% === 線形化された運動方程式の導出 ===
% 6. 平衡点での線形化
% 平衡点: phi = 0, phi_dot = 0, x_dot = 0

% 線形化のための変数置換
phi_eq = 0;
phi_dot_eq = 0;
x_dot_eq = 0;
i_eq = 0;  % 電流の平衡点

% M行列の平衡点での値
M_eq = subs(M_matrix, [phi, phi_dot, x_dot], [phi_eq, phi_dot_eq, x_dot_eq]);

% 右辺の線形化
% rhs = -C(q,q_dot)*q_dot - G(q) + Q
% 平衡点周りでテイラー展開

% 右辺を各変数で偏微分して線形化係数を求める
% ∂rhs/∂x, ∂rhs/∂x_dot, ∂rhs/∂phi, ∂rhs/∂phi_dot, ∂rhs/∂i

% 平衡点での偏微分
drhs_dx = diff(rhs, x);
drhs_dx_dot = diff(rhs, x_dot);
drhs_dphi = diff(rhs, phi);
drhs_dphi_dot = diff(rhs, phi_dot);
drhs_di = diff(rhs, i);

% 平衡点での値
drhs_dx_eq = subs(drhs_dx, [phi, phi_dot, x_dot, i], [phi_eq, phi_dot_eq, x_dot_eq, i_eq]);
drhs_dx_dot_eq = subs(drhs_dx_dot, [phi, phi_dot, x_dot, i], [phi_eq, phi_dot_eq, x_dot_eq, i_eq]);
drhs_dphi_eq = subs(drhs_dphi, [phi, phi_dot, x_dot, i], [phi_eq, phi_dot_eq, x_dot_eq, i_eq]);
drhs_dphi_dot_eq = subs(drhs_dphi_dot, [phi, phi_dot, x_dot, i], [phi_eq, phi_dot_eq, x_dot_eq, i_eq]);
drhs_di_eq = subs(drhs_di, [phi, phi_dot, x_dot, i], [phi_eq, phi_dot_eq, x_dot_eq, i_eq]);

% 7. 電気系の方程式（既に上で定義済み）

% 8. 状態空間表現の構築
% 状態変数: X = [x; x_dot; phi; phi_dot; i]

% M_eq * [x_ddot; phi_ddot] = drhs_dx_eq * x + drhs_dx_dot_eq * x_dot + 
%                               drhs_dphi_eq * phi + drhs_dphi_dot_eq * phi_dot + drhs_di_eq * i

% M_eq^(-1) を計算
M_eq_inv = inv(M_eq);

% 線形化された加速度
% [x_ddot; phi_ddot] = M_eq_inv * [drhs_dx_eq, drhs_dx_dot_eq, drhs_dphi_eq, drhs_dphi_dot_eq, drhs_di_eq] * [x; x_dot; phi; phi_dot; i]

% A行列の構築
A_sym = sym(zeros(5, 5));

% dx/dt = x_dot
A_sym(1, 2) = 1;

% dφ/dt = φ_dot
A_sym(3, 4) = 1;

% dx_dot/dt と dφ_dot/dt の係数
temp = M_eq_inv * [drhs_dx_eq, drhs_dx_dot_eq, drhs_dphi_eq, drhs_dphi_dot_eq, drhs_di_eq];
A_sym(2, 1:5) = temp(1, :);
A_sym(4, 1:5) = temp(2, :);

% di/dt の係数
A_sym(5, 2) = -Ke * G / (r * La);
A_sym(5, 5) = -Ra / La;

% B行列
B_sym = sym(zeros(5, 1));
B_sym(5, 1) = 1 / La;

% 線形化された状態方程式を保存
matlabFunction(A_sym, 'File', 'equation_of_state/linear_A_matrix', ...
    'Vars', {params}, ...
    'Outputs', {'A'});

matlabFunction(B_sym, 'File', 'equation_of_state/linear_B_matrix', ...
    'Vars', {params}, ...
    'Outputs', {'B'});
