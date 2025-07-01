%% derive_separated_systems.m
% DCモータと台車・振子の分離されたサブシステムの導出

clear; clc;

%% シンボリック変数の定義
syms x x_dot x_ddot phi phi_dot phi_ddot real
syms M m g l r Iw G Jp bx btheta Kt Ke Ra La real
syms i i_dot V F_motor real

%% 1. DCモータのサブシステム
fprintf('=== DCモータのサブシステム ===\n');
fprintf('状態変数: X_motor = [i]\n');
fprintf('入力: u_motor = V (印加電圧)\n');
fprintf('出力: y_motor = F_motor (モータ推力)\n\n');

% 電気系の方程式
% V = Ra*i + La*di/dt + Ke*omega_motor
% omega_motor = G*x_dot/r

% 状態方程式: di/dt = (V - Ra*i - Ke*G*x_dot/r)/La
i_dot_expr = (V - Ra*i - Ke*G*x_dot/r)/La;

% 状態空間表現（x_dotを外部入力として扱う）
A_motor = -Ra/La;
B_motor = 1/La;
E_motor = -Ke*G/(r*La);  % x_dotの係数（外乱項）

% 出力方程式: F_motor = 2*G*Kt*i/r
C_motor = 2*G*Kt/r;

fprintf('状態方程式: ẋ_motor = A_motor*x_motor + B_motor*u_motor + E_motor*x_dot\n');
fprintf('A_motor = -Ra/La\n');
fprintf('B_motor = 1/La\n');
fprintf('E_motor = -Ke*G/(r*La)\n');
fprintf('C_motor = 2*G*Kt/r\n\n');

%% 2. 台車・振子のサブシステム（運動方程式から導出）
fprintf('=== 台車・振子のサブシステム ===\n');
fprintf('状態変数: X_cart = [x; x_dot; phi; phi_dot]\n');
fprintf('入力: u_cart = F_motor (モータ推力)\n');
fprintf('出力: y_cart = [x; phi]\n\n');

% ラグランジアンの再構築（簡略版）
% 運動エネルギー
T = (1/2)*(M + 2*Iw/r^2)*x_dot^2 + (1/2)*m*(x_dot^2 + 2*l*x_dot*phi_dot*cos(phi) + l^2*phi_dot^2) + (1/2)*Jp*phi_dot^2;
% 位置エネルギー
V_pot = m*g*l*cos(phi);
% ラグランジアン
L = T - V_pot;

% ラグランジュの運動方程式
% x方向
dL_dx_dot = diff(L, x_dot);
dt_dL_dx_dot = diff(dL_dx_dot, x)*x_dot + diff(dL_dx_dot, x_dot)*x_ddot + ...
               diff(dL_dx_dot, phi)*phi_dot + diff(dL_dx_dot, phi_dot)*phi_ddot;
dL_dx = diff(L, x);
eq1 = dt_dL_dx_dot - dL_dx == F_motor - bx*x_dot;

% φ方向
dL_dphi_dot = diff(L, phi_dot);
dt_dL_dphi_dot = diff(dL_dphi_dot, x)*x_dot + diff(dL_dphi_dot, x_dot)*x_ddot + ...
                 diff(dL_dphi_dot, phi)*phi_dot + diff(dL_dphi_dot, phi_dot)*phi_ddot;
dL_dphi = diff(L, phi);
eq2 = dt_dL_dphi_dot - dL_dphi == -btheta*phi_dot;

% 行列形式
[M_mat, rhs] = equationsToMatrix([eq1, eq2], [x_ddot, phi_ddot]);
M_mat = simplify(M_mat);
rhs = simplify(rhs);

fprintf('慣性行列 M(q):\n');
pretty(M_mat);

% 平衡点での線形化（phi=0, phi_dot=0, x_dot=0）
M_eq = subs(M_mat, [phi, phi_dot, x_dot], [0, 0, 0]);
M_eq = simplify(M_eq);

fprintf('\n平衡点での慣性行列 M_eq:\n');
pretty(M_eq);

% 線形化のための偏微分
% rhs = [F_motor - bx*x_dot + f1(phi, phi_dot); f2(phi, phi_dot) - btheta*phi_dot]
drhs_dx = diff(rhs, x);
drhs_dx_dot = diff(rhs, x_dot);
drhs_dphi = diff(rhs, phi);
drhs_dphi_dot = diff(rhs, phi_dot);
drhs_dF = diff(rhs, F_motor);

% 平衡点での評価
drhs_dx_eq = subs(drhs_dx, [phi, phi_dot, x_dot], [0, 0, 0]);
drhs_dx_dot_eq = subs(drhs_dx_dot, [phi, phi_dot, x_dot], [0, 0, 0]);
drhs_dphi_eq = subs(drhs_dphi, [phi, phi_dot, x_dot], [0, 0, 0]);
drhs_dphi_dot_eq = subs(drhs_dphi_dot, [phi, phi_dot, x_dot], [0, 0, 0]);
drhs_dF_eq = subs(drhs_dF, [phi, phi_dot, x_dot], [0, 0, 0]);

% A行列とB行列の構築
M_eq_inv = inv(M_eq);
temp_A = M_eq_inv * [drhs_dx_eq, drhs_dx_dot_eq, drhs_dphi_eq, drhs_dphi_dot_eq];
temp_B = M_eq_inv * drhs_dF_eq;

A_cart = [0, 1, 0, 0;
          temp_A(1,:);
          0, 0, 0, 1;
          temp_A(2,:)];
          
B_cart = [0;
          temp_B(1);
          0;
          temp_B(2)];

C_cart = [1, 0, 0, 0;
          0, 0, 1, 0];

A_cart = simplify(A_cart);
B_cart = simplify(B_cart);

fprintf('\n線形化されたA_cart行列:\n');
pretty(A_cart);
fprintf('\n線形化されたB_cart行列:\n');
pretty(B_cart);

%% 3. 関数の保存
if ~exist('separated_systems', 'dir')
    mkdir('separated_systems');
end

% パラメータリスト
params = [M; m; g; l; r; Iw; G; Jp; bx; btheta; Kt; Ke; Ra; La];

% DCモータシステム
matlabFunction(A_motor, B_motor, C_motor, E_motor, ...
    'File', 'equation_of_state_v2/dynamics_functions/dc_motor_system', ...
    'Vars', {params}, ...
    'Outputs', {'A_motor', 'B_motor', 'C_motor', 'E_motor'});

% 台車・振子システム
matlabFunction(A_cart, B_cart, C_cart, ...
    'File', 'equation_of_state_v2/dynamics_functions/cart_pendulum_system', ...
    'Vars', {params}, ...
    'Outputs', {'A_cart', 'B_cart', 'C_cart'});

% 非線形台車・振子システム（F_motorを入力として）
% 加速度の解

accel_solution = M_mat \ rhs;
x_ddot_F = subs(accel_solution(1), i, F_motor*r/(2*G*Kt));
phi_ddot_F = subs(accel_solution(2), i, F_motor*r/(2*G*Kt));

f_cart_nonlinear = [x_dot;
                    x_ddot_F;
                    phi_dot;
                    phi_ddot_F];

matlabFunction(f_cart_nonlinear, ...
    'File', 'equation_of_state_v2/dynamics_functions/cart_pendulum_nonlinear', ...
    'Vars', {[x; x_dot; phi; phi_dot], F_motor, params}, ...
    'Outputs', {'dX_cart'});