%% 台車型倒立振子のシンボリック状態方程式導出
% このプログラムはシンボリック計算を用いて運動方程式を導出し、
% 線形化してシンボリック形式の状態方程式を求めます（Parameter Estimator用）
clear all; close all; clc;

%% シンボリック変数の定義
fprintf('シンボリック変数を定義中...\n');

% 時間変数
syms t real

% 状態変数（時間依存）
syms x(t) theta(t) phi(t) i(t)

% 状態変数（通常のシンボリック変数）
syms x_var x_dot_var x_ddot_var real
syms theta_var theta_dot_var theta_ddot_var real
syms phi_var phi_dot_var real
syms i_var i_dot_var real
syms V_var real  % 入力電圧

% 物理パラメータ（全てシンボリックのまま）
syms M m g l r Iw n G real positive      % 基本物理パラメータ
syms Jp bx btheta real positive          % 慣性・摩擦パラメータ  
syms Kt Ke Ra La real positive          % モータパラメータ

fprintf('定義されたパラメータ:\n');
fprintf('M: 台車質量, m: 振子質量, g: 重力加速度\n');
fprintf('l: 振子長さ, r: タイヤ半径, Iw: タイヤ慣性モーメント\n');
fprintf('n: タイヤ数, G: 減速比, Jp: 振子慣性モーメント\n');
fprintf('bx: 台車摩擦係数, btheta: 振子摩擦係数\n');
fprintf('Kt: トルク定数, Ke: 逆起電力定数, Ra: 電機子抵抗, La: 電機子インダクタンス\n\n');

% 時間微分の定義
x_dot = diff(x, t);
x_ddot = diff(x_dot, t);
theta_dot = diff(theta, t);
theta_ddot = diff(theta_dot, t);
phi_dot = diff(phi, t);
i_dot = diff(i, t);

%% 運動方程式の導出（ラグランジュ法）
fprintf('ラグランジュ法により運動方程式を導出中...\n');

% 拘束条件: タイヤが滑らない
% x_dot = r * phi_dot (または omega = phi_dot = x_dot/r)

% 振子重心の位置と速度
% 振子重心位置: (x + l*sin(theta), l*cos(theta))
% 振子重心速度
xp_dot = x_dot + l * cos(theta) * theta_dot;  % x方向速度
yp_dot = -l * sin(theta) * theta_dot;         % y方向速度（上向き正）

% 運動エネルギー
% 1. 台車の並進運動エネルギー
T_cart = (1/2) * M * x_dot^2;

% 2. タイヤの回転運動エネルギー（拘束条件使用）
% omega = x_dot/r なので
T_wheel = (1/2) * n * Iw * (x_dot/r)^2;

% 3. 振子の並進運動エネルギー
T_pendulum_trans = (1/2) * m * (xp_dot^2 + yp_dot^2);

% 4. 振子の回転運動エネルギー
T_pendulum_rot = (1/2) * Jp * theta_dot^2;

% 総運動エネルギー
T_total = T_cart + T_wheel + T_pendulum_trans + T_pendulum_rot;

fprintf('運動エネルギー項:\n');
fprintf('T_cart = (1/2)*M*x_dot^2\n');
fprintf('T_wheel = (1/2)*n*Iw*(x_dot/r)^2\n');
fprintf('T_pendulum = (1/2)*m*(xp_dot^2 + yp_dot^2) + (1/2)*Jp*theta_dot^2\n\n');

% 位置エネルギー（振子重心の高さ: l*cos(theta)、上向き正）
U_total = m * g * l * cos(theta);

fprintf('位置エネルギー:\n');
fprintf('U = m*g*l*cos(theta)\n\n');

% ラグランジアン
L = T_total - U_total;

%% オイラー・ラグランジュ方程式の適用
fprintf('オイラー・ラグランジュ方程式を適用中...\n');

% x方向の運動方程式
% d/dt(∂L/∂ẋ) - ∂L/∂x = F_x
dL_dx_dot = diff(L, x_dot);
dL_dx = diff(L, x);

% 時間微分を手動で計算
d_dt_dL_dx_dot = diff(dL_dx_dot, x_dot) * x_ddot + ...
                 diff(dL_dx_dot, theta_dot) * theta_ddot + ...
                 diff(dL_dx_dot, theta) * theta_dot;

eq_x = d_dt_dL_dx_dot - dL_dx;

% theta方向の運動方程式  
% d/dt(∂L/∂θ̇) - ∂L/∂θ = τ_theta
dL_dtheta_dot = diff(L, theta_dot);
dL_dtheta = diff(L, theta);

d_dt_dL_dtheta_dot = diff(dL_dtheta_dot, x_dot) * x_ddot + ...
                     diff(dL_dtheta_dot, theta_dot) * theta_ddot + ...
                     diff(dL_dtheta_dot, theta) * theta_dot;

eq_theta = d_dt_dL_dtheta_dot - dL_dtheta;

%% 通常の変数を用いた運動方程式の整理
fprintf('運動方程式を行列形式で整理中...\n');

% 時間関数を通常の変数に置換
eq_x_sub = subs(eq_x, [x(t), diff(x(t),t), diff(x(t),t,2), theta(t), diff(theta(t),t), diff(theta(t),t,2)], ...
                     [x_var, x_dot_var, x_ddot_var, theta_var, theta_dot_var, theta_ddot_var]);

eq_theta_sub = subs(eq_theta, [x(t), diff(x(t),t), diff(x(t),t,2), theta(t), diff(theta(t),t), diff(theta(t),t,2)], ...
                             [x_var, x_dot_var, x_ddot_var, theta_var, theta_dot_var, theta_ddot_var]);

% より直接的な方法で質量行列を構築（係数抽出の問題を回避）
fprintf('質量行列を直接計算中...\n');

% 運動エネルギーから質量行列を導出
T_expanded = (1/2) * M * x_dot_var^2 + ...
             (1/2) * n * Iw * (x_dot_var/r)^2 + ...
             (1/2) * m * ((x_dot_var + l * cos(theta_var) * theta_dot_var)^2 + ...
                         (-l * sin(theta_var) * theta_dot_var)^2) + ...
             (1/2) * Jp * theta_dot_var^2;

% 質量行列の要素を直接計算
M11 = diff(diff(T_expanded, x_dot_var), x_dot_var);
M12 = diff(diff(T_expanded, x_dot_var), theta_dot_var);
M21 = diff(diff(T_expanded, theta_dot_var), x_dot_var);
M22 = diff(diff(T_expanded, theta_dot_var), theta_dot_var);

% 質量行列
M_matrix = [M11, M12; M21, M22];

fprintf('質量行列 M:\n');
fprintf('M11 = %s\n', char(M11));
fprintf('M12 = %s\n', char(M12));
fprintf('M21 = %s\n', char(M21));
fprintf('M22 = %s\n', char(M22));
fprintf('\n');

% コリオリ・遠心力項と減衰項
C11 = bx;  % 台車の粘性摩擦
C12 = -m * l * sin(theta_var) * theta_dot_var;  % コリオリ項
C21 = 0;
C22 = btheta;  % 振子の粘性摩擦

C_matrix = [C11, C12; C21, C22];

fprintf('減衰・コリオリ行列 C:\n');
fprintf('C11 = %s\n', char(C11));
fprintf('C12 = %s\n', char(C12));
fprintf('C21 = %s\n', char(C21));
fprintf('C22 = %s\n', char(C22));
fprintf('\n');

% 重力項
G1 = 0;
G2 = -m * g * l * sin(theta_var);

G_vector = [G1; G2];

fprintf('重力項 G:\n');
fprintf('G1 = %s\n', char(G1));
fprintf('G2 = %s\n', char(G2));
fprintf('\n');

% 入力項（モータによる駆動力）
% モータトルク: τ_motor = G * Kt * i
% タイヤを通じた推力: F = n * τ_motor / r = n * G * Kt * i / r
F1 = n * G * Kt * i_var / r;
F2 = 0;  % モータは台車にのみ作用

F_vector = [F1; F2];

fprintf('入力項 F:\n');
fprintf('F1 = %s\n', char(F1));
fprintf('F2 = %s\n', char(F2));
fprintf('\n');

%% モータの電気方程式
fprintf('モータの電気方程式を導出中...\n');

% 電圧方程式: V = Ra*i + La*di/dt + e
% 逆起電力: e = Ke * omega_motor = Ke * G * (x_dot/r)
% よって: La*di/dt = V - Ra*i - Ke*G*x_dot/r

i_dot_equation = (V_var - Ra*i_var - Ke*G*x_dot_var/r) / La;

fprintf('電気方程式:\n');
fprintf('di/dt = %s\n', char(i_dot_equation));
fprintf('\n');

%% 線形化（平衡点: x=0, theta=0, 全ての速度=0, i=0）
fprintf('平衡点周りで線形化を実行中...\n');

% 線形化の近似
% sin(theta) ≈ theta
% cos(theta) ≈ 1
% theta * theta_dot ≈ 0 (2次項を無視)

% 質量行列の線形化
M11_lin = subs(M11, cos(theta_var), 1);
M12_lin = subs(M12, cos(theta_var), 1);
M21_lin = subs(M21, cos(theta_var), 1);
M22_lin = subs(M22, cos(theta_var), 1);

M_lin = [M11_lin, M12_lin; M21_lin, M22_lin];

fprintf('線形化された質量行列:\n');
fprintf('M11_lin = %s\n', char(M11_lin));
fprintf('M12_lin = %s\n', char(M12_lin));
fprintf('M21_lin = %s\n', char(M21_lin));
fprintf('M22_lin = %s\n', char(M22_lin));
fprintf('\n');

% 減衰行列の線形化（C12項の2次項を除去）
C11_lin = C11;
C12_lin = 0;  % theta * theta_dot ≈ 0
C21_lin = C21;
C22_lin = C22;

C_lin = [C11_lin, C12_lin; C21_lin, C22_lin];

% 重力項の線形化
G1_lin = 0;
G2_lin = subs(G2, sin(theta_var), theta_var);  % -m*g*l*theta

G_lin = [G1_lin; G2_lin];

fprintf('線形化された重力項:\n');
fprintf('G1_lin = %s\n', char(G1_lin));
fprintf('G2_lin = %s\n', char(G2_lin));
fprintf('\n');

% 入力項は変更なし
F_lin = F_vector;

%% 状態空間表現の構築
fprintf('状態空間表現を構築中...\n');

% 状態変数の定義: X = [x; x_dot; theta; theta_dot; i]
% 入力: u = V
% 出力: y = [x; theta] (位置と角度を観測)

% 運動方程式を解く: q_ddot = M^(-1) * (F - C*q_dot - G)
% q = [x; theta], q_dot = [x_dot; theta_dot]

M_inv = inv(M_lin);
acceleration_eq = M_inv * (F_lin - C_lin * [x_dot_var; theta_dot_var] - G_lin);

x_ddot_lin = acceleration_eq(1);
theta_ddot_lin = acceleration_eq(2);

fprintf('線形化された加速度方程式:\n');
fprintf('x_ddot = %s\n', char(x_ddot_lin));
fprintf('theta_ddot = %s\n', char(theta_ddot_lin));
fprintf('\n');

% 状態方程式の係数抽出
% X_dot = A*X + B*u
% X = [x; x_dot; theta; theta_dot; i]

A_symbolic = sym(zeros(5, 5));
B_symbolic = sym(zeros(5, 1));

% 第1行: dx/dt = x_dot
A_symbolic(1, 2) = 1;

% 第2行: dx_dot/dt = x_ddot_lin
% x_dot_varの係数
coeff_x_dot = coeffs(collect(x_ddot_lin, x_dot_var), x_dot_var);
if length(coeff_x_dot) > 1
    A_symbolic(2, 2) = coeff_x_dot(end);
end

% theta_varの係数
coeff_theta = coeffs(collect(x_ddot_lin, theta_var), theta_var);
if length(coeff_theta) > 1
    A_symbolic(2, 3) = coeff_theta(end);
end

% theta_dot_varの係数
coeff_theta_dot = coeffs(collect(x_ddot_lin, theta_dot_var), theta_dot_var);
if length(coeff_theta_dot) > 1
    A_symbolic(2, 4) = coeff_theta_dot(end);
end

% i_varの係数
coeff_i = coeffs(collect(x_ddot_lin, i_var), i_var);
if length(coeff_i) > 1
    A_symbolic(2, 5) = coeff_i(end);
end

% 第3行: dtheta/dt = theta_dot
A_symbolic(3, 4) = 1;

% 第4行: dtheta_dot/dt = theta_ddot_lin
% x_dot_varの係数
coeff_x_dot = coeffs(collect(theta_ddot_lin, x_dot_var), x_dot_var);
if length(coeff_x_dot) > 1
    A_symbolic(4, 2) = coeff_x_dot(end);
end

% theta_varの係数
coeff_theta = coeffs(collect(theta_ddot_lin, theta_var), theta_var);
if length(coeff_theta) > 1
    A_symbolic(4, 3) = coeff_theta(end);
end

% theta_dot_varの係数
coeff_theta_dot = coeffs(collect(theta_ddot_lin, theta_dot_var), theta_dot_var);
if length(coeff_theta_dot) > 1
    A_symbolic(4, 4) = coeff_theta_dot(end);
end

% i_varの係数
coeff_i = coeffs(collect(theta_ddot_lin, i_var), i_var);
if length(coeff_i) > 1
    A_symbolic(4, 5) = coeff_i(end);
end

% 第5行: di/dt = i_dot_equation
A_symbolic(5, 2) = -Ke*G/(r*La);  % x_dot項（逆起電力）
A_symbolic(5, 5) = -Ra/La;        % i項（抵抗）

% B行列（入力電圧の影響）
B_symbolic(5, 1) = 1/La;  % 電圧は電流方程式にのみ影響

% C行列（出力方程式）: y = [x; theta]
C_symbolic = [1, 0, 0, 0, 0;      % x
              0, 0, 1, 0, 0];     % theta

% D行列（直達項）
D_symbolic = [0; 0];

%% より正確な係数抽出
fprintf('係数を正確に抽出中...\n');

% 手動で係数を計算
% M_inv を展開
M_det = det(M_lin);
M_inv_11 = M22_lin / M_det;
M_inv_12 = -M12_lin / M_det;
M_inv_21 = -M21_lin / M_det;
M_inv_22 = M11_lin / M_det;

fprintf('逆質量行列の要素:\n');
fprintf('M_inv_11 = %s\n', char(simplify(M_inv_11)));
fprintf('M_inv_12 = %s\n', char(simplify(M_inv_12)));
fprintf('M_inv_21 = %s\n', char(simplify(M_inv_21)));
fprintf('M_inv_22 = %s\n', char(simplify(M_inv_22)));
fprintf('\n');

% 加速度方程式を手動で構築
% [x_ddot; theta_ddot] = M_inv * ([F1; F2] - [bx*x_dot; btheta*theta_dot] - [0; -m*g*l*theta])

eq1 = M_inv_11 * (F1 - bx*x_dot_var) + M_inv_12 * (-btheta*theta_dot_var + m*g*l*theta_var);
eq2 = M_inv_21 * (F1 - bx*x_dot_var) + M_inv_22 * (-btheta*theta_dot_var + m*g*l*theta_var);

x_ddot_final = simplify(eq1);
theta_ddot_final = simplify(eq2);

fprintf('最終的な加速度方程式:\n');
fprintf('x_ddot = %s\n', char(x_ddot_final));
fprintf('theta_ddot = %s\n', char(theta_ddot_final));
fprintf('\n');

% A行列を正確に構築
A_final = sym(zeros(5, 5));

% 第1行
A_final(1, 2) = 1;

% 第2行の係数を安全に抽出
A_final(2, 2) = get_coefficient(x_ddot_final, x_dot_var);
A_final(2, 3) = get_coefficient(x_ddot_final, theta_var);  
A_final(2, 4) = get_coefficient(x_ddot_final, theta_dot_var);
A_final(2, 5) = get_coefficient(x_ddot_final, i_var);

% 第3行
A_final(3, 4) = 1;

% 第4行の係数を安全に抽出
A_final(4, 2) = get_coefficient(theta_ddot_final, x_dot_var);
A_final(4, 3) = get_coefficient(theta_ddot_final, theta_var);
A_final(4, 4) = get_coefficient(theta_ddot_final, theta_dot_var);
A_final(4, 5) = get_coefficient(theta_ddot_final, i_var);

% 第5行（電気方程式）
A_final(5, 2) = -Ke*G/(r*La);
A_final(5, 5) = -Ra/La;

% B行列
B_final = [0; 0; 0; 0; 1/La];

%% 結果の表示
fprintf('\n=== シンボリック状態空間モデル ===\n');
fprintf('状態変数: X = [x; x_dot; theta; theta_dot; i]\n');
fprintf('入力: u = V (電圧 [V])\n');
fprintf('出力: y = [x; theta] (位置 [m], 角度 [rad])\n\n');

fprintf('状態方程式: X_dot = A*X + B*u\n');
fprintf('出力方程式: y = C*X + D*u\n\n');

fprintf('A行列 (5x5):\n');
for i = 1:5
    fprintf('  [');
    for j = 1:5
        if j < 5
            fprintf('%s, ', char(simplify(A_final(i,j))));
        else
            fprintf('%s', char(simplify(A_final(i,j))));
        end
    end
    fprintf(']\n');
end

fprintf('\nB行列 (5x1):\n');
for i = 1:5
    fprintf('  %s\n', char(B_final(i)));
end

fprintf('\nC行列 (2x5):\n');
fprintf('  [1, 0, 0, 0, 0]\n');
fprintf('  [0, 0, 1, 0, 0]\n');

fprintf('\nD行列 (2x1):\n');
fprintf('  [0; 0]\n');

%% LaTeX形式での出力
fprintf('\n=== LaTeX形式の状態方程式 ===\n');
fprintf('\\dot{X} = AX + Bu\n');
fprintf('y = CX + Du\n\n');

fprintf('ここで、\n');
fprintf('$X = \\begin{bmatrix} x \\\\ \\dot{x} \\\\ \\theta \\\\ \\dot{\\theta} \\\\ i \\end{bmatrix}$, ');
fprintf('$u = V$, ');
fprintf('$y = \\begin{bmatrix} x \\\\ \\theta \\end{bmatrix}$\n\n');

% LaTeX形式でA行列を出力
fprintf('$A = \\begin{bmatrix}\n');
for i = 1:5
    fprintf('  ');
    for j = 1:5
        latex_expr = latex(simplify(A_final(i,j)));
        if j < 5
            fprintf('%s & ', latex_expr);
        else
            fprintf('%s', latex_expr);
        end
    end
    if i < 5
        fprintf(' \\\\\n');
    else
        fprintf('\n');
    end
end
fprintf('\\end{bmatrix}$\n\n');

fprintf('$B = \\begin{bmatrix}\n');
for i = 1:5
    fprintf('  %s', latex(B_final(i)));
    if i < 5
        fprintf(' \\\\');
    end
    fprintf('\n');
end
fprintf('\\end{bmatrix}$\n\n');

%% 結果の保存
% シンボリック行列をMATファイルに保存
save('calculate_equation_of_motion/symbolic_pendulum_model.mat', 'A_final', 'B_final', 'C_symbolic', 'D_symbolic');

% SimulinkのParameter Estimator用にワークスペースに変数を作成 
A_symbolic_matrix = A_final;
B_symbolic_matrix = B_final;
C_symbolic_matrix = C_symbolic;
D_symbolic_matrix = D_symbolic;

% パラメータリストも作成
parameter_list = [M, m, g, l, r, Iw, n, G, Jp, bx, btheta, Kt, Ke, Ra, La];
parameter_names = {'M', 'm', 'g', 'l', 'r', 'Iw', 'n', 'G', 'Jp', 'bx', 'btheta', 'Kt', 'Ke', 'Ra', 'La'};

fprintf('シンボリック状態空間モデルが以下の変数に保存されました:\n');
fprintf('- A_symbolic_matrix (5x5 シンボリック行列)\n');
fprintf('- B_symbolic_matrix (5x1 シンボリック行列)\n');
fprintf('- C_symbolic_matrix (2x5 シンボリック行列)\n');
fprintf('- D_symbolic_matrix (2x1 シンボリック行列)\n');
fprintf('- parameter_list (パラメータのシンボリック配列)\n');
fprintf('- parameter_names (パラメータ名の文字列配列)\n');

fprintf('\nParameter Estimatorでの使用方法:\n');
fprintf('1. このスクリプトを実行してシンボリック行列を生成\n');
fprintf('2. SimulinkでState-Space blockを使用\n');
fprintf('3. パラメータをTunable parametersとして設定\n');
fprintf('4. Parameter Estimatorでパラメータ同定を実行\n');

fprintf('\nプログラム完了。\n');

%% 補助関数: 安全な係数抽出（簡潔で確実な実装）
function coeff = get_coefficient(expr, var)
    % シンボリック式から指定された変数の1次の係数を安全に抽出
    try
        % 式を展開
        expr_expanded = expand(expr);
        
        % 1次の係数を微分で求める（最も確実な方法）
        coeff = diff(expr_expanded, var);
        
        % 結果の検証
        if isa(coeff, 'sym')
            % 正常なシンボリック式
            coeff = simplify(coeff);
        else
            % 予期しない型の場合は0
            coeff = sym(0);
        end
        
    catch ME
        % エラーが発生した場合
        fprintf('警告: 変数 %s の係数抽出でエラー: %s\n', char(var), ME.message);
        coeff = sym(0);
    end
end