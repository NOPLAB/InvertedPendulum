function [A, B, C, D] = pendulum_state_space(p_M, p_m, p_g, p_l, p_r, p_Iw, p_n, p_G, p_Jp, p_bx, p_btheta, p_Kt, p_Ke, p_Ra, p_La)
% 台車型倒立振子の状態空間表現（構造体パラメータ版）
% 
% 状態方程式: dx/dt = A*x + B*u
% 出力方程式: y = C*x + D*u
%
% 状態変数: X = [x; x_dot; theta; theta_dot; i]
% 入力: u = V (電圧)
% 出力: y = [x; theta] (位置, 角度)
%
% 入力:
%   X: 状態ベクトル [x; x_dot; theta; theta_dot; i] (5x1)
%   params: パラメータ構造体
%     params.mechanical: 機械パラメータ構造体
%       .M: 台車質量 [kg]
%       .m: 振子質量 [kg]
%       .g: 重力加速度 [m/s^2]
%       .l: 振子長さ [m]
%       .r: タイヤ半径 [m]
%       .Iw: タイヤ慣性モーメント [kg*m^2]
%       .n: タイヤ数 [-]
%       .G: 減速比 [-]
%       .Jp: 振子慣性モーメント [kg*m^2]
%     params.friction: 摩擦パラメータ構造体
%       .bx: 台車摩擦係数 [N*s/m]
%       .btheta: 振子摩擦係数 [N*m*s/rad]
%     params.motor: モータパラメータ構造体
%       .Kt: トルク定数 [N*m/A]
%       .Ke: 逆起電力定数 [V*s/rad]
%       .Ra: 電機子抵抗 [Ohm]
%       .La: 電機子インダクタンス [H]
%
% 出力:
%   A: システム行列 (5x5)
%   B: 入力行列 (5x1)
%   C: 出力行列 (2x5)
%   D: 直達行列 (2x1)

%% パラメータの展開
% 機械パラメータ
M = p_M;
m = p_m;
g = p_g;
l = p_l;
r = p_r;
Iw = p_Iw;
n = p_n;
G = p_G;
Jp = p_Jp;

% 摩擦パラメータ
bx = p_bx;
btheta = p_btheta;

% モータパラメータ
Kt = p_Kt;
Ke = p_Ke;
Ra = p_Ra;
La = p_La;

%% パラメータの妥当性チェック
if M <= 0 || m <= 0 || l <= 0 || r <= 0 || La <= 0
    error('Physical parameters must be positive');
end

if n <= 0 || G <= 0
    error('Gear parameters must be positive');
end

%% 現在の状態から角度を取得（線形化点の決定）
% 平衡点での線形化（theta = 0）
theta = 0;

%% 線形化された質量行列の計算
% M(theta) = [M11, M12; M21, M22]
% 平衡点(theta=0)での線形化: cos(0)=1, sin(0)=0

M11 = M + n*Iw/(r^2) + m;
M12 = m*l;  % cos(theta) = cos(0) = 1
M21 = m*l;  % cos(theta) = cos(0) = 1  
M22 = Jp + m*l^2;

% 質量行列の逆行列を解析的に計算
det_M = M11*M22 - M12*M21;

% 数値的安定性のチェック
if abs(det_M) < 1e-10
    error('Mass matrix is singular. Check parameters.');
end

M_inv_11 = M22 / det_M;
M_inv_12 = -M12 / det_M;
M_inv_21 = -M21 / det_M;
M_inv_22 = M11 / det_M;

%% 線形化された減衰・コリオリ行列
% 平衡点での線形化により、コリオリ項 -ml*sin(theta)*theta_dot ≈ 0
C11 = bx;
C12 = 0;    % コリオリ項は線形化により0
C21 = 0;
C22 = btheta;

%% 線形化された重力項
% G = [0; -mgl*sin(theta)] ≈ [0; -mgl*theta] (平衡点での線形化)
G1 = 0;
G2_coeff = -m*g*l;  % theta項の係数

%% 入力項（モータ駆動力）
F1_coeff = n*G*Kt/r;  % 電流iに対する係数
F2_coeff = 0;

%% A行列の構築 (5x5)
A = zeros(5, 5);

% 第1行: dx/dt = x_dot
A(1, 2) = 1;

% 第2行: dx_dot/dt = M_inv_11*(-bx*x_dot + n*G*Kt*i/r) + M_inv_12*(-btheta*theta_dot - mgl*theta)
A(2, 2) = -M_inv_11 * bx;                    % x_dot項
A(2, 3) = M_inv_12 * G2_coeff;               % theta項  
A(2, 4) = -M_inv_12 * btheta;                % theta_dot項
A(2, 5) = M_inv_11 * F1_coeff;               % i項

% 第3行: dtheta/dt = theta_dot  
A(3, 4) = 1;

% 第4行: dtheta_dot/dt = M_inv_21*(-bx*x_dot + n*G*Kt*i/r) + M_inv_22*(-btheta*theta_dot - mgl*theta)
A(4, 2) = -M_inv_21 * bx;                    % x_dot項
A(4, 3) = M_inv_22 * G2_coeff;               % theta項
A(4, 4) = -M_inv_22 * btheta;                % theta_dot項  
A(4, 5) = M_inv_21 * F1_coeff;               % i項

% 第5行: di/dt = (-Ra*i - Ke*G*x_dot/r + V)/La
A(5, 2) = -Ke*G/(r*La);                      % x_dot項（逆起電力）
A(5, 5) = -Ra/La;                            % i項（抵抗）

%% B行列の構築 (5x1)
B = zeros(5, 1);
B(5, 1) = 1/La;  % 電圧は電流方程式にのみ影響

%% C行列の構築 (2x5) - 出力: [x; theta]
C = zeros(2, 5);
C(1, 1) = 1;  % x出力
C(2, 3) = 1;  % theta出力

%% D行列の構築 (2x1) - 直達項なし
D = zeros(2, 1);

end