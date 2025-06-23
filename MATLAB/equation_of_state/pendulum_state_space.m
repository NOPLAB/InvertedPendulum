function [A, B, C, D] = pendulum_state_space(p_M, p_m, p_g, p_l, p_r, p_Iw, p_n, p_G, p_Jp, p_bx, p_btheta, p_Kt, p_Ke, p_Ra, p_La)
% 台車型倒立振子の状態空間表現（上向き基準）
%
% 状態方程式: dx/dt = A*x + B*u
% 出力方程式: y = C*x + D*u
%
% 状態変数: X = [x; x_dot; phi; phi_dot; i]
% 入力: u = V (電圧)
% 出力: y = [x; phi] (位置, 上向きからの角度)
%
% 入力パラメータ:
% p_M: 台車質量 [kg]
% p_m: 振子質量 [kg]
% p_g: 重力加速度 [m/s^2]
% p_l: 振子長さ（回転軸から重心まで） [m]
% p_r: タイヤ半径 [m]
% p_Iw: タイヤ慣性モーメント [kg*m^2]
% p_n: タイヤ数 [-]
% p_G: 減速比 [-]
% p_Jp: 振子慣性モーメント（回転軸周り） [kg*m^2]
% p_bx: 台車の粘性摩擦係数 [N*s/m]
% p_btheta: 振子の粘性摩擦係数 [N*m*s/rad]
% p_Kt: トルク定数 [N*m/A]
% p_Ke: 逆起電力定数 [V*s/rad]
% p_Ra: 電機子抵抗 [Ohm]
% p_La: 電機子インダクタンス [H]
%
% 出力:
% A: システム行列 (5x5)
% B: 入力行列 (5x1)
% C: 出力行列 (2x5)
% D: 直達行列 (2x1)

matrix_params = [p_M; p_m; p_g; p_l; p_r; p_Iw; p_n; p_G; p_Jp; p_bx; p_btheta; p_Kt; p_Ke; p_Ra; p_La];

% パラメータ値の代入
A = linear_A_matrix(matrix_params);
B = linear_B_matrix(matrix_params);

% C行列（出力: y = [x; phi]）
C = [1, 0, 0, 0, 0;
     0, 0, 1, 0, 0];

% D行列
D = [0; 0];

end