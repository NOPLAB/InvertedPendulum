function [A, B, C, D, sys] = inverted_pendulum_state_equation(M, m, g, l, Jp, bx, btheta)
% 倒立振子の状態方程式を導出する関数
% 
% 入力パラメータ:
% M: 台車質量 [kg]
% m: 振子質量 [kg]
% g: 重力加速度 [m/s^2]
% l: 振子長さ [m]
% Jp: 振子慣性モーメント [kg*m^2]
% bx: 台車の粘性摩擦係数 [N*s/m]
% btheta: 振子の粘性摩擦係数 [N*m*s/rad]
%
% 出力:
% A, B, C, D: 状態空間表現の行列
% sys: 状態空間システム

fprintf('=== 倒立振子の状態方程式導出 ===\n\n');

% パラメータ表示
fprintf('システムパラメータ:\n');
fprintf('台車質量 M = %.3f [kg]\n', M);
fprintf('振子質量 m = %.3f [kg]\n', m);
fprintf('重力加速度 g = %.3f [m/s^2]\n', g);
fprintf('振子長さ l = %.3f [m]\n', l);
fprintf('振子慣性モーメント Jp = %.6f [kg*m^2]\n', Jp);
fprintf('台車粘性摩擦係数 bx = %.6f [N*s/m]\n', bx);
fprintf('振子粘性摩擦係数 btheta = %.6f [N*m*s/rad]\n', btheta);
fprintf('\n');

% 状態変数の定義
fprintf('状態変数の定義:\n');
fprintf('x1 = x   (台車位置)\n');
fprintf('x2 = dx  (台車速度)\n');
fprintf('x3 = θ   (振子角度)\n');
fprintf('x4 = dθ  (振子角速度)\n');
fprintf('u = F    (台車への入力力)\n\n');

% 運動方程式の導出過程
fprintf('=== 運動方程式の導出 ===\n\n');

fprintf('1. ラグランジュ法による運動方程式:\n');
fprintf('台車の運動方程式:\n');
fprintf('(M + m)ẍ + mlθ̈cos(θ) - mlθ̇²sin(θ) + bxẋ = F\n\n');

fprintf('振子の運動方程式:\n');
fprintf('(Jp + ml²)θ̈ + mlẍcos(θ) - mglsin(θ) + bθθ̇ = 0\n\n');

% 線形化（小角度近似）
fprintf('2. 小角度近似による線形化:\n');
fprintf('sin(θ) ≈ θ, cos(θ) ≈ 1, θ̇² ≈ 0\n\n');

fprintf('線形化された運動方程式:\n');
fprintf('(M + m)ẍ + mlθ̈ + bxẋ = F\n');
fprintf('(Jp + ml²)θ̈ + mlẍ - mglθ + bθθ̇ = 0\n\n');

% 状態方程式の導出
fprintf('=== 状態方程式の導出 ===\n\n');

% 係数行列の計算
fprintf('3. 運動方程式を行列形式で表現:\n');
fprintf('M_matrix * [ẍ; θ̈] + B_matrix * [ẋ; θ̇] + K_matrix * [x; θ] = [F; 0]\n\n');

% 質量行列
M_matrix = [M + m, m*l; m*l, Jp + m*l^2];
fprintf('質量行列 M_matrix:\n');
disp(M_matrix);

% 粘性行列
B_matrix = [bx, 0; 0, btheta];
fprintf('粘性行列 B_matrix:\n');
disp(B_matrix);

% 剛性行列
K_matrix = [0, 0; 0, -m*g*l];
fprintf('剛性行列 K_matrix:\n');
disp(K_matrix);

% 入力行列
F_matrix = [1; 0];
fprintf('入力行列 F_matrix:\n');
disp(F_matrix);

% 状態空間表現の導出
fprintf('4. 状態空間表現 ẋ = Ax + Bu の導出:\n\n');

% 質量行列の逆行列
M_inv = inv(M_matrix);
fprintf('質量行列の逆行列 M_inv:\n');
disp(M_inv);

% 状態行列Aの計算
A = zeros(4, 4);
A(1, 2) = 1;  % dx/dt = ẋ
A(3, 4) = 1;  % dθ/dt = θ̇

% 台車の加速度方程式: ẍ = -M_inv(1,:) * K_matrix * [x; θ] - M_inv(1,:) * B_matrix * [ẋ; θ̇]
A(2, 1) = -M_inv(1,1) * K_matrix(1,1) - M_inv(1,2) * K_matrix(2,1);  % x項
A(2, 2) = -M_inv(1,1) * B_matrix(1,1) - M_inv(1,2) * B_matrix(2,1);  % ẋ項
A(2, 3) = -M_inv(1,1) * K_matrix(1,2) - M_inv(1,2) * K_matrix(2,2);  % θ項
A(2, 4) = -M_inv(1,1) * B_matrix(1,2) - M_inv(1,2) * B_matrix(2,2);  % θ̇項

% 振子の角加速度方程式: θ̈ = -M_inv(2,:) * K_matrix * [x; θ] - M_inv(2,:) * B_matrix * [ẋ; θ̇]
A(4, 1) = -M_inv(2,1) * K_matrix(1,1) - M_inv(2,2) * K_matrix(2,1);  % x項
A(4, 2) = -M_inv(2,1) * B_matrix(1,1) - M_inv(2,2) * B_matrix(2,1);  % ẋ項
A(4, 3) = -M_inv(2,1) * K_matrix(1,2) - M_inv(2,2) * K_matrix(2,2);  % θ項
A(4, 4) = -M_inv(2,1) * B_matrix(1,2) - M_inv(2,2) * B_matrix(2,2);  % θ̇項

% より詳細な計算過程
fprintf('A行列の各要素の計算:\n');
fprintf('A(1,2) = 1 (dx/dt = ẋ)\n');
fprintf('A(3,4) = 1 (dθ/dt = θ̇)\n\n');

fprintf('台車の加速度方程式から:\n');
fprintf('A(2,1) = %.6f\n', A(2,1));
fprintf('A(2,2) = %.6f\n', A(2,2));
fprintf('A(2,3) = %.6f\n', A(2,3));
fprintf('A(2,4) = %.6f\n', A(2,4));
fprintf('\n');

fprintf('振子の角加速度方程式から:\n');
fprintf('A(4,1) = %.6f\n', A(4,1));
fprintf('A(4,2) = %.6f\n', A(4,2));
fprintf('A(4,3) = %.6f\n', A(4,3));
fprintf('A(4,4) = %.6f\n', A(4,4));
fprintf('\n');

% 入力行列Bの計算
B = zeros(4, 1);
B(2) = M_inv(1,1) * F_matrix(1) + M_inv(1,2) * F_matrix(2);
B(4) = M_inv(2,1) * F_matrix(1) + M_inv(2,2) * F_matrix(2);

fprintf('B行列の計算:\n');
fprintf('B(2) = %.6f\n', B(2));
fprintf('B(4) = %.6f\n', B(4));
fprintf('\n');

% 出力行列（台車位置と振子角度を出力とする）
C = [1, 0, 0, 0;   % 台車位置
     0, 0, 1, 0];  % 振子角度
D = [0; 0];

fprintf('=== 最終的な状態空間表現 ===\n\n');
fprintf('ẋ = Ax + Bu\n');
fprintf('y = Cx + Du\n\n');

fprintf('A行列 (状態行列):\n');
fprintf('A = \n');
for i = 1:4
    fprintf('[');
    for j = 1:4
        fprintf('%10.6f', A(i,j));
        if j < 4, fprintf(', '); end
    end
    fprintf(']\n');
end
fprintf('\n');

fprintf('B行列 (入力行列):\n');
fprintf('B = [%.6f; %.6f; %.6f; %.6f]\n\n', B(1), B(2), B(3), B(4));

fprintf('C行列 (出力行列):\n');
fprintf('C = \n');
for i = 1:2
    fprintf('[');
    for j = 1:4
        fprintf('%10.6f', C(i,j));
        if j < 4, fprintf(', '); end
    end
    fprintf(']\n');
end
fprintf('\n');

fprintf('D行列 (直達項):\n');
fprintf('D = [%.6f; %.6f]\n\n', D(1), D(2));

% 状態空間システムの作成
sys = ss(A, B, C, D);
fprintf('状態空間システムが作成されました。\n');

% 安定性の確認
eigenvalues = eig(A);
fprintf('\n=== システムの安定性解析 ===\n');
fprintf('A行列の固有値:\n');
for i = 1:length(eigenvalues)
    if isreal(eigenvalues(i))
        fprintf('λ%d = %.6f\n', i, eigenvalues(i));
    else
        fprintf('λ%d = %.6f + %.6fi\n', i, real(eigenvalues(i)), imag(eigenvalues(i)));
    end
end

% 安定性の判定
if all(real(eigenvalues) < 0)
    fprintf('\nシステムは安定です（全ての固有値の実部が負）\n');
elseif any(real(eigenvalues) > 0)
    fprintf('\nシステムは不安定です（正の実部を持つ固有値が存在）\n');
else
    fprintf('\nシステムは限界安定です\n');
end

% 可制御性の確認
Wc = ctrb(A, B);
rank_Wc = rank(Wc);
fprintf('\n=== 可制御性解析 ===\n');
fprintf('可制御性行列のランク: %d\n', rank_Wc);
fprintf('システムの次数: %d\n', size(A, 1));
if rank_Wc == size(A, 1)
    fprintf('システムは完全可制御です\n');
else
    fprintf('システムは可制御ではありません\n');
end

% 可観測性の確認
Wo = obsv(A, C);
rank_Wo = rank(Wo);
fprintf('\n=== 可観測性解析 ===\n');
fprintf('可観測性行列のランク: %d\n', rank_Wo);
if rank_Wo == size(A, 1)
    fprintf('システムは完全可観測です\n');
else
    fprintf('システムは可観測ではありません\n');
end

end