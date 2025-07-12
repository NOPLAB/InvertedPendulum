use crate::constants;
use micromath::F32Ext;

// 振子の物理パラメータ
#[derive(Debug, Clone, Copy)]
pub struct PendulumParams {
    pub length: f32,    // 振子の長さ [m]
    pub mass: f32,      // 振子の質量 [kg]
    pub gravity: f32,   // 重力加速度 [m/s²]
    pub damping: f32,   // 粘性抵抗係数 [N·m·s/rad]
    pub cart_mass: f32, // カートの質量 [kg]
}

impl Default for PendulumParams {
    fn default() -> Self {
        Self {
            length: 0.2,    // 20cm
            mass: 0.1,      // 100g
            gravity: 9.81,  // 標準重力
            damping: 0.01,  // 小さな粘性抵抗
            cart_mass: 1.0, // 1kg
        }
    }
}

// ルーエンベルガー観測器による振子角速度推定
#[derive(Debug)]
pub struct PendulumAngularVelocityObserver {
    params: PendulumParams,
    sample_time: f32,

    // 状態変数 [角度, 角速度]
    estimated_state: [f32; 2],

    // 観測器ゲイン
    observer_gain: [f32; 2],

    // 前回の制御入力
    prev_control_input: f32,
}

impl PendulumAngularVelocityObserver {
    pub fn new(params: PendulumParams, sample_time: f32) -> Self {
        // 観測器ゲインの設計（極配置法）
        // 連続時間での希望極: -10, -20（高速な収束）
        let desired_poles = [-10.0, -20.0];
        let observer_gain = Self::design_observer_gain(&params, &desired_poles);

        Self {
            params,
            sample_time,
            estimated_state: [0.0, 0.0],
            observer_gain,
            prev_control_input: 0.0,
        }
    }

    // 観測器ゲインの設計
    fn design_observer_gain(params: &PendulumParams, desired_poles: &[f32; 2]) -> [f32; 2] {
        let PendulumParams {
            length,
            mass,
            gravity,
            damping,
            cart_mass,
        } = params;

        // 線形化された振子の状態方程式のパラメータ
        let total_mass = cart_mass + mass;
        let a22 = -(damping) / (mass * length * length);
        let a21 = (mass * gravity * length) / (mass * length * length);

        // A行列 = [0, 1; a21, a22]
        // C行列 = [1, 0]（角度のみ観測可能）

        // 特性方程式: λ² - (a22 + L1)λ + (a21 - L2) = 0
        // 希望特性方程式: (λ + p1)(λ + p2) = λ² + (p1+p2)λ + p1*p2
        let p1 = -desired_poles[0];
        let p2 = -desired_poles[1];

        let l1 = -(a22 + p1 + p2);
        let l2 = a21 - p1 * p2;

        [l1, l2]
    }

    // 観測器の更新
    pub fn update(&mut self, measured_angle: f32, control_force: f32) -> f32 {
        let PendulumParams {
            length,
            mass,
            gravity,
            damping,
            cart_mass,
        } = &self.params;

        // 線形化された状態方程式のパラメータ
        let total_mass = cart_mass + mass;
        let a21 = (mass * gravity * length) / (mass * length * length);
        let a22 = -(damping) / (mass * length * length);
        let b2 = control_force / (mass * length * length);

        // 現在の推定状態
        let [theta_est, theta_dot_est] = self.estimated_state;

        // 観測誤差
        let observation_error = measured_angle - theta_est;

        // 状態方程式による予測
        // dx/dt = Ax + Bu
        // A = [0, 1; a21, a22], B = [0; b2]
        let theta_dot_pred = theta_dot_est + self.observer_gain[0] * observation_error;
        let theta_ddot_pred =
            a21 * theta_est + a22 * theta_dot_est + b2 + self.observer_gain[1] * observation_error;

        // オイラー法による数値積分
        self.estimated_state[0] = theta_est + theta_dot_pred * self.sample_time;
        self.estimated_state[1] = theta_dot_est + theta_ddot_pred * self.sample_time;

        self.prev_control_input = control_force;

        // 推定角速度を返す
        self.estimated_state[1]
    }

    // より高精度な観測器（ルンゲクッタ法）
    pub fn update_rk4(&mut self, measured_angle: f32, control_force: f32) -> f32 {
        let h = self.sample_time;
        let [theta_est, theta_dot_est] = self.estimated_state;

        // 観測誤差
        let observation_error = measured_angle - theta_est;

        // k1
        let k1_theta = theta_dot_est + self.observer_gain[0] * observation_error;
        let k1_theta_dot = self.state_derivative(theta_est, theta_dot_est, control_force)
            + self.observer_gain[1] * observation_error;

        // k2
        let theta_k2 = theta_est + 0.5 * h * k1_theta;
        let theta_dot_k2 = theta_dot_est + 0.5 * h * k1_theta_dot;
        let error_k2 = measured_angle - theta_k2;
        let k2_theta = theta_dot_k2 + self.observer_gain[0] * error_k2;
        let k2_theta_dot = self.state_derivative(theta_k2, theta_dot_k2, control_force)
            + self.observer_gain[1] * error_k2;

        // k3
        let theta_k3 = theta_est + 0.5 * h * k2_theta;
        let theta_dot_k3 = theta_dot_est + 0.5 * h * k2_theta_dot;
        let error_k3 = measured_angle - theta_k3;
        let k3_theta = theta_dot_k3 + self.observer_gain[0] * error_k3;
        let k3_theta_dot = self.state_derivative(theta_k3, theta_dot_k3, control_force)
            + self.observer_gain[1] * error_k3;

        // k4
        let theta_k4 = theta_est + h * k3_theta;
        let theta_dot_k4 = theta_dot_est + h * k3_theta_dot;
        let error_k4 = measured_angle - theta_k4;
        let k4_theta = theta_dot_k4 + self.observer_gain[0] * error_k4;
        let k4_theta_dot = self.state_derivative(theta_k4, theta_dot_k4, control_force)
            + self.observer_gain[1] * error_k4;

        // 状態更新
        self.estimated_state[0] =
            theta_est + (h / 6.0) * (k1_theta + 2.0 * k2_theta + 2.0 * k3_theta + k4_theta);
        self.estimated_state[1] = theta_dot_est
            + (h / 6.0) * (k1_theta_dot + 2.0 * k2_theta_dot + 2.0 * k3_theta_dot + k4_theta_dot);

        self.prev_control_input = control_force;

        // 推定角速度を返す
        self.estimated_state[1]
    }

    // 状態微分の計算
    fn state_derivative(&self, theta: f32, theta_dot: f32, control_force: f32) -> f32 {
        let PendulumParams {
            length,
            mass,
            gravity,
            damping,
            cart_mass: _,
        } = &self.params;

        let ml2 = mass * length * length;
        let mgl = mass * gravity * length;

        // 非線形モデル: θ'' = (g/l)sin(θ) - (d/ml²)θ' + (F/ml²)
        (gravity / length) * theta.sin() - (damping / ml2) * theta_dot + control_force / ml2
    }

    // 現在の推定状態を取得
    pub fn get_estimated_state(&self) -> (f32, f32) {
        (self.estimated_state[0], self.estimated_state[1])
    }

    // 推定角速度を取得
    pub fn get_estimated_angular_velocity(&self) -> f32 {
        self.estimated_state[1]
    }

    // 観測器をリセット
    pub fn reset(&mut self) {
        self.estimated_state = [0.0, 0.0];
        self.prev_control_input = 0.0;
    }

    // 観測器ゲインを設定
    pub fn set_observer_gain(&mut self, l1: f32, l2: f32) {
        self.observer_gain = [l1, l2];
    }

    // パラメータを更新
    pub fn update_parameters(&mut self, params: PendulumParams) {
        // 新しいパラメータで観測器ゲインを再設計
        let desired_poles = [-10.0, -20.0];
        self.observer_gain = Self::design_observer_gain(&params, &desired_poles);
        self.params = params;
    }
}

// カルマンフィルタベースの観測器（拡張版）
pub struct ExtendedKalmanObserver {
    params: PendulumParams,
    sample_time: f32,

    // 状態変数 [角度, 角速度]
    state: [f32; 2],

    // 誤差共分散行列 P (2x2)
    covariance: [[f32; 2]; 2],

    // プロセスノイズ共分散 Q
    process_noise: [[f32; 2]; 2],

    // 観測ノイズ分散 R
    measurement_noise: f32,
}

impl ExtendedKalmanObserver {
    pub fn new(params: PendulumParams, sample_time: f32) -> Self {
        Self {
            params,
            sample_time,
            state: [0.0, 0.0],
            covariance: [[1.0, 0.0], [0.0, 1.0]], // 初期共分散
            process_noise: [[0.01, 0.0], [0.0, 0.1]], // プロセスノイズ
            measurement_noise: 0.001,             // 角度センサーノイズ
        }
    }

    pub fn update(&mut self, measured_angle: f32, control_force: f32) -> f32 {
        // 予測ステップ
        self.predict(control_force);

        // 更新ステップ
        self.correct(measured_angle);

        // 推定角速度を返す
        self.state[1]
    }

    fn predict(&mut self, control_force: f32) {
        let dt = self.sample_time;
        let [theta, theta_dot] = self.state;

        // 状態予測
        let theta_pred = theta + theta_dot * dt;
        let theta_dot_pred =
            theta_dot + self.state_derivative(theta, theta_dot, control_force) * dt;

        self.state = [theta_pred, theta_dot_pred];

        // ヤコビアン行列 F の計算
        let f11 = 1.0;
        let f12 = dt;
        let f21 = dt * (self.params.gravity / self.params.length) * theta.cos(); // ∂f/∂θ
        let f22 = 1.0
            - dt * (self.params.damping
                / (self.params.mass * self.params.length * self.params.length));

        let jacobian_f = [[f11, f12], [f21, f22]];

        // 共分散予測: P = F*P*F' + Q
        self.covariance = self.matrix_add_2x2(
            self.matrix_multiply_2x2(
                self.matrix_multiply_2x2(jacobian_f, self.covariance),
                self.matrix_transpose_2x2(jacobian_f),
            ),
            self.process_noise,
        );
    }

    fn correct(&mut self, measurement: f32) {
        // 観測行列 H = [1, 0]
        let h = [1.0, 0.0];

        // イノベーション
        let innovation = measurement - self.state[0];

        // イノベーション共分散 S = H*P*H' + R
        let s = self.covariance[0][0] + self.measurement_noise;

        // カルマンゲイン K = P*H'/S
        let k1 = self.covariance[0][0] / s;
        let k2 = self.covariance[1][0] / s;

        // 状態更新
        self.state[0] += k1 * innovation;
        self.state[1] += k2 * innovation;

        // 共分散更新: P = (I - K*H)*P
        let i_kh = [[1.0 - k1, 0.0], [-k2, 1.0]];
        self.covariance = self.matrix_multiply_2x2(i_kh, self.covariance);
    }

    fn state_derivative(&self, theta: f32, theta_dot: f32, control_force: f32) -> f32 {
        let PendulumParams {
            length,
            mass,
            gravity,
            damping,
            cart_mass: _,
        } = &self.params;

        let ml2 = mass * length * length;
        let mgl = mass * gravity * length;

        (gravity / length) * theta.sin() - (damping / ml2) * theta_dot + control_force / ml2
    }

    // 行列演算ヘルパー関数
    fn matrix_multiply_2x2(&self, a: [[f32; 2]; 2], b: [[f32; 2]; 2]) -> [[f32; 2]; 2] {
        [
            [
                a[0][0] * b[0][0] + a[0][1] * b[1][0],
                a[0][0] * b[0][1] + a[0][1] * b[1][1],
            ],
            [
                a[1][0] * b[0][0] + a[1][1] * b[1][0],
                a[1][0] * b[0][1] + a[1][1] * b[1][1],
            ],
        ]
    }

    fn matrix_transpose_2x2(&self, a: [[f32; 2]; 2]) -> [[f32; 2]; 2] {
        [[a[0][0], a[1][0]], [a[0][1], a[1][1]]]
    }

    fn matrix_add_2x2(&self, a: [[f32; 2]; 2], b: [[f32; 2]; 2]) -> [[f32; 2]; 2] {
        [
            [a[0][0] + b[0][0], a[0][1] + b[0][1]],
            [a[1][0] + b[1][0], a[1][1] + b[1][1]],
        ]
    }

    pub fn get_estimated_angular_velocity(&self) -> f32 {
        self.state[1]
    }

    pub fn reset(&mut self) {
        self.state = [0.0, 0.0];
        self.covariance = [[1.0, 0.0], [0.0, 1.0]];
    }
}
