//! AdaptiveLineController - 倒立振子用ライントレースコントローラ
//! 振子安定化（縦方向）とライン追従（横方向）を同時制御

use crate::constants::*;
use crate::controller::{ControlReferences, ControllerError, HighLevelController};
use crate::fmt::info;
use crate::line_controller::{LineController, LineControllerError};
use crate::lpf::LowPassFilter;
use crate::mit_adaptive_controller::MitAdaptiveController;
use crate::sensor::SensorManager;

/// 倒立振子用アダプティブライン追跡コントローラ
///
/// 制御戦略：
/// 1. 縦方向（前後）: MITアダプティブコントローラで振子安定化
/// 2. 横方向（左右）: ライン追従のための左右モーター差分制御
#[derive(Debug)]
pub struct AdaptiveLineController {
    // ライン追跡
    line_controller: LineController,

    // ペンデュラム制御（MITアダプティブ - 前後制御用）
    adaptive_controller: MitAdaptiveController,

    // フィルタリング
    theta_filter: LowPassFilter,
    prev_filtered_theta: f32,
    theta_velocity: f32,

    // 前進制御
    target_velocity: f32,     // 目標前進速度 [m/s]
    position_reference: f32,  // 動的位置リファレンス [m]
    target_theta_offset: f32, // 目標振子角度オフセット [rad] (正=前傾)
}

impl AdaptiveLineController {
    /// 新しいAdaptiveLineControllerを作成
    pub fn new(sample_time: f32) -> Self {
        let mut adaptive_controller = MitAdaptiveController::new(sample_time);

        // 初期ゲインを設定（LQRゲインベース）
        adaptive_controller.set_initial_gains(K_POSITION, K_VELOCITY, K_ANGLE, K_ANGULAR_VELOCITY);

        // リファレンスモデルを設定（倒立振子用）
        adaptive_controller.set_reference_model(10.0, 0.7);

        Self {
            line_controller: LineController::new(),
            adaptive_controller,
            theta_filter: LowPassFilter::new(sample_time, THETA_FILTER_CUTOFF_FREQ, 1.0),
            prev_filtered_theta: 0.0,
            theta_velocity: 0.0,
            target_velocity: 0.01,    // 目標前進速度 [m/s]
            position_reference: 0.0,  // 初期位置リファレンス
            target_theta_offset: 0.0, // 目標前傾角度 [rad]
        }
    }

    /// ラインコントローラのPIDゲインを設定
    pub fn set_line_pid_gains(&mut self, kp: f32, ki: f32, kd: f32) {
        self.line_controller.set_pid_gains(kp, ki, kd);
    }

    /// アダプティブコントローラの初期ゲインを設定
    pub fn set_adaptive_gains(&mut self, k_pos: f32, k_vel: f32, k_angle: f32, k_angular_vel: f32) {
        self.adaptive_controller
            .set_initial_gains(k_pos, k_vel, k_angle, k_angular_vel);
    }

    /// 目標前進速度を設定
    pub fn set_target_velocity(&mut self, velocity: f32) {
        self.target_velocity = velocity;
    }

    /// 目標振子前傾角度を設定 [rad] (正=前傾)
    pub fn set_target_theta_offset(&mut self, theta_offset: f32) {
        self.target_theta_offset = theta_offset;
    }

    /// ライン検出状態を取得
    pub fn is_line_detected(&self) -> bool {
        self.line_controller.is_line_detected()
    }

    /// 現在のライン位置を取得
    pub fn get_line_position(&self) -> f32 {
        self.line_controller.get_line_position()
    }

    /// センサーデバッグ情報を取得
    pub fn get_sensor_debug_info(&self) -> [u16; 4] {
        self.line_controller.get_sensor_debug_info()
    }

    /// 倒立振子ライントレース制御出力を計算
    /// 戻り値：(左モーター力, 右モーター力)
    pub fn compute_control_outputs(
        &mut self,
        sensor_data: &SensorManager,
        references: &ControlReferences,
    ) -> Result<(f32, f32), ControllerError> {
        // 1. ライン位置を検出
        let line_follow = match self.line_controller.compute_position_command() {
            Ok(offset) => offset,
            Err(LineControllerError::NoLineDetected) => 0.0, // ライン未検出時は直進
            Err(_) => 0.0,                                   // その他のエラーも直進
        };

        // 2. 状態変数を準備（振子安定化用）
        let x = (sensor_data.position_r + sensor_data.position_l) / 2.0;
        let dx = (sensor_data.velocity_r + sensor_data.velocity_l) / 2.0;
        let theta = self.theta_filter.update(sensor_data.theta0);

        info!(
            "Line offset: {}, Position ref: {}, Position actual: {}, Theta: {}",
            line_follow, self.position_reference, x, theta
        );

        // 角速度を計算（フィルタ後のtheta微分）
        self.theta_velocity = (theta - self.prev_filtered_theta) / DT;
        self.prev_filtered_theta = theta;
        let dtheta = self.theta_velocity;

        // 3. 基本制御力を計算（振子安定化のみ - 位置基準は0に固定）
        let base_force = self.adaptive_controller.update(
            x, dx, theta, dtheta,
            0.0, // 位置リファレンスを0に固定して安定化のみ
        );

        // 4. 前進制御（前傾角度による）
        let theta_error = theta - self.target_theta_offset;
        let forward_bias = -theta_error * 1.0; // ゲイン減少

        // 5. ライン追従のための左右差分を計算
        let lateral_force = if self.line_controller.is_line_detected() {
            line_follow
        } else {
            0.0 // ライン未検出時は横方向制御なし
        };

        // 6. 移動方向の判定（速度ベース）
        let velocity = dx;
        let is_moving_backward = velocity < -0.01; // 後進判定閾値

        // 7. 左右モーターへの力配分
        let (force_left, force_right) = if is_moving_backward {
            // 後進時：横方向制御を逆転
            (
                base_force + forward_bias - lateral_force,
                base_force + forward_bias + lateral_force,
            )
        } else {
            // 前進時：通常の横方向制御
            (
                base_force + forward_bias + lateral_force,
                base_force + forward_bias - lateral_force,
            )
        };

        // 6. 制御力を制限
        let limited_force_left = clamp_f32(force_left, -MAX_FORCE, MAX_FORCE);
        let limited_force_right = clamp_f32(force_right, -MAX_FORCE, MAX_FORCE);

        Ok((limited_force_left, limited_force_right))
    }
}

impl HighLevelController for AdaptiveLineController {
    fn compute_control_force(
        &mut self,
        sensor_data: &SensorManager,
        _references: &ControlReferences,
    ) -> Result<f32, ControllerError> {
        // この実装は後方互換性のためのダミー実装
        // 実際の制御にはcompute_control_outputsを使用
        let (force_left, force_right) = self.compute_control_outputs(sensor_data, _references)?;

        // 平均値を返す（実際の制御では使用されない）
        Ok((force_left + force_right) / 2.0)
    }

    fn reset(&mut self) {
        self.line_controller.reset();
        self.adaptive_controller.reset();
        self.theta_filter.reset();
        self.prev_filtered_theta = 0.0;
        self.theta_velocity = 0.0;
        self.position_reference = 0.0; // 位置リファレンスもリセット（予備用）
    }

    fn set_parameters(&mut self, params: &[f32]) {
        // 最初の4つのパラメータ：アダプティブコントローラゲイン
        if params.len() >= 4 {
            self.adaptive_controller
                .set_initial_gains(params[0], params[1], params[2], params[3]);
        }

        // 次の3つのパラメータ：ラインPIDゲイン
        if params.len() >= 7 {
            self.line_controller
                .set_pid_gains(params[4], params[5], params[6]);
        }

        // 8番目のパラメータ：目標前進速度
        if params.len() >= 8 {
            self.target_velocity = params[7];
        }

        // 10番目のパラメータ：目標前傾角度
        if params.len() >= 9 {
            self.target_theta_offset = params[8];
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_adaptive_line_controller_creation() {
        let controller = AdaptiveLineController::new(0.0005);
        assert!(!controller.is_line_detected());
        assert_eq!(controller.get_line_position(), 0.0);
    }

    #[test]
    fn test_parameter_setting() {
        let mut controller = AdaptiveLineController::new(0.0005);
        let params = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0];
        controller.set_parameters(&params);

        // パラメータが正しく設定されたかを確認
        // 実際の実装では、これらの値が内部で正しく設定されているかを確認する必要があります
    }
}
