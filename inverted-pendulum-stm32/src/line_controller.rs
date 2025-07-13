//! 新しいLineController - multiplexer 0~4を使用したラインコントローラ
//! 0~1024が黒（地面）、それ以上がライン

use crate::constants::*;
use crate::sensor::adc::get_all_line_sensors;

/// ラインセンサー値の閾値定数
const LINE_THRESHOLD: u16 = 1000;

/// センサー位置 (メートル単位で左から右へ)
/// センサー0: 左端、センサー1: 左中、センサー2: 右中、センサー3: 右端
const SENSOR_POSITIONS: [f32; 4] = [-0.03, -0.01, 0.01, 0.03];

/// エラー処理
#[derive(Debug, Clone, Copy)]
pub enum LineControllerError {
    NoLineDetected,
    SensorError,
}

/// ライン追跡コントローラ
#[derive(Debug)]
pub struct LineController {
    // ライン検出状態
    line_position: f32,
    line_velocity: f32,
    prev_line_position: f32,
    line_detected: bool,

    // PIDコントローラパラメータ
    kp: f32,
    ki: f32,
    kd: f32,

    // PID状態
    error_integral: f32,
    prev_error: f32,

    // フィルタリング
    position_filter_alpha: f32,
}

impl LineController {
    /// 新しいLineControllerを作成
    pub fn new() -> Self {
        Self {
            line_position: 0.0,
            line_velocity: 0.0,
            prev_line_position: 0.0,
            line_detected: false,

            // PIDゲイン（チューニング可能）
            kp: 80.0,
            ki: 0.5,
            kd: 1.2,

            // PID状態
            error_integral: 0.0,
            prev_error: 0.0,

            // フィルタ係数（20Hz カットオフ）
            position_filter_alpha: calculate_alpha(20.0, DT),
        }
    }

    /// PIDゲインを設定
    pub fn set_pid_gains(&mut self, kp: f32, ki: f32, kd: f32) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    /// ライン位置を検出（マルチプレクサーセンサー0~3を使用）
    pub fn detect_line_position(&mut self) -> Result<f32, LineControllerError> {
        // ADCから4つのセンサー値を取得
        let sensor_values = get_all_line_sensors();

        // デバッグ情報（必要に応じてコメントアウト）
        /*
        crate::fmt::info!(
            "Sensors: [{}, {}, {}, {}]",
            sensor_values[0], sensor_values[1], sensor_values[2], sensor_values[3]
        );
        */

        let mut weighted_sum = 0.0;
        let mut total_weight = 0.0;
        let mut line_detected = false;

        // 各センサーでライン検出
        for (i, &sensor_value) in sensor_values.iter().enumerate() {
            if sensor_value > LINE_THRESHOLD {
                line_detected = true;
                // ライン強度を重みとして使用
                let weight = (sensor_value - LINE_THRESHOLD) as f32;
                weighted_sum += SENSOR_POSITIONS[i] * weight;
                total_weight += weight;
            }
        }

        // ライン位置を計算
        let raw_position = if line_detected && total_weight > 0.0 {
            weighted_sum / total_weight
        } else {
            return Err(LineControllerError::NoLineDetected);
        };

        // ローパスフィルタを適用
        let filtered_position = if self.line_detected {
            // 前回もライン検出していた場合、フィルタを適用
            self.line_position * (1.0 - self.position_filter_alpha)
                + raw_position * self.position_filter_alpha
        } else {
            // 初回検出の場合、フィルタなしで設定
            raw_position
        };

        // 速度を計算（フィルタ後の位置から）
        self.line_velocity = (filtered_position - self.prev_line_position) / DT;

        // 状態を更新
        self.prev_line_position = self.line_position;
        self.line_position = filtered_position;
        self.line_detected = line_detected;

        Ok(self.line_position)
    }

    /// ライン追跡のための位置コマンドを計算
    /// 戻り値：目標位置 [m] （アダプティブコントローラへの入力）
    pub fn compute_position_command(&mut self) -> Result<f32, LineControllerError> {
        // ライン位置を更新
        self.detect_line_position()?;

        // ライン中心からのエラー（目標は0.0）
        let error = -self.line_position; // ラインに向かって移動

        // 積分項（ワインドアップ保護付き）
        self.error_integral += error * DT;
        self.error_integral = clamp_f32(self.error_integral, -0.1, 0.1);

        // 微分項
        let error_derivative = (error - self.prev_error) / DT;
        self.prev_error = error;

        // PID計算
        let position_command =
            self.kp * error + self.ki * self.error_integral + self.kd * error_derivative;

        // 位置コマンドを制限（±0.5m）
        let limited_command = clamp_f32(position_command, -0.5, 0.5);

        Ok(limited_command)
    }

    /// 現在のライン検出状態を取得
    pub fn is_line_detected(&self) -> bool {
        self.line_detected
    }

    /// 現在のライン位置を取得
    pub fn get_line_position(&self) -> f32 {
        self.line_position
    }

    /// 現在のライン速度を取得
    pub fn get_line_velocity(&self) -> f32 {
        self.line_velocity
    }

    /// コントローラをリセット
    pub fn reset(&mut self) {
        self.line_position = 0.0;
        self.line_velocity = 0.0;
        self.prev_line_position = 0.0;
        self.line_detected = false;
        self.error_integral = 0.0;
        self.prev_error = 0.0;
    }

    /// センサー値のデバッグ情報を取得
    pub fn get_sensor_debug_info(&self) -> [u16; 4] {
        get_all_line_sensors()
    }
}

impl Default for LineController {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_line_controller_creation() {
        let controller = LineController::new();
        assert_eq!(controller.line_position, 0.0);
        assert_eq!(controller.line_detected, false);
    }

    #[test]
    fn test_sensor_positions() {
        // センサー位置が正しく設定されているかテスト
        assert_eq!(SENSOR_POSITIONS.len(), 4);
        assert!(SENSOR_POSITIONS[0] < SENSOR_POSITIONS[1]);
        assert!(SENSOR_POSITIONS[1] < SENSOR_POSITIONS[2]);
        assert!(SENSOR_POSITIONS[2] < SENSOR_POSITIONS[3]);
    }

    #[test]
    fn test_line_threshold() {
        // ライン閾値が正しく設定されているかテスト
        assert_eq!(LINE_THRESHOLD, 1024);
    }
}
