/*
 * test_mit_adaptive_controller.cpp
 *
 *  Created on: July 8, 2025
 *      Author: nop
 */

#include "mit_adaptive_controller.hpp"
#include "test_framework.hpp"

TEST_FUNCTION(mit_adaptive_controller_initialization) {
  float dt = 0.0001f;
  MitAdaptiveController controller(dt, 0.1f, 0.1f, 0.1f, 0.1f);

  float gains[4];
  controller.getAdaptiveGains(gains);

  // 初期化直後は適応ゲインは0であることを確認
  for (int i = 0; i < 4; i++) {
    ASSERT_EQ(0.0f, gains[i]);
  }

  // 初期ゲインを設定
  controller.setInitialGains(-1.4142f, -5.3580f, -20.9010f, -2.5862f);
  controller.getAdaptiveGains(gains);

  // 初期ゲインが正しく設定されることを確認
  ASSERT_NEAR(-1.4142f, gains[0], 0.0001f);
  ASSERT_NEAR(-5.3580f, gains[1], 0.0001f);
  ASSERT_NEAR(-20.9010f, gains[2], 0.0001f);
  ASSERT_NEAR(-2.5862f, gains[3], 0.0001f);

  return true;
}

TEST_FUNCTION(mit_adaptive_controller_update) {
  float dt = 0.0001f;
  MitAdaptiveController controller(dt, 0.1f, 0.1f, 0.1f, 0.1f);

  // 初期ゲインを設定
  controller.setInitialGains(-1.4142f, -5.3580f, -20.9010f, -2.5862f);
  controller.setReferenceModel(10.0f, 0.7f);

  // 状態変数の設定
  float x = 0.01f;      // 位置 [m]
  float dx = 0.1f;      // 速度 [m/s]
  float theta = 0.1f;   // 角度 [rad]
  float dtheta = 0.5f;  // 角速度 [rad/s]
  float reference = 0.0f;

  // 制御入力の計算
  float control_input = controller.update(x, dx, theta, dtheta, reference);

  // 制御入力が計算されることを確認
  ASSERT_TRUE(control_input != 0.0f);

  // 適応誤差が計算されることを確認
  float adaptation_error = controller.getAdaptationError();
  ASSERT_TRUE(adaptation_error != 0.0f);

  return true;
}

TEST_FUNCTION(mit_adaptive_controller_gain_adaptation) {
  float dt = 0.0001f;
  MitAdaptiveController controller(dt, 0.1f, 0.1f, 0.1f, 0.1f);

  // 初期ゲインを設定
  controller.setInitialGains(-1.4142f, -5.3580f, -20.9010f, -2.5862f);
  controller.setReferenceModel(10.0f, 0.7f);

  float gains_before[4];
  controller.getAdaptiveGains(gains_before);

  // 複数回の更新を実行
  for (int i = 0; i < 1000; i++) {
    float x = 0.01f * i / 1000.0f;
    float dx = 0.1f;
    float theta = 0.1f;
    float dtheta = 0.5f;
    float reference = 0.0f;

    controller.update(x, dx, theta, dtheta, reference);
  }

  float gains_after[4];
  controller.getAdaptiveGains(gains_after);

  // ゲインが適応により変化することを確認
  bool gains_changed = false;
  for (int i = 0; i < 4; i++) {
    if (gains_before[i] != gains_after[i]) {
      gains_changed = true;
      break;
    }
  }
  ASSERT_TRUE(gains_changed);

  return true;
}

TEST_FUNCTION(mit_adaptive_controller_reset) {
  float dt = 0.0001f;
  MitAdaptiveController controller(dt, 0.1f, 0.1f, 0.1f, 0.1f);

  // 初期ゲインを設定
  controller.setInitialGains(-1.4142f, -5.3580f, -20.9010f, -2.5862f);

  // 複数回の更新を実行してゲインを変化させる
  for (int i = 0; i < 100; i++) {
    controller.update(0.01f, 0.1f, 0.1f, 0.5f, 0.0f);
  }

  // リセット実行
  controller.reset();

  // ゲインが初期値に戻ることを確認
  float gains[4];
  controller.getAdaptiveGains(gains);

  ASSERT_NEAR(-1.4142f, gains[0], 0.0001f);
  ASSERT_NEAR(-5.3580f, gains[1], 0.0001f);
  ASSERT_NEAR(-20.9010f, gains[2], 0.0001f);
  ASSERT_NEAR(-2.5862f, gains[3], 0.0001f);

  // 適応誤差もリセットされることを確認
  ASSERT_EQ(0.0f, controller.getAdaptationError());

  return true;
}

TEST_FUNCTION(mit_adaptive_controller_gain_limits) {
  float dt = 0.0001f;
  MitAdaptiveController controller(dt, 10.0f, 10.0f, 10.0f,
                                   10.0f);  // 高い適応率

  // 初期ゲインを設定
  controller.setInitialGains(-1.4142f, -5.3580f, -20.9010f, -2.5862f);

  // 大きな誤差を発生させる更新を実行
  for (int i = 0; i < 10000; i++) {
    controller.update(1.0f, 1.0f, 1.0f, 1.0f, 0.0f);
  }

  float gains[4];
  controller.getAdaptiveGains(gains);

  // ゲインが制限範囲内にあることを確認（負のゲインの場合は境界が逆転）
  for (int i = 0; i < 4; i++) {
    float initial_gain = (i == 0)   ? -1.4142f
                         : (i == 1) ? -5.3580f
                         : (i == 2) ? -20.9010f
                                    : -2.5862f;

    // 負のゲインの場合は上限と下限が逆転する
    float upper_bound = initial_gain * 0.5f;  // より負の値
    float lower_bound = initial_gain * 2.0f;  // より負でない値

    ASSERT_TRUE(gains[i] <= upper_bound);
    ASSERT_TRUE(gains[i] >= lower_bound);
  }

  return true;
}

void test_mit_adaptive_controller_run_all_tests() {
  RUN_TEST(mit_adaptive_controller_initialization);
  RUN_TEST(mit_adaptive_controller_update);
  RUN_TEST(mit_adaptive_controller_gain_adaptation);
  RUN_TEST(mit_adaptive_controller_reset);
  RUN_TEST(mit_adaptive_controller_gain_limits);
}