%% 統合制御器クラス
classdef sample2_class < handle
    properties
        % システムパラメータ
        params
        
        % 制御ゲイン
        Kp_i    % 電流比例ゲイン
        Ki_i    % 電流積分ゲイン
        Kp_v    % 速度比例ゲイン
        Ki_v    % 速度積分ゲイン
        K_lqr   % LQRゲイン
        
        % 積分器の状態
        int_i = 0    % 電流誤差の積分
        int_v = 0    % 速度誤差の積分
        
        % サンプリング時間
        dt = 0.001   % 1kHz
        
        % フィルタ
        lpf_current  % 電流測定用ローパスフィルタ
        lpf_velocity % 速度測定用ローパスフィルタ
    end
    
    methods
        function obj = sample2_class(params, Kp_i, Ki_i, Kp_v, Ki_v, K_lqr)
            obj.params = params;
            obj.Kp_i = Kp_i;
            obj.Ki_i = Ki_i;
            obj.Kp_v = Kp_v;
            obj.Ki_v = Ki_v;
            obj.K_lqr = K_lqr;
            
            % ローパスフィルタの設計（ノイズ除去用）
            fc_current = 1000;  % カットオフ周波数 1kHz
            fc_velocity = 100;  % カットオフ周波数 100Hz
            obj.lpf_current = obj.design_lpf(fc_current, 1/obj.dt);
            obj.lpf_velocity = obj.design_lpf(fc_velocity, 1/obj.dt);
        end
        
        function lpf = design_lpf(~, fc, fs)
            % 1次ローパスフィルタの設計
            wc = 2*pi*fc;
            lpf.alpha = wc*2/fs / (1 + wc*2/fs);
            lpf.state = 0;
        end
        
        function y_filt = apply_lpf(~, lpf, u)
            % ローパスフィルタの適用
            lpf.state = lpf.alpha * u + (1 - lpf.alpha) * lpf.state;
            y_filt = lpf.state;
        end
        
        function [V, i_ref, v_ref, F_ref] = compute_control(obj, x_meas, x_ref)
            % カスケード制御の計算
            % 入力:
            %   x_meas: 測定値 [x; x_dot; phi; phi_dot; i]
            %   x_ref: 目標値 [x_ref; x_dot_ref; phi_ref; phi_dot_ref]
            % 出力:
            %   V: モータ印加電圧
            %   i_ref: 電流指令値
            %   v_ref: 速度指令値
            %   F_ref: 推力指令値
            
            % 測定値のフィルタリング
            i_filt = obj.apply_lpf(obj.lpf_current, x_meas(5));
            v_filt = obj.apply_lpf(obj.lpf_velocity, x_meas(2));
            
            % Level 3: 位置・角度制御（推力指令の計算）
            x_cart = x_meas(1:4);
            x_cart_ref = x_ref;
            e_cart = x_cart_ref - x_cart;
            F_ref = -obj.K_lqr * e_cart;
            
            % 推力から速度指令への変換（簡略化）
            % 定常状態では F = b_x * v なので
            v_ref = F_ref / obj.params(9);  % 簡易的な速度指令
            
            % Level 2: 速度制御（電流指令の計算）
            e_v = v_ref - v_filt;
            obj.int_v = obj.int_v + e_v * obj.dt;
            
            % アンチワインドアップ（積分器の飽和防止）
            obj.int_v = max(min(obj.int_v, 10), -10);
            
            % 速度制御出力（必要なモータトルク）
            tau_ref = obj.Kp_v * e_v + obj.Ki_v * obj.int_v;
            
            % トルクから電流指令への変換
            i_ref = tau_ref * obj.params(5) / (2 * obj.params(7) * obj.params(11));
            
            % Level 1: 電流制御（電圧指令の計算）
            e_i = i_ref - i_filt;
            obj.int_i = obj.int_i + e_i * obj.dt;
            
            % アンチワインドアップ
            obj.int_i = max(min(obj.int_i, 10), -10);
            
            % 電流制御出力 + フィードフォワード項
            omega_motor = obj.params(7) * v_filt / obj.params(5);
            V_ff = obj.params(13) * i_ref + obj.params(12) * omega_motor;  % フィードフォワード
            V_fb = obj.Kp_i * e_i + obj.Ki_i * obj.int_i;                  % フィードバック
            V = V_ff + V_fb;
            
            % 電圧飽和
            V_max = 24;  % 最大電圧 24V
            V = max(min(V, V_max), -V_max);
        end
        
        function reset_integrators(obj)
            % 積分器のリセット
            obj.int_i = 0;
            obj.int_v = 0;
        end
    end
end