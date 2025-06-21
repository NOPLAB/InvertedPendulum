function save_parameters(filename)
    % ワークスペースの指定パラメータを保存
    save(filename, 'p_M', 'p_m', 'p_g', 'p_l', 'p_r', 'p_Iw', 'p_n', 'p_G', ...
         'p_Jp', 'p_bx', 'p_btheta', 'p_Kt', 'p_Ke', 'p_Ra', 'p_La');
end