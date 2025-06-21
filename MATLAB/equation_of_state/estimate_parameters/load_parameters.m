function load_parameters(filename)
    % ファイルからパラメータをワークスペースに読み込み
    evalin('caller', ['load(''' filename ''')']);
end