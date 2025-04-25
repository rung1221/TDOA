clear all;
close all;
clc;

%% 参数设置
% 基础参数
% 初始声音传播速度(m/s) - 在20°C下的标准值
c_base = 343;

% 采样频率(Hz)
fs = 44100;

% 信号长度(s)
signal_length = 0.5;

% 无人机位置参数(真实位置，用于生成仿真数据)
drone_pos = [5, 3, 15]; % [x, y, z](m)

%% 环境参数设置
% 风速向量 [x方向, y方向, z方向] (m/s)
wind_velocity = [2, 1, 0.5]; % 风从西南方吹向东北方，略有上升气流

% 气温设置 (摄氏度)
temperature = 25; % 标准温度25°C

% 空气压力 (百帕)
pressure = 101.325; % 标准大气压

% 湿度(百分比) 
humidity = 60; % 相对湿度60%

% 根据温度计算声速 (m/s) - 经典物理模型
% c = 331.3 * sqrt(1 + (T/273.15))
c = 331.3 * sqrt(1 + (temperature/273.15));

% 考虑相对湿度影响的声速修正 (简化模型)
% 湿空气中的声速比干空气略高
humidity_factor = 1 + (humidity * 0.0001); % 简化模型
c = c * humidity_factor;

% 考虑大气压力的影响 (简化模型)
% 大气压力对声速影响较小，主要通过改变气体密度影响
pressure_factor = 1 + ((pressure - 101.325) * 0.00001); % 简化模型
c = c * pressure_factor;

% 计算空气密度 (kg/m^3) - 使用理想气体状态方程
% ρ = P/(R*T)，其中R是特定气体常数
R_air = 287.05; % 干空气的特定气体常数 (J/(kg·K))
T_kelvin = temperature + 273.15; % 转换为开尔文温度
air_density = pressure * 1000 / (R_air * T_kelvin); % 压力从百帕转换为帕

fprintf('===== 环境参数 =====\n');
fprintf('温度: %.1f °C\n', temperature);
fprintf('大气压力: %.3f kPa\n', pressure);
fprintf('相对湿度: %.1f %%\n', humidity);
fprintf('空气密度: %.4f kg/m³\n', air_density);
fprintf('声速: %.2f m/s\n', c);
fprintf('风向: [%.1f, %.1f, %.1f] m/s\n', wind_velocity);
fprintf('=====================\n\n');

%% 麦克风阵列设计 (保持不变)
% 创建两个相距较远的阵列，每个阵列具有较大的垂直高度差
array_distance = 10; % 两个阵列之间的距离(m)

% 定义更高的立体结构(三角形塔状结构，底部宽，顶部窄)
% 第一个阵列
array1_positions = [
    % 底层(地面) - 正方形布局
    0, 0, 0;
    1, 0, 0;
    1, 1, 0;
    0, 1, 0;
    % 中层 - 更小的正方形
    0.25, 0.25, 3;
    0.75, 0.25, 3;
    0.75, 0.75, 3;
    0.25, 0.75, 3;
    % 顶层 - 单点
    0.5, 0.5, 6
];

% 第二个阵列(平移到x轴上array_distance处)
array2_positions = array1_positions;
array2_positions(:,1) = array2_positions(:,1) + array_distance;

% 合并所有麦克风位置
mic_positions = [array1_positions; array2_positions];
num_mics = size(mic_positions, 1);

%% 生成仿真信号 - 考虑环境因素影响
% 创建时间向量
t = (0:signal_length*fs-1)/fs;

% 创建无人机信号(多频率正弦波模拟无人机声音)
drone_signal = zeros(1, length(t));
% 添加低频成分(模拟螺旋桨旋转声音)
drone_signal = drone_signal + sin(2*pi*100*t); 
drone_signal = drone_signal + 0.8*sin(2*pi*120*t); 
drone_signal = drone_signal + 0.6*sin(2*pi*140*t); 

% 添加高频成分(模拟电机声音)
drone_signal = drone_signal + 0.4*sin(2*pi*800*t);
drone_signal = drone_signal + 0.3*sin(2*pi*1200*t);

% 添加少量噪声和调制
envelope = 1 + 0.1*sin(2*pi*10*t);
drone_signal = envelope .* drone_signal + 0.05*randn(1, length(t));

% 为每个麦克风生成接收信号，考虑风的影响
mic_signals = zeros(num_mics, length(t));

for i = 1:num_mics
    % 计算从无人机到麦克风的向量
    direction_vec = mic_positions(i,:) - drone_pos;
    dist = norm(direction_vec);
    
    % 单位方向向量
    if dist > 0
        unit_direction = direction_vec / dist;
    else
        unit_direction = [0, 0, 0];
    end
    
    % 计算风对声速的影响 - 使用矢量投影
    % 风向分量在传播方向上的投影
    wind_proj = dot(wind_velocity, unit_direction);
    
    % 有效声速(顺风或逆风时的声速变化)
    c_effective = c + wind_proj;
    
    % 计算延迟时间(秒)，考虑有效声速
    delay_time = dist / c_effective;
    
    % 计算时间对应的采样点（取整）
    delay_samples = round(delay_time * fs);
    
    % 计算衰减因子 - 考虑距离衰减和空气吸收
    % 基础距离衰减(反比于距离)
    dist_attenuation = 1 / dist;
    
    % 空气吸收衰减(与频率、湿度、温度、距离相关) - 简化模型
    % 空气密度越大，吸收越多
    density_factor = air_density / 1.2; % 相对于标准空气密度(1.2kg/m³)的比值
    air_absorption = exp(-0.002 * density_factor * dist); % 简化的指数衰减模型
    
    % 总衰减
    attenuation = dist_attenuation * air_absorption;
    
    % 生成延迟信号(确保不超出范围)
    delayed_signal = zeros(1, length(t));
    if delay_samples < length(t)
        delayed_signal(delay_samples+1:end) = drone_signal(1:end-delay_samples) * attenuation;
    end
    
    % 添加环境噪声(风噪声随风速增加)
    wind_noise_level = 0.002 * (1 + 0.5 * norm(wind_velocity));
    noise = wind_noise_level * randn(1, length(t));
    
    % 最终信号
    mic_signals(i, :) = delayed_signal + noise;
end

%% 使用互相关的TDOA估计方法
% 参考麦克风(选择位于塔顶的麦克风作为参考，提高高度敏感度)
ref_mic = 9; % 第一个阵列的顶部麦克风

% 计算TDOA 
tdoa_estimates = zeros(num_mics, 1);
tdoa_estimates(ref_mic) = 0;  % 参考麦克风的TDOA为0

for i = [1:ref_mic-1, ref_mic+1:num_mics]
    % 提取信号
    sig_ref = mic_signals(ref_mic, :);
    sig_i = mic_signals(i, :);
    
    % 设置安全的最大延迟窗口(不能超过信号长度的一半)
    max_lag = min(round(0.1 * fs), floor(length(t)/2)-1);
    
    % 直接使用MATLAB的xcorr函数计算互相关
    [corr_vals, lags] = xcorr(sig_i, sig_ref, max_lag);
    
    % 找到相关函数的最大值
    [~, max_idx] = max(abs(corr_vals));
    
    % 获取对应的延迟
    lag_samples = lags(max_idx);
    
    % 转换为秒
    tdoa_estimates(i) = lag_samples / fs;
end

% 计算距离差(米)，使用实际的声速值
distance_diffs = tdoa_estimates * c;

%% 定义考虑环境因素的声源定位误差函数
function err = location_error(pos, mic_positions, ref_mic, distance_diffs, c, wind_velocity)
    num_mics = size(mic_positions, 1);
    err = zeros(num_mics, 1);
    
    % 参考麦克风到声源的方向向量和距离
    ref_direction = pos - mic_positions(ref_mic,:);
    d_ref = norm(ref_direction);
    unit_ref_direction = ref_direction / d_ref;
    
    % 计算风对参考麦克风的声速影响
    wind_proj_ref = dot(wind_velocity, unit_ref_direction);
    c_effective_ref = c + wind_proj_ref;
    
    % 参考麦克风到声源的有效距离
    effective_d_ref = d_ref;
    
    for i = 1:num_mics
        if i == ref_mic
            err(i) = 0;
            continue;
        end
        
        % 当前麦克风到声源的方向向量和距离
        mic_direction = pos - mic_positions(i,:);
        d_i = norm(mic_direction);
        
        if d_i > 0
            unit_mic_direction = mic_direction / d_i;
            
            % 计算风对当前麦克风的声速影响
            wind_proj_i = dot(wind_velocity, unit_mic_direction);
            c_effective_i = c + wind_proj_i;
            
            % 风速影响下的有效距离
            effective_d_i = d_i;
            
            % 预测的距离差(考虑不同路径上的有效声速)
            time_diff = effective_d_i/c_effective_i - effective_d_ref/c_effective_ref;
            predicted_diff = c * time_diff; % 转换为距离差
        else
            predicted_diff = -effective_d_ref;
        end
        
        % 误差(给高处麦克风更高权重)
        weight = 1.0;
        if i == 9 || i == 18  % 顶层麦克风
            weight = 3.0;
        elseif (i >= 5 && i <= 8) || (i >= 14 && i <= 17)  % 中层麦克风
            weight = 2.0;
        end
        
        err(i) = weight * (predicted_diff - distance_diffs(i));
    end
end

%% 多起点非线性优化
% 配置最小二乘非线性优化器
options = optimoptions('lsqnonlin', 'Display', 'off', 'Algorithm', 'trust-region-reflective', ...
                      'MaxIterations', 200, 'FunctionTolerance', 1e-8);

% 使用多个初始点进行优化
num_starts = 10;
best_pos = zeros(1, 3);
best_cost = inf;

% 设置搜索边界
lb = [-2, -2, 0];
ub = [15, 10, 30];

% 创建一组合理的初始点
initial_guesses = zeros(num_starts, 3);

% 第一个初始点是基于麦克风阵列中心的位置
initial_guesses(1,:) = [5, 3, 10];

% 其他初始点是在边界内的随机点
for i = 2:num_starts
    initial_guesses(i,:) = [
        5 + 3*randn(),   % x位置随机
        3 + 3*randn(),   % y位置随机
        5 + 10*rand()    % z位置随机
    ];
    
    % 确保在边界内
    initial_guesses(i,:) = max(lb, min(ub, initial_guesses(i,:)));
end

% 包装误差函数以包含风速参数
error_func = @(pos) location_error(pos, mic_positions, ref_mic, distance_diffs, c, wind_velocity);

% 运行多起点优化
for i = 1:num_starts
    [pos, cost] = lsqnonlin(error_func, initial_guesses(i,:), lb, ub, options);
    
    % 保存最佳结果
    if cost < best_cost
        best_cost = cost;
        best_pos = pos;
    end
end

% 最终估计位置
estimated_pos = best_pos;

%% 系统误差分析 - 测试不同环境条件
% 定义测试参数范围
wind_speeds = [0, 2, 5, 10]; % 风速大小(m/s)
wind_directions = {[0,0,0], [1,0,0], [0,1,0], [1,1,0], [-1,0,0]}; % 风向
temperatures = [0, 10, 20, 30, 40]; % 温度范围(°C)

% 初始化结果矩阵
num_wind_speeds = length(wind_speeds);
num_wind_dirs = length(wind_directions);
num_temps = length(temperatures);
total_tests = num_wind_speeds * num_wind_dirs * num_temps;

% 结果矩阵
results = zeros(total_tests, 8); % [风速大小, 风向x, 风向y, 风向z, 温度, 误差x, 误差y, 误差z]

% 运行系统误差测试
test_idx = 1;
fprintf('===== 开始系统误差分析 =====\n');
fprintf('总测试数: %d\n\n', total_tests);

for ws_idx = 1:num_wind_speeds
    ws = wind_speeds(ws_idx);
    
    for wd_idx = 1:num_wind_dirs
        wd = wind_directions{wd_idx};
        
        % 标准化风向
        if norm(wd) > 0
            wd = ws * wd / norm(wd); % 调整风速大小
        else
            wd = [0, 0, 0]; % 无风
        end
        
        for t_idx = 1:num_temps
            temp = temperatures(t_idx);
            
            % 计算当前温度下的声速
            c_test = 331.3 * sqrt(1 + (temp/273.15));
            
            fprintf('测试 %d/%d: 风速=[%.1f, %.1f, %.1f] m/s, 温度=%.1f°C, 声速=%.2f m/s\n', ...
                    test_idx, total_tests, wd(1), wd(2), wd(3), temp, c_test);
            
            % 生成此环境下接收到的信号
            test_mic_signals = generate_signals(drone_pos, mic_positions, drone_signal, fs, c_test, wd, air_density);
            
            % 计算TDOA
            test_tdoa = calculate_tdoa(test_mic_signals, ref_mic, fs, t);
            test_distance_diffs = test_tdoa * c_test;
            
            % 包装误差函数
            test_error_func = @(pos) location_error(pos, mic_positions, ref_mic, test_distance_diffs, c_test, wd);
            
            % 优化位置
            for i = 1:num_starts
                [pos, cost] = lsqnonlin(test_error_func, initial_guesses(i,:), lb, ub, options);
                
                % 保存最佳结果
                if cost < best_cost
                    best_cost = cost;
                    test_estimated_pos = pos;
                end
            end
            
            % 计算误差
            pos_error = test_estimated_pos - drone_pos;
            
            % 存储结果
            results(test_idx, :) = [ws, wd, temp, pos_error];
            
            % 输出当前测试结果
            fprintf('  估计位置: [%.2f, %.2f, %.2f], 误差: %.2f m\n\n', ...
                    test_estimated_pos(1), test_estimated_pos(2), test_estimated_pos(3), norm(pos_error));
            
            test_idx = test_idx + 1;
        end
    end
end

%% 可视化系统误差分析结果
% 1. 风速对定位误差的影响
figure('Position', [100, 100, 1200, 400]);

% 按风速大小分组
wind_speed_values = unique(results(:,1));
errors_by_wind = zeros(length(wind_speed_values), 1);

for i = 1:length(wind_speed_values)
    ws = wind_speed_values(i);
    mask = results(:,1) == ws;
    errors_by_wind(i) = mean(sqrt(sum(results(mask, 6:8).^2, 2))); % 平均欧几里得误差
end

% 风速影响绘图
subplot(1, 3, 1);
bar(wind_speed_values, errors_by_wind);
xlabel('风速 (m/s)');
ylabel('平均定位误差 (m)');
title('风速对定位精度的影响');
grid on;

% 2. 温度对定位误差的影响
temp_values = unique(results(:,5));
errors_by_temp = zeros(length(temp_values), 1);

for i = 1:length(temp_values)
    temp = temp_values(i);
    mask = results(:,5) == temp;
    errors_by_temp(i) = mean(sqrt(sum(results(mask, 6:8).^2, 2)));
end

% 温度影响绘图
subplot(1, 3, 2);
bar(temp_values, errors_by_temp);
xlabel('温度 (°C)');
ylabel('平均定位误差 (m)');
title('温度对定位精度的影响');
grid on;

% 3. 风向对定位误差的影响
subplot(1, 3, 3);
% 提取不同风向（使用前两个风向分量作为分类）
wind_dirs = unique(results(:,[2,3]), 'rows');
num_dirs = size(wind_dirs, 1);
dir_labels = cell(num_dirs, 1);
errors_by_dir = zeros(num_dirs, 1);

for i = 1:num_dirs
    wx = wind_dirs(i,1);
    wy = wind_dirs(i,2);
    
    % 创建标签
    if norm([wx, wy]) == 0
        dir_labels{i} = '无风';
    elseif wx > 0 && wy == 0
        dir_labels{i} = '→';
    elseif wx < 0 && wy == 0
        dir_labels{i} = '←';
    elseif wx == 0 && wy > 0
        dir_labels{i} = '↑';
    elseif wx > 0 && wy > 0
        dir_labels{i} = '↗';
    elseif wx < 0 && wy > 0
        dir_labels{i} = '↖';
    end
    
    % 计算对应风向的平均误差
    mask = (results(:,2) == wx) & (results(:,3) == wy);
    errors_by_dir(i) = mean(sqrt(sum(results(mask, 6:8).^2, 2)));
end

% 风向影响绘图
bar(1:num_dirs, errors_by_dir);
xticks(1:num_dirs);
xticklabels(dir_labels);
xlabel('风向');
ylabel('平均定位误差 (m)');
title('风向对定位精度的影响');
grid on;

% 调整整体布局
sgtitle('环境因素对TDOA声源定位的影响分析');

%% 可视化三维结果
figure('Position', [100, 100, 800, 600]);
hold on;

% 创建一个空的图例列表
legend_handles = [];
legend_labels = {};

% 首先绘制地面平面(但不添加到图例中)
x_ground = [-2, 15];
y_ground = [-2, 10];
[X_ground, Y_ground] = meshgrid(x_ground, y_ground);
Z_ground = zeros(size(X_ground));
surf(X_ground, Y_ground, Z_ground, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'FaceColor', [0.8 0.8 0.8]);

% === 绘制麦克风阵列结构 ===
% 绘制支架和连接线(不添加到图例中)
% 阵列1支架线
for i = 1:4
    % 连接底层
    next_i = mod(i, 4) + 1;
    plot3([array1_positions(i,1), array1_positions(next_i,1)], ...
          [array1_positions(i,2), array1_positions(next_i,2)], ...
          [array1_positions(i,3), array1_positions(next_i,3)], 'b-', 'LineWidth', 1);
    
    % 连接底层到中层
    plot3([array1_positions(i,1), array1_positions(i+4,1)], ...
          [array1_positions(i,2), array1_positions(i+4,2)], ...
          [array1_positions(i,3), array1_positions(i+4,3)], 'b-', 'LineWidth', 1);
    
    % 连接中层
    plot3([array1_positions(i+4,1), array1_positions(mod(i,4)+1+4,1)], ...
          [array1_positions(i+4,2), array1_positions(mod(i,4)+1+4,2)], ...
          [array1_positions(i+4,3), array1_positions(mod(i,4)+1+4,3)], 'b-', 'LineWidth', 1);
    
    % 连接中层到顶层
    plot3([array1_positions(i+4,1), array1_positions(9,1)], ...
          [array1_positions(i+4,2), array1_positions(9,2)], ...
          [array1_positions(i+4,3), array1_positions(9,3)], 'b-', 'LineWidth', 1);
end

% 阵列2支架线
for i = 1:4
    % 连接底层
    next_i = mod(i, 4) + 1;
    plot3([array2_positions(i,1), array2_positions(next_i,1)], ...
          [array2_positions(i,2), array2_positions(next_i,2)], ...
          [array2_positions(i,3), array2_positions(next_i,3)], 'g-', 'LineWidth', 1);
    
    % 连接底层到中层
    plot3([array2_positions(i,1), array2_positions(i+4,1)], ...
          [array2_positions(i,2), array2_positions(i+4,2)], ...
          [array2_positions(i,3), array2_positions(i+4,3)], 'g-', 'LineWidth', 1);
    
    % 连接中层
    plot3([array2_positions(i+4,1), array2_positions(mod(i,4)+1+4,1)], ...
          [array2_positions(i+4,2), array2_positions(mod(i,4)+1+4,2)], ...
          [array2_positions(i+4,3), array2_positions(mod(i,4)+1+4,3)], 'g-', 'LineWidth', 1);
    
    % 连接中层到顶层
    plot3([array2_positions(i+4,1), array2_positions(9,1)], ...
          [array2_positions(i+4,2), array2_positions(9,2)], ...
          [array2_positions(i+4,3), array2_positions(9,3)], 'g-', 'LineWidth', 1);
end

% 现在添加需要在图例中显示的元素
% 阵列1麦克风(单独一次性绘制所有点，使图例只有一个条目)
h1 = plot3(array1_positions(:,1), array1_positions(:,2), array1_positions(:,3), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', '阵列1麦克风');
legend_handles = [legend_handles; h1];
legend_labels{end+1} = '阵列1麦克风';

% 阵列2麦克风
h2 = plot3(array2_positions(:,1), array2_positions(:,2), array2_positions(:,3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', '阵列2麦克风');
legend_handles = [legend_handles; h2];
legend_labels{end+1} = '阵列2麦克风';

% 参考麦克风(特别标识)
h3 = plot3(mic_positions(ref_mic,1), mic_positions(ref_mic,2), mic_positions(ref_mic,3), 'co', 'MarkerSize', 10, 'MarkerFaceColor', 'c', 'LineWidth', 2, 'DisplayName', '参考麦克风');
legend_handles = [legend_handles; h3];
legend_labels{end+1} = '参考麦克风';

% 无人机真实位置
h4 = plot3(drone_pos(1), drone_pos(2), drone_pos(3), 'r*', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', '无人机真实位置');
legend_handles = [legend_handles; h4];
legend_labels{end+1} = '无人机真实位置';

% 估计位置
h5 = plot3(estimated_pos(1), estimated_pos(2), estimated_pos(3), 'mx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', '估计位置');
legend_handles = [legend_handles; h5];
legend_labels{end+1} = '估计位置';

% 定位误差线
h6 = plot3([drone_pos(1), estimated_pos(1)], ...
           [drone_pos(2), estimated_pos(2)], ...
           [drone_pos(3), estimated_pos(3)], 'k--', 'LineWidth', 1.5, 'DisplayName', '定位误差');
legend_handles = [legend_handles; h6];
legend_labels{end+1} = '定位误差';

% 绘制风速向量
% 绘制位置设在场景中心
wind_start = [5, 3, 8];
wind_scale = 2; % 缩放风速向量以便于可视化
wind_end = wind_start + wind_scale * wind_velocity;
h7 = quiver3(wind_start(1), wind_start(2), wind_start(3), ...
            wind_scale*wind_velocity(1), wind_scale*wind_velocity(2), wind_scale*wind_velocity(3), ...
            'LineWidth', 2, 'Color', 'r', 'MaxHeadSize', 1, 'DisplayName', '风速向量');
legend_handles = [legend_handles; h7];
legend_labels{end+1} = sprintf('风速向量 [%.1f,%.1f,%.1f] m/s', wind_velocity);

% 添加标签
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title(sprintf('麦克风阵列声源定位 - 环境因素影响分析 (温度: %.1f °C)', temperature));

% 创建明确指定的图例
legend(legend_handles, legend_labels, 'Location', 'best');

grid on;
view(30, 30);
axis([-2 15 -2 10 0 25]);

% 显示定位结果
fprintf('\n===== 定位结果 =====\n');
fprintf('真实无人机位置: [%.2f, %.2f, %.2f] m\n', drone_pos);
fprintf('估计无人机位置: [%.2f, %.2f, %.2f] m\n', estimated_pos);
fprintf('定位误差: %.2f m\n', norm(drone_pos - estimated_pos));
fprintf('高度估计值: %.2f m (真实值: %.2f m)\n', estimated_pos(3), drone_pos(3));

% 计算定位误差百分比
error_percent = norm(drone_pos - estimated_pos) / norm(drone_pos) * 100;
fprintf('定位误差百分比: %.2f%%\n', error_percent);
fprintf('高度估计误差: %.2f m (%.2f%%)\n', abs(drone_pos(3) - estimated_pos(3)), ...
        abs(drone_pos(3) - estimated_pos(3))/drone_pos(3)*100);
        
fprintf('\n===== 环境因素影响总结 =====\n');
% 修正风速影响部分
wind_corr = corr(wind_speed_values, errors_by_wind);
if isinf(wind_corr)
    wind_effect = '不确定';
elseif wind_corr > 0
    wind_effect = '大';
else
    wind_effect = '小';
end
fprintf('1. 风速影响: 风速越大，平均误差越%s\n', wind_effect);

% 修正温度影响部分
temp_corr = corr(temp_values, errors_by_temp);
if isinf(temp_corr)
    temp_effect = '不确定';
elseif temp_corr > 0
    temp_effect = '大';
else
    temp_effect = '小';
end
fprintf('2. 温度影响: 温度越高，平均误差越%s\n', temp_effect);
fprintf('3. 最不利风向: %s\n', dir_labels{find(errors_by_dir == max(errors_by_dir), 1, 'first')});
fprintf('4. 最有利风向: %s\n', dir_labels{find(errors_by_dir == min(errors_by_dir), 1, 'first')});

%% 辅助函数

% 生成考虑环境因素的信号
function mic_signals = generate_signals(drone_pos, mic_positions, drone_signal, fs, c, wind_velocity, air_density)
    num_mics = size(mic_positions, 1);
    mic_signals = zeros(num_mics, length(drone_signal));
    
    for i = 1:num_mics
        % 计算从无人机到麦克风的向量
        direction_vec = mic_positions(i,:) - drone_pos;
        dist = norm(direction_vec);
        
        % 单位方向向量
        if dist > 0
            unit_direction = direction_vec / dist;
        else
            unit_direction = [0, 0, 0];
        end
        
        % 计算风对声速的影响
        wind_proj = dot(wind_velocity, unit_direction);
        c_effective = c + wind_proj;
        
        % 计算延迟时间(秒)
        delay_time = dist / c_effective;
        
        % 计算时间对应的采样点（取整）
        delay_samples = round(delay_time * fs);
        
        % 信号强度衰减(考虑距离和空气密度)
        dist_attenuation = 1 / dist;
        density_factor = air_density / 1.2;
        air_absorption = exp(-0.002 * density_factor * dist);
        attenuation = dist_attenuation * air_absorption;
        
        % 生成延迟信号(确保不超出范围)
        delayed_signal = zeros(1, length(drone_signal));
        if delay_samples < length(drone_signal)
            delayed_signal(delay_samples+1:end) = drone_signal(1:end-delay_samples) * attenuation;
        end
        
        % 添加环境噪声(风噪声)
        wind_noise_level = 0.002 * (1 + 0.5 * norm(wind_velocity));
        noise = wind_noise_level * randn(1, length(drone_signal));
        mic_signals(i, :) = delayed_signal + noise;
    end
end

% 计算TDOA
function tdoa_estimates = calculate_tdoa(mic_signals, ref_mic, fs, t)
    num_mics = size(mic_signals, 1);
    tdoa_estimates = zeros(num_mics, 1);
    tdoa_estimates(ref_mic) = 0;  % 参考麦克风的TDOA为0
    
    for i = [1:ref_mic-1, ref_mic+1:num_mics]
        % 提取信号
        sig_ref = mic_signals(ref_mic, :);
        sig_i = mic_signals(i, :);
        
        % 设置安全的最大延迟窗口
        max_lag = min(round(0.1 * fs), floor(length(sig_ref)/2)-1);
        
        % 计算互相关
        [corr_vals, lags] = xcorr(sig_i, sig_ref, max_lag);
        
        % 找到相关函数的最大值
        [~, max_idx] = max(abs(corr_vals));
        
        % 获取对应的延迟
        lag_samples = lags(max_idx);
        
        % 转换为秒
        tdoa_estimates(i) = lag_samples / fs;
    end
end