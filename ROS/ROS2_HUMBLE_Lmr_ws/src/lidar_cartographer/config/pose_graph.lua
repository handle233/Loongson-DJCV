POSE_GRAPH = {
  -- 每90个节点进行一次优化
  optimize_every_n_nodes = 60,

  -- 约束构建器设置
  constraint_builder = {
    -- 采样比率，用于控制约束构建时的采样密度
    sampling_ratio = 0.6,
    -- 最大约束距离，超过此距离的数据点将不考虑
    max_constraint_distance = 0.7,
    -- 最小匹配分数，低于该值的匹配将被丢弃
    min_score = 0.60,
    -- 全局定位的最小匹配分数
    global_localization_min_score = 0.7,
    -- 环路闭合的平移权重
    loop_closure_translation_weight = 1e2,
    -- 环路闭合的旋转权重
    loop_closure_rotation_weight = 1e5,
    -- 是否记录匹配的日志
    log_matches = true,

    -- 快速相关扫描匹配器配置（用于2D匹配）
    fast_correlative_scan_matcher = {
      -- 线性搜索窗口
      linear_search_window = 3,
      -- 角度搜索窗口
      angular_search_window = math.rad(15),
      -- 分支和边界搜索深度
      branch_and_bound_depth = 7,
    },

    -- Ceres 扫描匹配器配置（用于优化匹配过程）
    ceres_scan_matcher = {
      -- 占用空间的权重
      occupied_space_weight = 20.,
      -- 平移的权重
      translation_weight = 50,
      -- 旋转的权重
      rotation_weight = 10,
      -- Ceres求解器的选项
      ceres_solver_options = {
        -- 是否使用非单调步骤
        use_nonmonotonic_steps = true,
        -- 最大迭代次数
        max_num_iterations = 10,
        -- 使用的线程数
        num_threads = 1,
      },
    },

    -- 快速相关扫描匹配器配置（用于3D匹配）
    fast_correlative_scan_matcher_3d = {
      -- 分支和边界搜索深度
      branch_and_bound_depth = 8,
      -- 完整分辨率的深度
      full_resolution_depth = 3,
      -- 最小旋转分数
      min_rotational_score = 0.77,
      -- 最小低分辨率的分数
      min_low_resolution_score = 0.55,
      -- 线性XY搜索窗口
      linear_xy_search_window = 5.,
      -- 线性Z搜索窗口
      linear_z_search_window = 1.,
      -- 角度搜索窗口
      angular_search_window = math.rad(15.),
    },

    -- Ceres 扫描匹配器配置（用于3D优化）
    ceres_scan_matcher_3d = {
      -- 占用空间权重
      occupied_space_weight_0 = 5.,
      occupied_space_weight_1 = 30.,
      -- 平移的权重
      translation_weight = 10.,
      -- 旋转的权重
      rotation_weight = 1.,
      -- 是否仅优化偏航角
      only_optimize_yaw = false,
      -- Ceres求解器的选项
      ceres_solver_options = {
        -- 是否使用非单调步骤
        use_nonmonotonic_steps = false,
        -- 最大迭代次数
        max_num_iterations = 10,
        -- 使用的线程数
        num_threads = 1,
      },
    },
  },

  -- 匹配器平移和旋转的权重
  matcher_translation_weight = 5e2,
  matcher_rotation_weight = 1.6e3,

  -- 优化问题设置
  optimization_problem = {
    -- Huber损失函数的尺度
    huber_scale = 1e1,
    -- 加速度的权重
    acceleration_weight = 1.1e2,
    -- 旋转的权重
    rotation_weight = 1.6e4,
    -- 局部SLAM的平移和旋转权重
    local_slam_pose_translation_weight = 1e6,
    local_slam_pose_rotation_weight = 1e3,
    -- 里程计的平移和旋转权重
    odometry_translation_weight = 1e3,
    odometry_rotation_weight = 1e6,
    -- 固定坐标框的平移和旋转权重
    fixed_frame_pose_translation_weight = 1e5,
    fixed_frame_pose_rotation_weight = 1e5,
    -- 是否使用容忍损失函数
    fixed_frame_pose_use_tolerant_loss = false,
    -- 容忍损失函数的参数a
    fixed_frame_pose_tolerant_loss_param_a = 1,
    -- 容忍损失函数的参数b
    fixed_frame_pose_tolerant_loss_param_b = 1,
    -- 是否记录求解器的总结
    log_solver_summary = false,
    -- 是否使用在线IMU外部参数（适用于3D）
    use_online_imu_extrinsics_in_3d = true,
    -- 是否固定Z轴（适用于3D）
    fix_z_in_3d = false,
    -- Ceres求解器的选项
    ceres_solver_options = {
      -- 是否使用非单调步骤
      use_nonmonotonic_steps = false,
      -- 最大迭代次数
      max_num_iterations = 50,
      -- 使用的线程数
      num_threads = 7,
    },
  },

  -- 最终优化的最大迭代次数
  max_num_final_iterations = 200,
  -- 全局采样比率
  global_sampling_ratio = 0.001,
  -- 是否记录残差直方图
  log_residual_histograms = true,
  -- 在多少秒后进行全局约束搜索
  global_constraint_search_after_n_seconds = 10.,

  -- 你可以取消注释并调整这些值来控制子地图的修剪
  -- overlapping_submaps_trimmer_2d = {
  --    fresh_submaps_count = 1,
  --    min_covered_area = 2,
  --    min_added_submaps_count = 5,
  -- },
}
