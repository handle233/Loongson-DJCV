POSE_GRAPH = {
  -- ÿ90���ڵ����һ���Ż�
  optimize_every_n_nodes = 60,

  -- Լ������������
  constraint_builder = {
    -- �������ʣ����ڿ���Լ������ʱ�Ĳ����ܶ�
    sampling_ratio = 0.6,
    -- ���Լ�����룬�����˾�������ݵ㽫������
    max_constraint_distance = 0.7,
    -- ��Сƥ����������ڸ�ֵ��ƥ�佫������
    min_score = 0.60,
    -- ȫ�ֶ�λ����Сƥ�����
    global_localization_min_score = 0.7,
    -- ��·�պϵ�ƽ��Ȩ��
    loop_closure_translation_weight = 1e2,
    -- ��·�պϵ���תȨ��
    loop_closure_rotation_weight = 1e5,
    -- �Ƿ��¼ƥ�����־
    log_matches = true,

    -- �������ɨ��ƥ�������ã�����2Dƥ�䣩
    fast_correlative_scan_matcher = {
      -- ������������
      linear_search_window = 3,
      -- �Ƕ���������
      angular_search_window = math.rad(15),
      -- ��֧�ͱ߽��������
      branch_and_bound_depth = 7,
    },

    -- Ceres ɨ��ƥ�������ã������Ż�ƥ����̣�
    ceres_scan_matcher = {
      -- ռ�ÿռ��Ȩ��
      occupied_space_weight = 20.,
      -- ƽ�Ƶ�Ȩ��
      translation_weight = 50,
      -- ��ת��Ȩ��
      rotation_weight = 10,
      -- Ceres�������ѡ��
      ceres_solver_options = {
        -- �Ƿ�ʹ�÷ǵ�������
        use_nonmonotonic_steps = true,
        -- ����������
        max_num_iterations = 10,
        -- ʹ�õ��߳���
        num_threads = 1,
      },
    },

    -- �������ɨ��ƥ�������ã�����3Dƥ�䣩
    fast_correlative_scan_matcher_3d = {
      -- ��֧�ͱ߽��������
      branch_and_bound_depth = 8,
      -- �����ֱ��ʵ����
      full_resolution_depth = 3,
      -- ��С��ת����
      min_rotational_score = 0.77,
      -- ��С�ͷֱ��ʵķ���
      min_low_resolution_score = 0.55,
      -- ����XY��������
      linear_xy_search_window = 5.,
      -- ����Z��������
      linear_z_search_window = 1.,
      -- �Ƕ���������
      angular_search_window = math.rad(15.),
    },

    -- Ceres ɨ��ƥ�������ã�����3D�Ż���
    ceres_scan_matcher_3d = {
      -- ռ�ÿռ�Ȩ��
      occupied_space_weight_0 = 5.,
      occupied_space_weight_1 = 30.,
      -- ƽ�Ƶ�Ȩ��
      translation_weight = 10.,
      -- ��ת��Ȩ��
      rotation_weight = 1.,
      -- �Ƿ���Ż�ƫ����
      only_optimize_yaw = false,
      -- Ceres�������ѡ��
      ceres_solver_options = {
        -- �Ƿ�ʹ�÷ǵ�������
        use_nonmonotonic_steps = false,
        -- ����������
        max_num_iterations = 10,
        -- ʹ�õ��߳���
        num_threads = 1,
      },
    },
  },

  -- ƥ����ƽ�ƺ���ת��Ȩ��
  matcher_translation_weight = 5e2,
  matcher_rotation_weight = 1.6e3,

  -- �Ż���������
  optimization_problem = {
    -- Huber��ʧ�����ĳ߶�
    huber_scale = 1e1,
    -- ���ٶȵ�Ȩ��
    acceleration_weight = 1.1e2,
    -- ��ת��Ȩ��
    rotation_weight = 1.6e4,
    -- �ֲ�SLAM��ƽ�ƺ���תȨ��
    local_slam_pose_translation_weight = 1e6,
    local_slam_pose_rotation_weight = 1e3,
    -- ��̼Ƶ�ƽ�ƺ���תȨ��
    odometry_translation_weight = 1e3,
    odometry_rotation_weight = 1e6,
    -- �̶�������ƽ�ƺ���תȨ��
    fixed_frame_pose_translation_weight = 1e5,
    fixed_frame_pose_rotation_weight = 1e5,
    -- �Ƿ�ʹ��������ʧ����
    fixed_frame_pose_use_tolerant_loss = false,
    -- ������ʧ�����Ĳ���a
    fixed_frame_pose_tolerant_loss_param_a = 1,
    -- ������ʧ�����Ĳ���b
    fixed_frame_pose_tolerant_loss_param_b = 1,
    -- �Ƿ��¼��������ܽ�
    log_solver_summary = false,
    -- �Ƿ�ʹ������IMU�ⲿ������������3D��
    use_online_imu_extrinsics_in_3d = true,
    -- �Ƿ�̶�Z�ᣨ������3D��
    fix_z_in_3d = false,
    -- Ceres�������ѡ��
    ceres_solver_options = {
      -- �Ƿ�ʹ�÷ǵ�������
      use_nonmonotonic_steps = false,
      -- ����������
      max_num_iterations = 50,
      -- ʹ�õ��߳���
      num_threads = 7,
    },
  },

  -- �����Ż�������������
  max_num_final_iterations = 200,
  -- ȫ�ֲ�������
  global_sampling_ratio = 0.001,
  -- �Ƿ��¼�в�ֱ��ͼ
  log_residual_histograms = true,
  -- �ڶ���������ȫ��Լ������
  global_constraint_search_after_n_seconds = 10.,

  -- �����ȡ��ע�Ͳ�������Щֵ�������ӵ�ͼ���޼�
  -- overlapping_submaps_trimmer_2d = {
  --    fresh_submaps_count = 1,
  --    min_covered_area = 2,
  --    min_added_submaps_count = 5,
  -- },
}
