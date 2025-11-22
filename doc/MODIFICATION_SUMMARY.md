# Path Searching 模块修改总结

## 修改日期
2024年（当前会话）

## 概述
本次修改主要涉及路径规划系统（path_searching）的多个节点，包括A*规划器、全向轮跟踪器、里程计模拟器等，以及相关的Web界面和QoS桥接节点。

---

## 1. astar_planner_node.py 修改

### 1.1 删除的功能
- **删除 map2 订阅**：移除了对 `/map2` 话题的订阅和 `map2_callback` 方法
- **删除 has_map2 标志**：不再需要 map2 数据作为规划触发条件

### 1.2 新增的功能

#### 1.2.1 定时器触发机制
- **定时器规划**：创建了 `planning_timer`，每 0.1 秒调用一次 `check_and_plan()`
- **规划触发条件**：需要 `has_kfs_data`、`has_odom` 和 `trigger_received` 都为 True

#### 1.2.2 路径删除和发布机制
- **delete_path() 方法**：发布空路径来删除旧路径
- **publish_path() 改进**：先删除旧路径，再发布新路径，确保路径更新清晰

#### 1.2.3 动态起点规划
- **订阅 odom_world**：获取机器人当前位置
- **动态起点计算**：起点从固定的 `[0, 1]` 改为从 `odom_world` 获取的当前位置对应的 grid 坐标
- **map_to_grid_coords() 方法**：将 map 坐标转换为 grid 索引

#### 1.2.4 方向覆盖功能
- **订阅 /planning/direction**：接收方向选择（"left" 或 "right"）
- **direction_override 变量**：存储当前方向覆盖设置
- **障碍物判定改进**：
  - `direction="left"` 时：col==2 位置的 kfs=1 不算障碍物
  - `direction="right"` 时：col==0 位置的 kfs=1 不算障碍物

#### 1.2.5 多目标点规划
- **多个目标点**：从单个目标 `[3, 1]` 改为三个目标点 `[[3, 0], [3, 1], [3, 2]]`
- **plan_path_to_goal() 方法**：规划到单个目标点并返回路径和代价
- **最优路径选择**：
  - 规划到所有三个目标点
  - 选择 cost 最小的路径
  - cost 相等时，按列表顺序优先选择（[3,0] > [3,1] > [3,2]）

### 1.3 修改的逻辑
- **kfs_data_callback**：每次收到 kfs_data 都将 `trigger_received` 设为 True
- **check_and_plan**：改为定时器回调，检查条件后执行规划

---

## 2. omnidirectional_tracker_node.py 修改

### 2.1 删除的功能
- **删除 can_go 订阅**：移除了对 `/can_go` 话题的订阅和相关逻辑
- **删除平滑减速机制**：移除了基于 `deceleration_rate` 的减速逻辑

### 2.2 新增的功能

#### 2.2.1 KFS 数据订阅和检查
- **订阅 /kfs_grid_data**：接收 KFS 网格数据
- **网格配置**：添加了与 astar_planner 一致的网格配置
- **map_to_grid_coords() 方法**：坐标转换功能
- **get_target_kfs_value() 方法**：获取目标点位置的 KFS 值

#### 2.2.2 KFS 等待机制
- **距离检查**：当距离目标点 < 0.7m 时，检查目标点的 KFS 值
- **停止逻辑**：如果 KFS ≠ 0，立即停止（cmd_vel=0）并等待
- **恢复逻辑**：持续检查直到 KFS 转为 0，然后继续移动
- **等待状态管理**：使用 `is_waiting_for_kfs` 和 `waiting_target` 标志

#### 2.2.3 路径起点智能判断
- **grid 坐标检查**：收到新路径时，检查是否已在第一个点的 grid 上
- **自动跳过**：如果已在第一个点的 grid 上且路径有多个点，从索引 1 开始跟踪

### 2.3 修改的逻辑
- **控制逻辑简化**：移除了 can_go 相关的复杂判断，直接根据 KFS 状态控制

---

## 3. odom_simulator.py 修改

### 3.1 新增功能

#### 3.1.1 TF 发布功能
- **TransformBroadcaster**：添加了 TF 发布器
- **_publish_tf() 方法**：发布从 `odom_frame` 到 `base_frame` 的 TF 变换
- **同步发布**：每次发布 odom 消息时同时发布 TF 变换
- **依赖添加**：在 `package.xml` 中添加了 `tf2_ros` 依赖

#### 3.1.2 初始位姿设置
- **订阅 /initialpose**：接收初始位姿设置
- **_initial_pose_callback() 方法**：
  - 更新机器人位置（x, y, z）
  - 从四元数提取 yaw 角
  - 支持在 RViz 中使用 "2D Pose Estimate" 工具设置初始位置

---

## 4. launch/path_tracking.launch.py 修改

### 4.1 节点配置
- **注释 path_decision_node**：暂时不使用路径决策节点
- **参数更新**：
  - 移除了 `deceleration_rate` 参数
  - 添加了 `kfs_check_distance` 参数（0.7m）

---

## 5. kfs_grid.html 修改（triple_map_manager 包）

### 5.1 新增功能

#### 5.1.1 方向控制按钮
- **Left/Right 按钮**：在 grid 右侧添加了两个方向选择按钮
- **publishDirection() 函数**：发布 `/planning/direction` 话题
- **按钮状态反馈**：active 状态显示当前选择的方向

#### 5.1.2 点击清空功能
- **clearCell() 函数**：点击已放置的 KFS 可以清空该格点
- **自动更新**：清空后自动更新计数并发布 grid 数据

#### 5.1.3 自动发布机制
- **自动发布**：放置或清空 KFS 时自动发布 grid 数据
- **无需手动点击**：移除了手动 "Publish to ROS2" 按钮的需求

#### 5.1.4 话题修改
- **发布话题变更**：从 `/kfs_grid_data` 改为 `/kfs_grid_data_raw`
- **QoS 桥接**：通过 `kfs_qos_bridge_node` 转发到 `/kfs_grid_data`（使用正确的 QoS）

---

## 6. kfs_qos_bridge_node.py（triple_map_manager 包）

### 6.1 新建节点
- **功能**：QoS 转发节点，解决 rosbridge 发布的 QoS 不匹配问题
- **订阅**：`/kfs_grid_data_raw`（来自 rosbridge，任何 QoS）
- **发布**：`/kfs_grid_data`（使用 TRANSIENT_LOCAL QoS）
- **作用**：确保使用 TRANSIENT_LOCAL QoS 的订阅者能够接收到消息

---

## 7. 系统整体流程

### 7.1 数据流
```
外部系统
├── /map2 (已删除)
├── /kfs_grid_data_raw (Web) → kfs_qos_bridge → /kfs_grid_data
├── /task/trigger
└── /planning/direction (Web)

/odom_world (odom_simulator)
    ↓
astar_planner_node
    ├── 订阅: odom_world, kfs_data, trigger, direction
    ├── 规划: 从当前位置到 [3,0], [3,1], [3,2]（选择cost最小的）
    └── 发布: /planning/path

/planning/path
    ↓
omnidirectional_tracker_node
    ├── 订阅: path, odom_world, kfs_data
    ├── 检查: 距离target 0.7m时检查KFS
    ├── 控制: PID跟踪路径
    └── 发布: /cmd_vel

/cmd_vel
    ↓
odom_simulator
    ├── 积分速度得到位姿
    ├── 发布: /odom_world
    └── 发布: TF (map -> base_link)
```

### 7.2 关键特性

1. **动态起点规划**：A* 规划器从机器人当前位置开始规划
2. **多目标点选择**：规划到三个目标点，自动选择 cost 最小的
3. **方向覆盖**：通过方向选择可以改变障碍物判定规则
4. **KFS 等待机制**：接近目标点时检查 KFS，如果非0则等待
5. **智能路径跟踪**：如果已在路径起点，自动跳过第一个点

---

## 8. 参数配置

### 8.1 astar_planner_node
- 网格大小：4 rows × 3 cols
- 目标点：`[[3, 0], [3, 1], [3, 2]]`
- 规划频率：0.1s（定时器）

### 8.2 omnidirectional_tracker_node
- KFS 检查距离：0.7m
- 目标距离阈值：0.1m
- 控制频率：50 Hz

### 8.3 odom_simulator
- 发布频率：50 Hz
- 坐标系：odom_frame='map', base_frame='base_link'

---

## 9. 注意事项

1. **QoS 匹配**：kfs_grid_data 使用 TRANSIENT_LOCAL QoS，需要匹配的订阅者
2. **路径删除**：每次发布新路径前会先删除旧路径（发布空路径）
3. **方向覆盖**：方向改变只更新 `direction_override`，不触发规划
4. **KFS 检查**：只在距离目标点 0.7m 时检查，不是每个路径点都检查
5. **多目标规划**：每次规划都会计算到所有三个目标点的路径，选择最优的

---

## 10. 待优化问题

1. **路径跟踪顿挫**：当前实现中，接近每个路径点时 PID 误差变小导致减速
   - **建议**：使用 lookahead 机制，选择路径前方一定距离的点作为目标

---

## 11. 文件清单

### 修改的文件
- `path_searching/path_searching/astar_planner_node.py`
- `path_searching/path_searching/omnidirectional_tracker_node.py`
- `path_searching/path_searching/odom_simulator.py`
- `path_searching/launch/path_tracking.launch.py`
- `path_searching/package.xml`（添加 tf2_ros 依赖）

### 新建的文件
- `triple_map_manager/triple_map_manager/kfs_qos_bridge_node.py`
- `triple_map_manager/web/kfs_grid.html`（修改）

### 修改的其他文件
- `triple_map_manager/setup.py`（添加 kfs_qos_bridge 入口点）
- `triple_map_manager/launch/kfs_direct.launch.py`（添加 kfs_qos_bridge 节点）

---

## 12. 测试建议

1. **测试多目标规划**：验证是否选择 cost 最小的目标点
2. **测试方向覆盖**：验证 left/right 方向是否正确影响障碍物判定
3. **测试 KFS 等待**：验证在 KFS 非0时是否正确停止和恢复
4. **测试动态起点**：验证机器人移动后规划起点是否正确更新
5. **测试路径跟踪**：观察是否存在顿挫现象，考虑实现 lookahead 机制




