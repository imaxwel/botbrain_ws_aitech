# 在另一个项目中实现 DEX3 右手压力（触觉）读取的设计说明（订阅 `/lf/dex3/right/state`）

## 目标

在目标项目中新增一个“DEX3 右手压力读取模块”，用于**从 ROS2 Topic `/lf/dex3/right/state` 订阅手状态消息**，并从消息中的**压力传感器数组**提取出可用的压力值（可选：做 baseline 标定与聚合输出）。

> 约束：只需要**右手**。Topic 固定为 **`/lf/dex3/right/state`**。

---

## 1. 输入与输出定义

### 1.1 输入（订阅）

- ROS2 topic：`/lf/dex3/right/state`
- 消息类型：`unitree_hg::msg::HandState`（来自 Unitree HG 消息包）
- 压力数据位置（在 unitree_g1_dex3_stack 中使用方式）：
  - `msg->press_sensor_state`（数组/列表）
  - 每个元素 `press` 中有 `press.pressure`（数组/列表，按 taxel 索引）

> 注：如果目标项目的消息类型与字段名不同，以实际为准；但设计目标是拿到一个 `pressure[]` 数组（或多个 `pressure[]`）进行解析。

### 1.2 输出（建议提供两种层级）

- **raw 输出（可选）**：所有有效 taxel 的压力浮点值（经过缩放、可减 baseline）
- **聚合输出（推荐）**：将 taxel 分组，输出如下 5 个值：
  - `thumb`（拇指）
  - `index`（食指）
  - `middle`（中指）
  - `palm`（掌心）
  - `finger_palm`（index+middle+palm 的整体平均；原项目用于闭环抓取）

输出形式建议：
- C++：提供 getter（线程安全）
- ROS2：发布一个自定义/标准消息（例如 `std_msgs/Float32MultiArray` 或自定义 `Dex3Tactile`）

---

## 2. 压力值解析规则（必须对齐原项目）

以下规则来自 `unitree_g1_dex3_stack` 的 `dex3_controller` 逻辑，应尽量保持一致，以保证阈值与行为匹配：

### 2.1 无效值（invalid sentinel）

- 若 `pressure[idx] == 30000`，视为无效/缺失，必须跳过（不用于标定与聚合）。

### 2.2 缩放比例（scale）

- 读取后转为浮点：  
  `val = pressure[idx] / 10000.0f`

### 2.3 baseline 标定（零点校准）

目的：消除每个 taxel 的初始偏置，输出接触增量。

- baseline 数组 `baseline[idx]` 初始为 `NaN`（表示未标定）
- 标定触发条件：模块启动后首次收到数据时自动标定，或通过显式服务/接口触发标定
- 标定规则：
  - 当 `pressure[idx] != 30000` 且 `baseline[idx]` 仍为 `NaN` 时：
    - `baseline[idx] = pressure[idx] / 10000.0f`
    - 将 `valid[idx] = true`（记录该 taxel “确实存在且可用”）
- 运行时使用值：
  - `val_used = pressure[idx]/10000.0f - baseline[idx]`

> 设计要点：  
> - baseline 只对读到“有效值”的 idx 初始化；永远无效的 idx 不参与计算。  
> - `valid[]` 用于屏蔽不可用 taxel，避免污染聚合结果。

---

## 3. taxel 分组与聚合策略（推荐复用）

原项目将 taxel 映射到不同部位并做平均值，用于稳定闭环：

- Thumb（拇指）：indices `{0, 1}`
- Middle（中指）：indices `{2, 3}`
- Index（食指）：indices `{4, 5}`
- Palm（掌心）：indices `{6, 7, 8}`

聚合过程：
1. 遍历 `msg->press_sensor_state`（可能有多个 `press` 元素）
2. 对每个 group 的 idx：
   - 检查边界 `idx < pressure.size()`
   - 检查 `pressure[idx] != 30000`
   - 检查 `valid[idx] == true`
3. 累加 `val_used`，统计 count
4. `avg = sum / count`（如果 `count==0` 则输出 0）

再计算：
- `finger_palm`：将 `index + middle + palm` 的 sum/count 合并后再平均（与原项目一致）

---

## 4. 模块结构设计（ROS2 C++ 示例架构）

建议实现一个独立节点，例如：

- 节点名：`dex3_right_tactile_reader`
- 订阅：`/lf/dex3/right/state`
- 可选发布：`/dex3/right/tactile_agg`（Float32MultiArray，顺序固定）

### 4.1 内部状态（线程安全）

- `std::vector<float> baseline_`（用 `NaN` 初始化）
- `std::vector<bool> valid_`
- `bool need_calibration_`（启动时 true；标定完成置 false）
- 最近一次聚合结果：
  - `float thumb_`, `index_`, `middle_`, `palm_`, `finger_palm_`

并使用 mutex 或者原子/lock-free 缓冲（取决于项目风格）保证读取一致性。

---

## 5. 核心伪代码（给另一个 AI 直接落地实现用）

```cpp
// constants
const int INVALID = 30000;
const float SCALE = 10000.0f;

// state
std::vector<float> baseline;   // NaN means unset
std::vector<bool> valid;
bool need_calib = true;

// subscribe callback
void onHandState(const unitree_hg::msg::HandState::SharedPtr msg) {

  // 1) calibration (once)
  if (need_calib) {
    size_t max_idx = 0;
    for (auto &press : msg->press_sensor_state) {
      if (!press.pressure.empty()) max_idx = max(max_idx, press.pressure.size() - 1);
    }
    resize baseline/valid to max_idx+1 (baseline fill NaN, valid fill false)

    for (auto &press : msg->press_sensor_state) {
      for (size_t idx=0; idx<press.pressure.size(); ++idx) {
        if (press.pressure[idx] != INVALID && isNaN(baseline[idx])) {
          baseline[idx] = press.pressure[idx] / SCALE;
          valid[idx] = true;
        }
      }
    }
    need_calib = false;
  }

  // 2) aggregate helper
  auto agg_group = [&](std::initializer_list<size_t> indices) -> std::pair<float,int> {
    float sum = 0.f; int count = 0;
    for (auto &press : msg->press_sensor_state) {
      for (auto idx : indices) {
        if (idx < press.pressure.size() &&
            idx < valid.size() && valid[idx] &&
            press.pressure[idx] != INVALID) {

          float v = press.pressure[idx] / SCALE - baseline[idx];
          sum += v;
          count += 1;
        }
      }
    }
    return {sum, count};
  };

  auto [thumb_sum, thumb_cnt]   = agg_group({0,1});
  auto [middle_sum, middle_cnt] = agg_group({2,3});
  auto [index_sum, index_cnt]   = agg_group({4,5});
  auto [palm_sum, palm_cnt]     = agg_group({6,7,8});

  float thumb_avg  = thumb_cnt  ? thumb_sum  / thumb_cnt  : 0.f;
  float middle_avg = middle_cnt ? middle_sum / middle_cnt : 0.f;
  float index_avg  = index_cnt  ? index_sum  / index_cnt  : 0.f;
  float palm_avg   = palm_cnt   ? palm_sum   / palm_cnt   : 0.f;

  float fp_sum = index_sum + middle_sum + palm_sum;
  int fp_cnt   = index_cnt + middle_cnt + palm_cnt;
  float finger_palm_avg = fp_cnt ? fp_sum / fp_cnt : 0.f;

  // 3) store & optionally publish
  store(thumb_avg, index_avg, middle_avg, palm_avg, finger_palm_avg);
  publish_if_needed(...);
}
```

---

## 6. 校准触发策略（建议）

由于你只要求右手且 topic 固定，推荐两种方案：

1) **自动校准一次**：节点启动后第一帧可用数据作为 baseline（简单）
2) **提供服务/接口重新校准**：例如 `std_srvs/Trigger`，调用后将 `need_calib=true`，下一帧重新设 baseline（更稳健）

> 注意：baseline 校准应尽量在“未接触物体/空载”状态进行。

---

## 7. 测试与验证要点

- 打印或发布 raw pressure（缩放后）与 baseline 后的差值，确保：
  - 未接触时接近 0（baseline 生效）
  - 接触时为正（或符合预期）
- 检查 invalid sentinel：
  - 若某些 idx 长期为 30000，应确认这些 taxel 实际不存在或未启用
- 检查分组映射：
  - thumb/index/middle/palm 的 idx 分组是否与设备固件一致（必要时调整映射）

---

## 8. 实现边界（明确不做的事情）

- 本设计**不涉及底层硬件采集**（I2C/SPI/ADC），仅从 `/lf/dex3/right/state` 读取。
- 若系统中没有任何节点发布 `/lf/dex3/right/state`，则本模块无法工作；需要先解决 state 发布链路。

---