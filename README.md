# GNSS 单点定位与完好性监测服务（本地模拟）

一个**完全本地、确定性**的卫星导航单点定位（SPP）与接收机自主完好性监测（RAIM）服务。
所有星座、广播星历与伪距观测均由内置模拟器产生，支持注入接收机噪声、多径偏差、
对流层/电离层传播延迟以及单星阶跃、多星阶跃与缓慢漂移三类故障；**不依赖任何外部
定位服务或真实卫星信号**。

## 运行

```bash
# 演示：16 个历元，含 G11 阶跃故障与部分观测缺失
go run ./cmd/gnss -seed 20260930 -epochs 16 -store /tmp/gnss-state.gob

# 全量自动化测试（含竞态检测）
go test -race ./...
```

命令行参数：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `-seed` | `20260930` | 随机种子，固定种子下结果逐历元可复现 |
| `-epochs` | `12` | 模拟历元数 |
| `-store` | 临时目录下 `gnss-state.gob` | 持久化状态文件 |

## 包结构

- `model/`：跨包公共类型（星历、观测、解、质量/完好性报告、状态枚举）。
- `ephemeris/`：广播星历（ICD-GPS-200 开普勒参数）→ 卫星 ECEF 位置、卫星钟差、
  信号传播期间地球自转（Sagnac）改正。
- `simulator/`：Walker 星座星历生成；按历元/卫星独立确定性子流生成伪距，叠加噪声、
  多径、仰角相关传播延迟与故障偏移。
- `position/`：迭代加权最小二乘 PVT（位置 + 接收机钟差）、WGS84 大地坐标与 ENU 旋转、
  DOP/协方差/残差统计、标称传播延迟改正模型。
- `raim/`：卡方全局一致性检验、单星标准化残差门限、前向搜索多星识别、剔除后重解、
  保护水平（HPL/VPL）、有界窗口慢漂移检测。
- `pipeline/`：异常观测链路（缺失/重复/乱序/钟跳/轻微抖动）、钟跳按历元有序检测与
  时间轴校正、重复到达标记、失败结果全局清理、有界接收通道、多通道并发引擎、
  确定性回放与重启状态重建。
- `store/`：GOB 原子落盘、按通道幂等提交、重启恢复。
- `integration/`：跨包端到端测试。

## 坐标系定义

- 卫星与接收机位置均在 **ECEF 地心地固直角坐标系**（X、Y、Z，单位米），采用
  **WGS84** 椭球；地球自转角速度与引力常数见 `ephemeris` 常量。
- 定位输出同时给出 **WGS84 大地坐标**：纬度、经度（度）与椭球高 `HeightM`（米）。
- 几何精度因子与保护水平在以接收机近似位置为原点的 **站心 ENU 坐标系** 中计算，
  由 ECEF→ENU 旋转矩阵（`position.ENUMatrix`）从 ECEF 协方差变换得到。
- 时间基准为 UTC `time.Time`；星历周内秒由 GPS 历元（1980-01-06）起算并按 GPS 周取模。

## 误差模型（模拟器）

伪距构造（`simulator.Simulator.Epoch`）：

```
rho_s = |R_recv - R_sat(tx) · Rz(ωe·τ)|      几何距离（含 Sagnac 旋转）
        - c·(δt_sat)                            卫星钟差（含相对论改正）
        + d_tropo(el) + d_iono(el)              仰角相关标称传播延迟
        + ε_thermal + b_multipath              高斯噪声与多径
        + f_fault(epoch)                        故障偏移
```

- 热噪声：每历元每卫星独立的零均值高斯，标准差 `NoiseSigma`（米）。
- 多径：同样为零均值高斯（确定性独立子流），标准差 `MultipathSigma`（米），
  代表本地多径偏差，已计入测量误差预算。
- 传播延迟：天顶标称值 `TropoDelay`/`IonoDelay`，随仰角按 `1/sin(el)` 放大，
  5° 截止（`position.DelayModel`）；解算器使用**同一标称模型**改正，保证自洽。
- 随机数：`rand.NewSource(seed ^ epoch·a ^ satIndex·b)`，每个 (历元, 卫星) 组合独立，
  固定种子下逐历元逐项一致。

故障模型（`simulator.Fault`，按 `[StartEpoch, EndEpoch]` 闭区间生效）：

- `FaultStep`：单星恒定阶跃 `Magnitude`（米）。
- `FaultMulti`：多颗卫星同时叠加相同阶跃。
- `FaultDrift`：从起始历元起线性增长 `Rate·(epoch-StartEpoch)`（米/历元）。

## 定位解算

- 迭代加权最小二乘，未知量为 ECEF 三维位置与接收机钟差（米），观测为伪距。
- 每轮按当前估计计算信号发射时刻、Sagnac 旋转、卫星钟差与仰角延迟改正，线性化迭代，
  位移增量小于 `ToleranceM` 即收敛。
- 协方差取加权法方程逆并乘 `SigmaRange²`（米²），对角为正；`sigma0` 为验后单位权中误差。
- **失败状态可区分，不输出无依据位置**：
  - `FailureInsufficientGeometry`：健康可见星少于 `MinSatellites`（默认 4）或法方程奇异；
  - `FailureNoConvergence`：达到 `MaxIterations` 仍未收敛；
  - `FailureNoUsableSatellites`：存在观测但全部无效（如伪距 ≤ 0）。

## 精度与完好性输出

每个历元输出（`model.QualityReport` / `model.IntegrityReport`）：

- DOP：`GDOP/PDOP/HDOP/VDOP/TDOP`，由 ENU 协方差对角项导出；几何越差 DOP 越大
  （有自动化测试 `TestDOPVariesWithGeometry`）。
- 残差：验后残差 RMS、最大残差、`Sigma0`、完整 4×4 协方差。
- 完好性：全局统计量、阈值、自由度、故障星识别结果、HPL/VPL 与告警限、慢漂移标志。

### RAIM 判定阈值（均可配置）

1. **全局一致性检验**：统计量 `T = Σ (v_s/σ)²`，健康假设下服从自由度 `n-4` 的卡方分布；
   阈值取 `χ²(1-Pfa, n-4)`。`FixedThreshold > 0` 时改用固定无量纲卡方阈值。
2. **单星识别门限**：前向搜索只剔除标准化残差 `|v_s|/σ ≥ SingleSatZThreshold` 的卫星，
   防止健康卫星因偶发噪声被误剔除（默认 4.5）。
3. **前向搜索**：逐颗剔除最可疑卫星并在剩余集合重解，直到统计量回到门限以下；最多剔除
   `MaxFaults` 颗，剔除后剩余卫星少于 `MinSatellites`（定位门限，默认 4）时停止。
4. **慢漂移**：有界窗口（`DriftWindow` 历元）内对每颗卫星的归一化残差做线性回归，
   斜率显著性超过 `DriftThreshold` 判为漂移；故障确认后清空窗口，避免残差污染后续历元。
5. **保护水平**：`HPL/VPL = k·σ_H/V`，其中 `k = Φ⁻¹(1-Pfa/2)`，σ 由 ENU 协方差给出；
   HPL > HAL 或 VPL > VAL 时解标记为降级。
6. 几何不足（健康星 < RAIM `MinSatellites`，默认 5）时输出 `RAIMUnavailable`，
   绝不伪造完好性结论。

### 解状态语义

- `StatusOK`：定位成功且完好性正常。
- `StatusDegraded`：定位有效但经历故障剔除、RAIM 不可用或保护水平超限。
- `StatusFailed`：检测到故障却无法安全剔除、几何/收敛失败，或整历元缺失占位。

### 失败状态的输出清理约定（全局一致）

失败历元**不得携带任何坐标、精度数值或完好性识别数值**，杜绝“报告失败却同时给出一套
明显偏离真值的坐标/把健康星列为故障星”的自相矛盾输出。该约定作用于**所有失败路径**
（几何不足、不收敛、无可用卫星、检测到故障但无法安全剔除、整历元缺失占位）：

- `PositionECEF`、`Geo`、`ClockBiasM` 清零；
- `UsedSats`、`ExcludedSats` 置空；
- `QualityReport` 整体清零（DOP、残差、`Sigma0`、协方差均不输出）；
- `IntegrityReport.Status = RAIMUnavailable`，`FaultSatellites` 显式置空，
  `Statistic/Threshold/DegreesFreedom/HPL/VPL/DriftDetected` 清零；
- `Status`、`FailureReason` 与链路标记（如 `FlagEpochMissing`）照常保留，
  告警限 `HAL/VAL` 作为配置项保留。

## 异常观测链路（`pipeline.LinkConfig`）

| 异常 | 配置 | 处理与输出标记 |
| --- | --- | --- |
| 整历元缺失 | `MissingWhole` | 链路不投递；通道按乱序容忍窗口确认缺口后提交失败占位，`FlagEpochMissing` |
| 部分卫星缺失 | `MissingSats` | 以剩余卫星解算，`FlagObservationMissing` |
| 重复到达 | `Duplicates` | 重复包被抑制、**绝不第二次提交**；对应历元最终结果带 `FlagDuplicateSuppressed` |
| 时间戳乱序 | `ReorderSwaps` | 在容忍窗口内缓冲重排；晚到包带 `FlagOutOfOrder` |
| 接收机钟跳 | `ClockJumps` | 接收机时间基准的持续阶跃；仅跳变发生历元带 `FlagClockJump` |
| 一次性轻微抖动 | `ClockJitter` | 只作用于单个历元的时间戳抖动；低于容差不标记 |

### 接收机时间异常（钟跳）处理规则

钟跳是**接收机时间基准**的阶跃（正跳/负跳均可），不是卫星故障；观测本身仍在标称历元
时刻产生，只是接收机加盖的时间戳偏移。通道严格按**历元序号有序**检测（与数据包到达
顺序无关），规则如下：

- 比较相邻历元打戳增量与标称历元间隔 `IntervalHint`；增量偏差的绝对值**严格大于**
  容差 `ClockJumpToleranceNS`（默认 `IntervalHint/2`）才判为跳变。
- 标记**只归属到真正发生跳变的那一个历元**；跳变之后的持续偏移不会连带标记紧邻历元，
  除非那些历元自身又发生跳变。连续多次跳变会分别标记到各自历元。
- 低于容差的轻微时间抖动（`ClockJitter`）不判为跳变。
- 解算时把接收时刻**校正回标称时间轴**（`t0 + 序号·IntervalHint`），卫星几何与残差
  恢复正常水平，因此钟跳历元给出误差处于正常噪声水平的可信定位；时间异常**不触发**
  RAIM 故障识别与卫星剔除，健康卫星不会被误判为故障星。
- 解结果的 `Epoch` 仍保留接收机实际加盖（可能含跳变）的原始时间戳，`FlagClockJump`
  与解算所用的校正时刻相互独立可追溯。
- 乱序投递只改变到达顺序，不改变钟跳标记的历元归属。

### 重复到达标记语义

- 通道按历元序号去重：同一序号的第二个及以后的数据包一律抑制，**绝不会被第二次提交**。
- 无论重复包在该历元提交**之前**还是**之后**到达（哪怕已超出内存乱序窗口），该历元
  最终提交的结果都带 `FlagDuplicateSuppressed`：
  - 提交前到达：并入该历元待处理数据，提交时一次性带上标记；
  - 提交后、乱序窗口内到达：原地补标到已提交结果；
  - 提交很久之后（超出内存窗口）到达：以持久化提交记录为权威识别为重复并补标，
    不新增提交（需使用 `store.Store`；新增接口 `Store.AddFlag` 幂等）。

### 标记独立共存

各类标记互相独立、互不覆盖，同一历元可同时携带多个标记（例如钟跳 + 重复抑制、
钟跳 + 部分缺失、乱序 + 重复）。钟跳标记的归属始终由实际发生跳变的历元决定，
不因与乱序/缺失/重复叠加而改变或产生误报。

## 并发与资源边界

- `pipeline.Engine` 支持多通道并发（`go test -race` 验证）；每通道持有独立的处理游标、
  乱序缓存、去重集合、时钟基准与 RAIM/漂移状态；一个通道的钟跳、重复到达或卫星故障
  不影响其他通道的状态、标记与完好性结论。持久化 `Store` 在落盘时编码独立深拷贝，
  与各通道对结果对象的并发修改解耦，全部并发测试在 `-race` 下通过。
- 状态**有界**：`pending`/`seen` 仅保留 `ReorderTolerance` 窗口内序号，慢漂移历史长度
  不超过 `DriftWindow`；内存与计算开销不随历元总数增长（`TestBoundedState`）。

## 确定性回放与重启恢复

- 固定 `Seed` 与相同输入序列时，每历元位置、质量标记、完好性结论逐项一致
  （`TestDeterminism`）。对残差的所有求和均按有序卫星列表进行，避免 Go map 随机遍历
  导致的浮点顺序差异。
- 结果通过 `store.Store` 原子（临时文件 + rename）GOB 落盘；`Commit` 幂等，重启后
  `NewChannel` 自动从最后已提交序号恢复。重放时恢复点之前的已提交历元以完全相同的
  顺序“只重建检测状态”（时钟基准与慢漂移历史），不产生也不重复提交结果；未完成历元
  按既定策略重算，已提交历元不会重复提交也不会跳过（`TestRestartRecomputesPending`、
  `TestRestartRecomputeWithAnomalies`）。含钟跳或重复到达的历元在重启重算后，其状态、
  标记组合与完好性结论与首次计算逐项一致。
- 历史持久化文件与提交游标**无需迁移**：本次修复只新增配置字段与 `Store.AddFlag`
  接口，不改变 GOB 结构、观测投递方式（`Ingest`/`DeliveryPlan`）与提交接口
  （`Commit`/`RunPlayback`/`Resume`），已有调用代码无需修改。

## 全部可配置项

`position.Config`：

| 字段 | 默认 | 含义 |
| --- | --- | --- |
| `MinSatellites` | 4 | 定位所需最少健康卫星 |
| `MaxIterations` | 20 | 最小二乘最大迭代次数 |
| `ToleranceM` | 1e-4 | 收敛位移门限（米） |
| `SigmaRange` | 3.0 | 标称伪距测量标准差（米），兼作权值与协方差尺度 |
| `Delay.TropoZenith` / `Delay.IonoZenith` | 0 | 天顶对流层/电离层标称延迟（米） |

`raim.Config`：

| 字段 | 默认 | 含义 |
| --- | --- | --- |
| `ProbabilityFalseAlarm` | 1e-6 | 全局检验虚警概率（卡方分位与保护水平 k 值） |
| `SigmaRange` | 3.0 | 标称伪距标准差（米） |
| `HAL` / `VAL` | 35 / 50 | 水平/垂直告警限（米） |
| `MaxFaults` | 2 | 最多尝试剔除的卫星数 |
| `MinSatellites` | 5 | 完好性可用所需最少卫星 |
| `DriftWindow` | 8 | 慢漂移检测窗口（历元） |
| `DriftThreshold` | 4.0 | 慢漂移归一化斜率门限 |
| `SingleSatZThreshold` | 4.5 | 单星识别标准化残差门限 |
| `FixedThreshold` | 0（自动） | 固定无量纲卡方阈值，>0 时覆盖自动分位数 |

`simulator.Config`：`Seed`、`StartTime`、`Interval`、`NumEpochs`、`NoiseSigma`、
`MultipathSigma`、`TropoDelay`、`IonoDelay`、`Faults`（见“误差模型”）。

`pipeline.Config`：

| 字段 | 默认 | 含义 |
| --- | --- | --- |
| `ID` | — | 接收通道标识 |
| `IntervalHint` | 0 | 标称历元间隔（纳秒）；>0 时启用钟跳检测并作为标称时间轴 |
| `ReorderTolerance` | 0 | 乱序容忍窗口（历元数） |
| `ClockJumpToleranceNS` | `IntervalHint/2` | 钟跳判定容差（纳秒），<=0 取默认值 |
| `Position` / `RAIM` | — | 内嵌的解算与完好性配置 |

钟跳容差也可在运行中通过 `Channel.SetClockJumpTolerance(ns)` 修改，变更只影响之后
处理的历元，不回溯重判。`Store.AddFlag(channel, index, flag)` 为幂等的标记追加接口，
用于在已提交结果上补“重复到达”等标记，不改变既有提交结构。

## 测试覆盖

- 定位精度与失败原因区分：`integration/pvt_test.go`、`integration/failure_test.go`
- 故障注入与排除（单星/多星/慢漂移、无误报）：`raim/raim_test.go`
- 观测与时钟异常、确定性回放、并发通道：`pipeline/pipeline_test.go`、`bounded_test.go`
- 重启恢复与未完成历元重算：`pipeline/restart_test.go`、`store/store_test.go`
- 异常观测链路验收：`pipeline/anomaly_test.go`，覆盖
  正向/负向/连续钟跳、容差内轻微抖动不误报、可配置容差、
  失败历元全局清理、重复到达在提交前/提交后/持久化迟到三种时序、
  钟跳与乱序/缺失/重复的标记独立共存、多通道并发隔离（`-race`）、
  含异常的确定性回放与重启重算逐项一致（不依赖随机游程）。

> 说明：这是一个教学/算法演示系统。误差模型、阈值与保护水平采用了简化的解析近似
> （如 1/sin 仰角映射、Wilson–Hilferty 卡方分位、协方差对角化保护水平），并非适航级
> GNSS 完好性实现。
