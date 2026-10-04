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
- `pipeline/`：异常观测链路（缺失/重复/乱序/钟跳）、有界接收通道、多通道并发引擎、
  确定性回放驱动。
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
- `StatusFailed`：检测到故障却无法安全剔除，或几何/收敛失败；此时不提供定位坐标。
  所有失败路径的坐标、精度、完好性数值一律清空、故障卫星列表为空，详见
  “异常观测链路”中的“失败状态的输出清理约定”。

## 异常观测链路（`pipeline.LinkConfig`）

| 异常 | 配置 | 处理与输出标记 |
| --- | --- | --- |
| 整历元缺失 | `MissingWhole` | 链路不投递；通道在输入流结束（`FlushEndOfStream`，`Engine.RunPlayback` 已内含）时确认缺口，提交失败占位 `FlagEpochMissing`。流中**不**提前确认缺口——容忍窗口内的迟到包可能恰好填补它 |
| 部分卫星缺失 | `MissingSats` | 以剩余卫星解算，`FlagObservationMissing` |
| 重复到达 | `Duplicates` | 通道按序号去重，重复包不产生第二个提交结果；该历元最终结果（无论重复包在提交之前还是之后到达）携带 `FlagDuplicateSuppressed`，标记幂等且会修订已落盘结果 |
| 时间戳乱序 | `ReorderSwaps` | 在容忍窗口内缓冲重排；到达时已见最远序号超出容忍窗口的真正晚到包带 `FlagOutOfOrder`，窗口内顺序打乱由重排吸收、不标记 |
| 接收机钟跳 | `ClockJumps` 或兜底检测 | 见下方“接收机钟跳处理规则” |

### 接收机钟跳处理规则

- 通道维护一个按**历元序号**（而非到达顺序）建立的标称时间网格，锚点为序号 0
  对应的标称时间；重启后由最早已提交历元重建，乱序晚到包会校准锚点。
- 历元严格按序号处理，因此钟跳标记始终归属真实发生跳变的历元，紧邻的后续历元
  不会被连带标记，正向与负向跳变同理；乱序投递不改变归属。
- 判定容差为 `Config.ClockJumpTolerance`（纳秒，缺失时默认 `IntervalHint/2`）：
  加盖时间戳相对标称网格的偏差绝对值超过容差才标记 `FlagClockJump`；容差以内的
  轻微时间抖动不标记。容差可通过 `Channel.SetClockJumpTolerance` 动态调整，
  只影响调整之后处理的历元。
- 时间基准异常不会被误判为卫星故障：解算一律使用该序号对应的标称时间（观测即按
  真实信号发射时刻构造），钟跳历元仍给出正常噪声水平的可信定位，RAIM 不会把健康
  卫星列为故障星，也不会触发故障剔除。

### 失败状态的输出清理约定

任何失败历元（几何不足、不收敛、无可用卫星、整历元缺失等**所有**失败路径）输出
全局一致：

- `Status = StatusFailed`，`FailureReason` 给出可区分原因；
- 不携带坐标：`PositionECEF`/`Geo`/`ClockBiasM` 全部清零；
- 不携带精度与完好性识别数值：`Quality` 清零，`Integrity.Statistic/Threshold/
  DegreesFreedom/HPL/VPL/DriftDetected` 清零，`FaultSatellites` 为空；
- `Integrity.Status = RAIMUnavailable`（保留 `HAL/VAL` 配置项），明确表示完好性
  不可用，绝不出现“失败状态 + 数值坐标/故障星列表”并存的结果；
- 链路标记（`FlagEpochMissing` 等）原样保留；`FailureTimeAnomaly` 预留给
  “拒绝时间异常定位”的失败路径。

### 标记组合语义

各标记相互独立、互不覆盖，同一历元可同时携带多个标记（如钟跳 + 重复抑制、
部分缺失 + 乱序）。标记按其实际触发条件幂等追加。

## 并发与资源边界

- `pipeline.Engine` 支持多通道并发（`go test -race` 验证）；每通道持有独立的处理游标、
  乱序缓存、去重集合、时间基准与 RAIM/漂移状态，一个通道的钟跳/重复/乱序不影响其他
  通道的状态、标记与完好性结论（`TestConcurrentAnomalyIsolation`）。
- 状态**有界**：`seen`/`committed` 仅保留 `ReorderTolerance` 窗口内序号；`pending`
  只缓存乱序窗口内尚未按序处理的到达包，在 `Drain`/`FlushEndOfStream` 处理后立即清空；
  慢漂移历史长度不超过 `DriftWindow`；开销不随历元总数增长（`TestBoundedState`）。

## 确定性回放与重启恢复

- 固定 `Seed` 与相同输入序列时，每历元位置、质量标记、完好性结论逐项一致
  （`TestDeterminism`）。对残差的所有求和均按有序卫星列表进行，避免 Go map 随机遍历
  导致的浮点顺序差异。
- 结果通过 `store.Store` 原子（临时文件 + rename）GOB 落盘；`Commit` 幂等，重启后
  `NewChannel` 自动从最后已提交序号恢复。未完成历元由调用方按既定策略重放重算，
  已提交历元不会重复提交，也不会跳过（`TestRestartRecomputesPending`）。
- 历史结果与提交游标**无需迁移**：`Solution` 结构仅新增枚举值（`FailureTimeAnomaly`，
  追加于枚举末尾），GOB 旧文件可直接读取；观测投递方式（`Ingest`/`Drain`）与提交接口
  （`Commit`）保持不变，已有调用代码无需修改（提交后重复包的标记修订走新增的
  `store.AnnotateFlags`，不改变 `Commit` 语义）。
- 重启后时间基准由最早已提交历元重建，包含钟跳/重复/缺失/乱序的未完成历元重算结果
  与首次完整计算逐项一致（`TestRestartPendingAnomaliesRecompute`）。

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

`pipeline.Config`：`ID`、`IntervalHint`（标称历元间隔，纳秒，用于钟跳检测与标称
时间网格）、`ReorderTolerance`（乱序容忍窗口，历元数）、`ClockJumpTolerance`
（钟跳判定容差，纳秒，缺省取 `IntervalHint/2`）、内嵌 `Position` 与 `RAIM` 配置。
运行期可用 `Channel.SetClockJumpTolerance` 调整容差，仅影响后续历元。

## 测试覆盖

- 定位精度与失败原因区分：`integration/pvt_test.go`、`integration/failure_test.go`
- 故障注入与排除（单星/多星/慢漂移、无误报）：`raim/raim_test.go`
- 观测与时钟异常、确定性回放、并发通道：`pipeline/pipeline_test.go`、`bounded_test.go`
- 正/负向与连续钟跳、钟跳容差配置与轻微抖动、钟跳+乱序归属、钟跳+缺失+重复标记组合、
  重复到达（提交前/提交后）、失败输出全局清空、多通道异常隔离、重启重算逐项一致与
  确定性深比较回放：`pipeline/anomaly_test.go`（`go test -race -count=2` 稳定复现，
  固定种子、不依赖随机游程）
- 重启恢复与未完成历元重算：`pipeline/restart_test.go`、`store/store_test.go`

> 说明：这是一个教学/算法演示系统。误差模型、阈值与保护水平采用了简化的解析近似
> （如 1/sin 仰角映射、Wilson–Hilferty 卡方分位、协方差对角化保护水平），并非适航级
> GNSS 完好性实现。
