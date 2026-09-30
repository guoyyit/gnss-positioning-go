# gnss-positioning-go

本地、离线的卫星导航单点定位（SPP）与接收机自主完好性监测（RAIM）服务。
所有观测均由内置星座模拟器确定性生成，支持注入随机噪声、多径偏差、
传播延迟偏差、卫星阶跃/漂移故障以及接收机时钟跳变；不依赖任何外部
定位服务或真实卫星信号。

## 功能

- 由 GPS 风格广播星历（开普勒根数）计算卫星 ECEF 位置与卫星钟差。
- 迭代最小二乘解算接收机 ECEF 三维位置、接收机钟差，并输出经纬度与椭球高。
- 几何不足、秩亏或不收敛时返回可区分的失败状态，不输出无依据位置。
- 每次解算附带 GDOP/PDOP/HDOP/VDOP/TDOP、残差 RMS/最大值、4x4 误差协方差。
- RAIM：SSE 卡方全局检测、贪心+双星组合的故障检测与排除（FDE）、
  残差历史线性斜率漂移检测、水平/垂直保护水平（HPL/VPL），支持单星、
  多星与缓慢漂移故障；检测到但无法隔离时返回明确失败状态。
- 异常观测链路：缺失、重复 PRN、重复历元（幂等）、时间戳乱序、
  接收机时钟跳变，均以独立质量位标记且不产生错误定位。
- 多通道并发，通道间状态完全隔离；同通道并发提交串行化且去重。
- 残差历史、近期解、排除集合均为定长，内存/计算开销不随历元数增长。
- 固定随机种子 + 相同输入序列下，逐历元位置、质量位与完好性结论完全可复现。
- 每通道 JSON-Lines WAL 持久化；重启后已提交历元直接复用，未完成历元重算，
  不重复提交、不跳过。

## 坐标系与时间

- 地固系：ECEF，单位米，WGS-84 椭球（a=6378137 m，f=1/298.257223563）。
- 地理坐标：`LLA{Lat, Lon}` 为弧度，`Alt` 为 WGS-84 椭球高，单位米。
- 卫星轨道传播包含地球自转改正（`ΩDot - ωe`），与模拟器生成伪距时
  使用的几何完全一致。
- 时间：`GNSSTime.Seconds` 为自 1980-01-06T00:00:00Z 起的单调仿真秒；
  本服务不处理闰秒、UTC 转换或真实 GPS 周翻转。

## 误差模型

模拟器对每颗可见卫星生成
`ρ = 几何距离 + 接收机钟差 − 卫星钟差 + 噪声 + 多径 + 延迟偏差 + 故障偏移`：

- `NoiseSigma`：零均值高斯测距噪声（米）。
- `MultipathSigma`：零均值高斯多径偏差（米）。
- `DelayBias[PRN]`：每颗星固定传播延迟偏差（米，正值表示延迟）。
- 接收机钟差 = `RxClockBias0 + RxClockDrift × t`，另可经 `ClockJumps[epoch]`
  注入历元级阶跃跳变。
- 随机数按 `Seed + epoch×1000003` 取种子，历元之间相互独立且确定。
- 故障 `FaultSpec{Bias, Rate}`：在 `[StartEpoch, EndEpoch]` 内注入
  `Bias + Rate × 已持续秒数` 的伪距偏差，分别覆盖阶跃与缓慢漂移。
- 仰角低于 `MaskElevation`（默认 5°）的卫星不产生观测。

协方差采用 `Q = (HᵀH)⁻¹ σ²`，其中后验 `σ² = SSE/(n−4)`；当 SSE
无法估计时回退为配置的 `SigmaUERE²`。DOP 由 ECEF 协方差经本地
ENU 旋转得到：HDOP=√(σE²+σN²)，VDOP=σU，PDOP=√(HDOP²+VDOP²)，
TDOP=√Qtt，GDOP=√tr(Q)（无量纲，乘 `SigmaUERE` 得米级精度）。

## 完好性判定

- 全局检测统计量：`T = √SSE / σ`，阈值 `√(χ²_{n−4,1−pfa})`，
  默认虚警率 `pfa=1e-5`（Wilson–Hilferty 卡方分位数），
  也可用 `GlobalThreshold` 直接给定 SSE 门限（米²）。
- FDE：超阈后逐星排除最小化 SSE；单星排除存在异号多故障模糊时，
  对所有两星子集组合搜索，SSE 回到门限以下即判定恢复。最多排除
  `MaxFaults` 颗，剩余少于 4 颗不再尝试。
- 漂移检测：仅在全局 SSE 未超阈时，对每颗星最近 `DriftWindowEpochs`
  个残差做最小二乘斜率拟合，斜率绝对值 ≥ `DriftRateLimit`（米/历元）
  且当前残差绝对值 ≥ `SlipThreshold`（米）时标记；候选星联合排除后
  SSE 正常才确认，避免污染基线上的误剔除。
- 保护水平：`HPL/VPL = 2σ·HDOP/VDOP + 最大斜率 × 偏差门限`，
  偏差门限取 `HorizontalAlert/VerticalAlert`（默认 500 m）。可见星
  少于 5 颗时 `RAIMResult.Available=false`。
- 排除只在成功恢复后持久化，且有 `HistoryEpochs` 历元的过期窗口；
  无法隔离的检测返回 `StatusRAIMDetectionFailure`，不输出定位结果。

## 可配置项

`SimConfig`：`Seed`、`StartTime`、`EpochStep`、`NumSatellites`、
`ReceiverLLA`、`RxClockBias0`、`RxClockDrift`、`NoiseSigma`、
`MultipathSigma`、`DelayBias`、`Faults`、`MaskElevation`、`ClockJumps`。

`SolverConfig`：`MinSatellites`（默认 4）、`MaxIterations`（默认 20）、
`ConvergenceTol`（默认 1e-4 m）、`SigmaUERE`（默认 1 m）、`RAIM`。

`RAIMConfig`：`Enabled`、`MaxFaults`（默认 2）、`GlobalThreshold`
（0 表示按 pfa=1e-5 自动）、`SlipThreshold`（默认 5σ）、
`DriftWindowEpochs`（默认 5）、`DriftRateLimit`、
`HorizontalAlert`、`VerticalAlert`、`SigmaUERE`。

`ChannelConfig`：`ID`、`Solver`、`ClockJumpThreshold`（默认 1000 m）、
`HistoryEpochs`（默认 10，决定残差缓冲、排除过期与近期解容量）。

## 持久化与恢复

- 每通道一个追加型 JSON-Lines 文件 `channel-<ID>.wal`，每次提交
  `fsync` 落盘；相同 `(channel, epoch)` 的重复提交直接返回历史结果。
- 重启后 `GetChannel` 重放该通道 WAL，已提交历元标记为 committed；
  调用方继续提交未完成历元即可，不会跳过或重复。

## 运行

```bash
go build ./...
go test ./...
go run ./cmd/gnss-sps -reset -epochs 30 -fault-prn 7 -fault-epoch 8
```

常用命令行参数：`-seed`、`-epochs`、`-state`、`-fault-prn`、
`-fault-epoch`、`-fault-m`、`-no-raim`、`-reset`。

## 代码结构

- `gnss/coord.go`：WGS-84/ECEF/ENU 与向量运算。
- `gnss/ephemeris.go`：广播星历轨道与卫星钟差。
- `gnss/simulator.go`：确定性星座与观测/故障模拟。
- `gnss/solver.go`：最小二乘解算、协方差、DOP 与残差。
- `gnss/raim.go` / `gnss/stats.go`：完好性检测、FDE、保护水平、统计分位数。
- `gnss/channel.go`：通道状态、异常标记、钟跳检测与有界历史。
- `gnss/persist.go` / `gnss/service.go`：WAL 持久化与并发多通道服务。
- `cmd/gnss-sps`：本地演示入口。
