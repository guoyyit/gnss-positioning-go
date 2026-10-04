package pipeline

import (
	"sync"
	"time"

	"github.com/guoyyit/gnss-positioning-go/model"
	"github.com/guoyyit/gnss-positioning-go/position"
	"github.com/guoyyit/gnss-positioning-go/raim"
	"github.com/guoyyit/gnss-positioning-go/store"
)

// Config 为接收通道配置。
type Config struct {
	ID               string
	IntervalHint     int64 // 标称历元间隔，纳秒
	ReorderTolerance int   // 乱序容忍窗口（历元数）
	// ClockJumpToleranceNS 为接收机时间基准阶跃判定容差（纳秒）。
	// <=0 时取标称历元间隔的一半；运行中可通过 SetClockJumpTolerance 修改，
	// 修改只影响后续历元的判定。
	ClockJumpToleranceNS int64
	Position             position.Config
	RAIM                 raim.Config
}

// Channel 为单个独立接收机通道，维护有界状态。
type Channel struct {
	mu    sync.Mutex
	cfg   Config
	store *store.Store

	drift *raim.DriftState

	// 有序处理游标：下一个待提交的历元序号。
	nextIndex int
	// 提交恢复点：序号小于该值的历元在重放时只重建检测状态、不重复提交。
	// 全新通道为 0；从持久化恢复时为“最近已提交序号 + 1”。
	commitFrom int
	// 乱序窗口内已到达但尚未轮到处理的历元，按序号索引，容量有界。
	pending map[int]*Delivery
	// 已提交/已见序号集合（仅保留窗口内），用于重复抑制。
	seen       map[int]bool
	maxArrival int

	// 重复到达：序号 -> 至少出现过一次重复包。仅保留有界窗口。
	duplicateArrivals map[int]bool
	// 已提交解（无存储模式下用于给提交后到达的重复包补标记）。
	// 保留固定宽度的有界窗口，不受最新到达序号推进影响。
	committed map[int]*model.Solution

	// 接收机时间基准跟踪（按历元序号有序推进，与到达顺序无关）。
	clockRefIndex   int       // 上一已处理历元序号
	clockRefNominal time.Time // 上一历元对齐回标称时间轴后的接收时刻（解算基准）
	clockRefStamped time.Time // 上一历元接收机实际加盖的时间戳（增量检测基准）
}

// NewChannel 构造通道，并从持久化存储恢复提交游标。
func NewChannel(cfg Config, st *store.Store) *Channel {
	c := &Channel{
		cfg:               cfg,
		store:             st,
		drift:             raim.NewDriftState(cfg.RAIM),
		pending:           map[int]*Delivery{},
		seen:              map[int]bool{},
		duplicateArrivals: map[int]bool{},
		committed:         map[int]*model.Solution{},
		clockRefIndex:     -1,
		nextIndex:         0,
	}
	if st != nil {
		c.commitFrom = st.LastIndex(cfg.ID) + 1
	}
	return c
}

// LastCommitted 返回最近一次已提交历元序号（无记录为 -1）。
func (c *Channel) LastCommitted() int {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.nextIndex - 1
}

// SetClockJumpTolerance 修改钟跳判定容差（纳秒），<=0 恢复为默认值。
// 变更只影响之后处理的历元，不回溯重判。
func (c *Channel) SetClockJumpTolerance(ns int64) {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.cfg.ClockJumpToleranceNS = ns
}

// Ingest 接收一个可能乱序或重复的数据包。
// 返回 false 表示该序号此前已到达（重复包）；重复包被抑制且不会被第二次提交。
// 无论重复包在该历元提交之前还是之后到达，对应历元的最终结果都会带上
// FlagDuplicateSuppressed 标记。
func (c *Channel) Ingest(d Delivery) bool {
	c.mu.Lock()
	defer c.mu.Unlock()
	idx := d.Epoch.Index
	if c.seen[idx] {
		c.duplicateArrivals[idx] = true
		c.markDuplicate(idx)
		return false
	}
	// 超出内存有界窗口的迟到包：以持久化提交记录为权威判断是否为重复。
	// 已提交则补标并抑制，避免“提交后很久才到的重复包”被当作新历元。
	if c.store != nil && idx < c.nextIndex && c.store.Solution(c.cfg.ID, idx) != nil {
		_, _ = c.store.AddFlag(c.cfg.ID, idx, model.FlagDuplicateSuppressed)
		return false
	}
	c.seen[idx] = true
	if idx < c.maxArrival {
		d.Flags = addFlag(d.Flags, model.FlagOutOfOrder)
	}
	if idx > c.maxArrival {
		c.maxArrival = idx
	}
	c.pending[idx] = &d
	return true
}

// trim 在一次 Drain 结束时按提交游标回收所有有界窗口外的状态。
// 只在 Drain 末尾调用：此时 nextIndex 已是提交后游标，窗口下界计算准确。
func (c *Channel) trim() {
	tolerance := c.cfg.ReorderTolerance
	if tolerance < 0 {
		tolerance = 0
	}
	// GC 水位跟随提交游标。Drain 后 nextIndex=N，最近一个完整乱序窗口
	// [N-1-tol, N-1] 内的已见/已提交历元必须保留，这样“提交后到达”的
	// 重复包无论在窗口内何时到达都能被识别并补标；有序到达的未来历元不推进该水位。
	lowWater := c.nextIndex - tolerance - 1
	for idx := range c.pending {
		// pending 只清除已处理项；未处理的缺口历元由 Drain 的超时占位逻辑接管，
		// 不能因最新到达序号推进而被提前删除。
		if idx < c.nextIndex {
			delete(c.pending, idx)
		}
	}
	for idx := range c.seen {
		if idx < lowWater {
			delete(c.seen, idx)
		}
	}
	for idx := range c.duplicateArrivals {
		if idx < lowWater {
			delete(c.duplicateArrivals, idx)
		}
	}
	for idx := range c.committed {
		if idx < lowWater {
			delete(c.committed, idx)
		}
	}
}

// markDuplicate 为一个待处理或已提交历元的最终结果补重复到达标记。
func (c *Channel) markDuplicate(idx int) {
	if p := c.pending[idx]; p != nil {
		p.Flags = addFlag(p.Flags, model.FlagDuplicateSuppressed)
	}
	// 已提交结果的内容修改统一经 store（store 锁与并发 flush 的拷贝互斥）。
	if c.store != nil {
		if ok, _ := c.store.AddFlag(c.cfg.ID, idx, model.FlagDuplicateSuppressed); ok {
			if s := c.committed[idx]; s != nil {
				s.Flags = addFlag(s.Flags, model.FlagDuplicateSuppressed)
			}
		}
		return
	}
	if s := c.committed[idx]; s != nil {
		s.Flags = addFlag(s.Flags, model.FlagDuplicateSuppressed)
	}
}

// Drain 按序号处理所有就绪历元，返回新提交结果；缺失历元以 FlagEpochMissing 占位提交。
// 重启恢复点之前的历元只重建检测状态，不产生也不重复提交结果。
func (c *Channel) Drain() []*model.Solution {
	c.mu.Lock()
	defer c.mu.Unlock()

	var out []*model.Solution
	tolerance := c.cfg.ReorderTolerance
	if tolerance < 0 {
		tolerance = 0
	}

	for {
		rehearse := c.nextIndex < c.commitFrom
		d, ok := c.pending[c.nextIndex]
		if !ok {
			if rehearse {
				// 重放恢复点之前的缺口：若该历元已持久化（含缺失占位），
				// 对齐时钟基准并推进，不重算不重提；否则等待其观测到达。
				if c.store != nil && c.store.Solution(c.cfg.ID, c.nextIndex) != nil {
					c.skipClockReference()
					c.nextIndex++
					continue
				}
				break
			}
			// 恢复点之后：找到窗口内最远到达序号；缺口无法被迟到数据填补时提交缺失占位。
			maxArrival := c.nextIndex - 1
			for idx := range c.pending {
				if idx > maxArrival {
					maxArrival = idx
				}
			}
			if maxArrival-c.nextIndex >= tolerance {
				out = append(out, c.commitMissing())
				continue
			}
			break
		}

		if rehearse {
			// 已提交历元：以完全相同的顺序重算以重建钟跳/漂移等检测状态，丢弃结果。
			c.rehearse(d)
			delete(c.pending, c.nextIndex)
			c.nextIndex++
			continue
		}

		sol := c.process(d)
		if c.duplicateArrivals[c.nextIndex] {
			sol.Flags = addFlag(sol.Flags, model.FlagDuplicateSuppressed)
		}
		if c.store != nil {
			stored := cloneSolution(sol)
			local := cloneSolution(sol)
			c.committed[c.nextIndex] = local
			// store 持有独立副本（仅在 store 锁内变动）；返回给调用方的结果
			// 与本地镜像共享同一指针，便于“提交后补标”反映到最终结果。
			if committed, err := c.store.Commit(c.cfg.ID, stored); err == nil && committed {
				out = append(out, local)
			}
		} else {
			out = append(out, sol)
			c.committed[c.nextIndex] = sol
		}
		delete(c.pending, c.nextIndex)
		c.nextIndex++
	}
	c.trim()
	return out
}

func (c *Channel) commitMissing() *model.Solution {
	sol := &model.Solution{
		Channel:       c.cfg.ID,
		Index:         c.nextIndex,
		Status:        model.StatusFailed,
		FailureReason: model.FailureNoUsableSatellites,
		Flags:         []model.QualityFlag{model.FlagEpochMissing},
		Integrity: model.IntegrityReport{
			Status: model.RAIMUnavailable,
			HAL:    c.cfg.RAIM.HAL,
			VAL:    c.cfg.RAIM.VAL,
		},
	}
	c.nextIndex++
	if c.store != nil {
		_, _ = c.store.Commit(c.cfg.ID, sol)
	}
	return sol
}

// cloneSolution 返回解的深拷贝，使持久化对象与调用方/通道内存解耦。
func cloneSolution(s *model.Solution) *model.Solution {
	cp := *s
	cp.Flags = append([]model.QualityFlag(nil), s.Flags...)
	cp.UsedSats = append([]model.SatID(nil), s.UsedSats...)
	cp.ExcludedSats = append([]model.SatID(nil), s.ExcludedSats...)
	cp.Integrity.FaultSatellites = append([]model.SatID(nil), s.Integrity.FaultSatellites...)
	return &cp
}

// detectClockJump 检测本历元接收机时间基准是否发生阶跃：比较相邻历元打戳
// 增量与标称历元间隔，增量偏差超过容差才判为跳变，且只标记发生跳变的历元，
// 跳变后的持续偏移不会连带标记后续历元。
// 返回观测在标称时间轴上的对应时刻（观测在标称时刻产生，跳变仅是接收机
// 打戳偏移），以及本历元自身是否发生跳变。
// 偏差不超过容差的轻微抖动不判为跳变，但解算时刻始终对齐标称时间轴。
//
// skipClockReference 用于重放状态重建：当某已提交历元没有可重放的观测包
// （如缺失占位）时，把时间基准参考点按序号前移一个标称间隔，保持与首次
// 有序处理完全一致。
func (c *Channel) detectClockJump(ep *model.EpochObs) (time.Time, bool) {
	if c.cfg.IntervalHint <= 0 {
		return ep.Epoch, false
	}
	if c.clockRefIndex < 0 {
		c.clockRefIndex = ep.Index
		c.clockRefNominal = ep.Epoch
		c.clockRefStamped = ep.Epoch
		return ep.Epoch, false
	}

	gap := time.Duration(int64(ep.Index-c.clockRefIndex) * c.cfg.IntervalHint)
	nominal := c.clockRefNominal.Add(gap)
	expectedStamped := c.clockRefStamped.Add(gap)
	incrementDeviation := ep.Epoch.Sub(expectedStamped)

	tolerance := c.cfg.ClockJumpToleranceNS
	if tolerance <= 0 {
		tolerance = c.cfg.IntervalHint / 2
	}

	jumped := abs64(int64(incrementDeviation)) > tolerance

	c.clockRefIndex = ep.Index
	c.clockRefNominal = nominal
	c.clockRefStamped = ep.Epoch
	return nominal, jumped
}

// skipClockReference 在无观测可重放（缺失占位）时，将时钟基准参考点按
// 标称历元间隔前移一个序号，等价于首次有序处理跳过该历元。
func (c *Channel) skipClockReference() {
	idx := c.nextIndex
	if c.clockRefIndex < 0 {
		// 尚未建立参考：以模拟器/来源的标称起点无法获知，直接标记占位序号。
		c.clockRefIndex = idx
		return
	}
	if idx <= c.clockRefIndex {
		return
	}
	gap := time.Duration(int64(idx-c.clockRefIndex) * c.cfg.IntervalHint)
	c.clockRefNominal = c.clockRefNominal.Add(gap)
	c.clockRefStamped = c.clockRefStamped.Add(gap)
	c.clockRefIndex = idx
}

// sanitizeFailed 清理失败历元的全部数值输出，保证“失败即无数值坐标/精度/完好性识别”。
// 保留状态、失败原因、链路标记与告警限配置；故障星列表显式清空（不可用）。
func sanitizeFailed(sol *model.Solution) {
	sol.PositionECEF = model.Vec3{}
	sol.Geo = model.GeoPoint{}
	sol.ClockBiasM = 0
	sol.UsedSats = nil
	sol.ExcludedSats = nil
	sol.Quality = model.QualityReport{}
	sol.Integrity.Status = model.RAIMUnavailable
	sol.Integrity.FaultSatellites = nil
	sol.Integrity.Statistic = 0
	sol.Integrity.Threshold = 0
	sol.Integrity.DegreesFreedom = 0
	sol.Integrity.HPL = 0
	sol.Integrity.VPL = 0
	sol.Integrity.DriftDetected = false
}

func addFlag(flags []model.QualityFlag, target model.QualityFlag) []model.QualityFlag {
	if hasFlag(flags, target) {
		return flags
	}
	return append(flags, target)
}

func hasFlag(flags []model.QualityFlag, target model.QualityFlag) bool {
	for _, f := range flags {
		if f == target {
			return true
		}
	}
	return false
}

func abs64(v int64) int64 {
	if v < 0 {
		return -v
	}
	return v
}

// rehearse 重放一个已提交历元以重建通道检测状态（时钟基准与慢漂移历史），
// 丢弃解算结果。它与首次有序处理调用完全相同的检测路径，且每个已提交历元
// 严格重放一次，因此重启后对后续历元的判定与首次计算逐项一致。
func (c *Channel) rehearse(d *Delivery) {
	c.process(d)
}

func (c *Channel) process(d *Delivery) *model.Solution {
	ep := d.Epoch
	flags := append([]model.QualityFlag{}, d.Flags...)

	// 钟跳检测严格按历元序号有序进行（process 只在 Drain 中按序调用），
	// 因而标记归属只取决于实际发生跳变的历元，与数据包到达顺序无关。
	solveAt, jumped := c.detectClockJump(ep)
	if jumped {
		flags = addFlag(flags, model.FlagClockJump)
	}

	// 时间基准异常不进入完好性故障识别：用校正回标称时间轴的接收时刻解算，
	// 残差恢复正常水平，健康卫星不会被误判为故障星、不会被误剔除。
	res := raim.Evaluate(ep.Obs, solveAt, c.cfg.Position, c.cfg.RAIM, c.drift)

	sol := &model.Solution{
		Channel: c.cfg.ID,
		Index:   ep.Index,
		Epoch:   ep.Epoch, // 保留接收机加盖（可能含跳变）的原始时间戳
		Flags:   flags,

		PositionECEF: res.Position,
		ClockBiasM:   res.ClockBiasM,
		UsedSats:     res.UsedSats,
		ExcludedSats: res.ExcludedSats,
	}

	if res.OK {
		sol.Geo = position.ECEFToGeo(res.Position)
		sol.Quality = position.BuildQualityReport(res.UsedSats, res.Residuals, res.Covariance, res.Sigma0, sol.Geo)
	}

	switch res.Status {
	case model.RAIMOK:
		sol.Status = model.StatusOK
	case model.RAIMFaultExcluded:
		sol.Status = model.StatusDegraded
		sol.Flags = addFlag(sol.Flags, model.FlagFaultExcluded)
	case model.RAIMFaultDetected:
		// 检测到但无法安全剔除：不输出无依据的位置。
		sol.Status = model.StatusFailed
		sol.FailureReason = model.FailureNoConvergence
	case model.RAIMUnavailable:
		if res.OK {
			sol.Status = model.StatusDegraded
			sol.Flags = addFlag(sol.Flags, model.FlagRAIMUnavailable)
		} else {
			sol.Status = model.StatusFailed
			sol.FailureReason = res.Reason
		}
	}

	sol.Integrity = model.IntegrityReport{
		Status:          res.Status,
		FaultSatellites: res.FaultSatellites,
		Statistic:       res.Statistic,
		Threshold:       res.Threshold,
		DegreesFreedom:  res.DegreesFreedom,
		HPL:             res.HPL,
		VPL:             res.VPL,
		HAL:             c.cfg.RAIM.HAL,
		VAL:             c.cfg.RAIM.VAL,
		DriftDetected:   res.DriftDetected,
	}

	if sol.Status == model.StatusFailed {
		// 全局一致：失败历元不携带坐标、精度数值与完好性识别数值。
		sanitizeFailed(sol)
	} else if sol.Integrity.HPL > sol.Integrity.HAL || sol.Integrity.VPL > sol.Integrity.VAL {
		if sol.Status == model.StatusOK {
			sol.Status = model.StatusDegraded
		}
	}

	return sol
}
