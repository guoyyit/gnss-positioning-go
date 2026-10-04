package pipeline

import (
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
	// ClockJumpTolerance 为钟跳判定容差（纳秒）：相邻历元时间戳相对标称时间网格的
	// 偏差绝对值超过该值才判为钟跳；<=0 时取 IntervalHint/2。更新后只影响后续历元。
	ClockJumpTolerance int64
	Position           position.Config
	RAIM               raim.Config
}

// Channel 为单个独立接收机通道，维护有界状态。
type Channel struct {
	cfg   Config
	store *store.Store

	drift *raim.DriftState

	// 有序处理游标：下一个待提交的历元序号。
	nextIndex int
	// 乱序窗口内已到达但尚未轮到处理的历元，按序号索引，容量有界。
	pending map[int]*Delivery
	// 已见序号集合（仅保留窗口内），用于重复抑制。
	seen map[int]bool
	// 最近已提交结果（仅保留窗口内），供提交后才到达的重复包追加标记。
	committed  map[int]*model.Solution
	maxArrival int

	// 时间基准：anchor 为序号 0 对应的标称时间；由首个处理历元或最早已提交结果建立。
	anchor      time.Time
	anchorSet   bool
	anchorIndex int
}

// NewChannel 构造通道，并从持久化存储恢复提交游标。
func NewChannel(cfg Config, st *store.Store) *Channel {
	c := &Channel{
		cfg:       cfg,
		store:     st,
		drift:     raim.NewDriftState(cfg.RAIM),
		pending:   map[int]*Delivery{},
		seen:      map[int]bool{},
		committed: map[int]*model.Solution{},
		nextIndex: 0,
	}
	if st != nil {
		c.nextIndex = st.LastIndex(cfg.ID) + 1
		// 重启后由最早已提交历元重建标称时间基准，保证未完成历元重算结果一致。
		if cfg.IntervalHint > 0 {
			if sols := st.All(cfg.ID); len(sols) > 0 {
				first := sols[0]
				c.anchor = first.Epoch.Add(-time.Duration(first.Index) * time.Duration(cfg.IntervalHint))
				c.anchorSet = true
				c.anchorIndex = first.Index
			}
		}
	}
	return c
}

// LastCommitted 返回最近一次已提交历元序号（无记录为 -1）。
func (c *Channel) LastCommitted() int {
	return c.nextIndex - 1
}

// Ingest 接收一个可能乱序或重复的数据包。重复数据被抑制并以返回值标记：
// 重复包不会产生第二个提交结果，但该历元最终结果会携带 FlagDuplicateSuppressed。
func (c *Channel) Ingest(d Delivery) bool {
	idx := d.Epoch.Index
	if c.seen[idx] {
		c.markDuplicateSuppressed(idx)
		return false
	}
	c.seen[idx] = true
	if idx < c.maxArrival {
		// 该历元到达时，已见到的最远序号超出容忍窗口：它是真正的晚到包
		// （窗口内的顺序打乱由重排缓冲吸收，不标记）。
		tolerance := c.cfg.ReorderTolerance
		if tolerance < 0 {
			tolerance = 0
		}
		if c.maxArrival-idx > tolerance {
			d.Flags = addFlag(d.Flags, model.FlagOutOfOrder)
		}
	}
	if idx > c.maxArrival {
		c.maxArrival = idx
	}
	// 在投递阶段维护时间基准：以“最早见过序号”的加盖时间建立锚点。
	// 乱序晚到包若序号更早，则校准锚点（假定其自身无跳变；真正的首历元跳变不可检测）。
	if c.cfg.IntervalHint > 0 {
		if !c.anchorSet || idx < c.anchorIndex {
			c.anchor = d.Epoch.Epoch.Add(-time.Duration(idx) * time.Duration(c.cfg.IntervalHint))
			c.anchorSet = true
			c.anchorIndex = idx
		}
	}
	c.garbageCollect(idx)
	c.pending[idx] = &d
	return true
}

// SetClockJumpTolerance 更新钟跳判定容差（纳秒）；仅影响更新之后处理的历元。
func (c *Channel) SetClockJumpTolerance(tolerance int64) {
	c.cfg.ClockJumpTolerance = tolerance
}

// markDuplicateSuppressed 在等待处理或已提交（含已落盘）的结果上幂等追加重复标记。
func (c *Channel) markDuplicateSuppressed(idx int) {
	if p := c.pending[idx]; p != nil {
		p.Flags = addFlag(p.Flags, model.FlagDuplicateSuppressed)
		return
	}
	if s := c.committed[idx]; s != nil {
		s.Flags = addFlag(s.Flags, model.FlagDuplicateSuppressed)
	}
	if c.store != nil {
		c.store.AnnotateFlags(c.cfg.ID, idx, model.FlagDuplicateSuppressed)
	}
}

// garbageCollect 保证 seen/committed 只保留有界窗口内的序号；
// pending 中的未处理历元不按窗口回收，以免乱序窗口内的迟到包被误当缺失。
func (c *Channel) garbageCollect(latestArrival int) {
	tolerance := c.cfg.ReorderTolerance
	if tolerance < 0 {
		tolerance = 0
	}
	lowWater := latestArrival - tolerance - 1
	// pending 中的未处理历元一律保留：乱序窗口内的迟到包序号更小，提前回收会
	// 被误当缺失；它们在被处理或 FlushEndOfStream 后自然移除。
	for idx := range c.seen {
		if idx < lowWater {
			delete(c.seen, idx)
		}
	}
	for idx := range c.committed {
		if idx < lowWater {
			delete(c.committed, idx)
		}
	}
}

// Drain 按序号处理所有就绪历元，返回新提交结果。流中不确认缺口（迟到包仍可能
// 填补）；缺失历元的 FlagEpochMissing 占位在 FlushEndOfStream 中统一提交。
func (c *Channel) Drain() []*model.Solution {
	return c.drainReady(false)
}

// FlushEndOfStream 通知通道输入流已结束：乱序容忍窗口内仍未等到的缺口全部确认为
// 缺失占位并提交，返回新提交结果。
func (c *Channel) FlushEndOfStream() []*model.Solution {
	// 先按序号处理所有已到达历元（中间缺口暂时保留，绝不误补占位），
	// 再把其后再无任何已到达历元的缺口逐个提交为缺失占位。
	out := c.drainReady(false)
	// 窗口内迟到但序号更早的历元：这些序号在游标之前，不能用 nextIndex 顺序处理；
	// 按序号升序逐个处理未提交的 pending（不会重复提交已落盘历元）。
	remaining := make([]int, 0, len(c.pending))
	for idx := range c.pending {
		remaining = append(remaining, idx)
	}
	sortInts(remaining)
	for _, idx := range remaining {
		d := c.pending[idx]
		if d == nil {
			continue
		}
		if c.store != nil && c.store.Solution(c.cfg.ID, idx) != nil {
			delete(c.pending, idx)
			continue
		}
		// 处理该迟到历元之前，先把它与游标之间的缺口补为缺失占位，
		// 保证提交结果序号连续且失败占位不被真实历元覆盖。
		for c.nextIndex < idx {
			out = append(out, c.commitMissing())
		}
		sol := c.process(d)
		c.committed[idx] = sol
		if c.store != nil {
			if _, err := c.store.Commit(c.cfg.ID, sol); err == nil {
				out = append(out, sol)
			}
		} else {
			out = append(out, sol)
		}
		delete(c.pending, idx)
		if idx >= c.nextIndex {
			c.nextIndex = idx + 1
		}
	}
	out = append(out, c.drainReady(true)...)
	return out
}

// drainReady 按序号处理就绪历元。commitGaps=true（流结束）时，其后再无任何
// pending 的缺口提交为缺失占位；否则流中保留缺口等待迟到包。
func (c *Channel) drainReady(commitGaps bool) []*model.Solution {
	var out []*model.Solution

	for {
		if commitGaps {
			// 记录当前最小 pending 序号：缺口只补到它之前。
			minPending := -1
			for idx := range c.pending {
				if idx >= c.nextIndex && (minPending < 0 || idx < minPending) {
					minPending = idx
				}
			}
			if minPending < 0 {
				// 其后再无已到达历元：若历史上见到过更远序号，说明确为缺口，补占位；
				// 否则流自然结束。
				if c.maxArrival < c.nextIndex {
					break
				}
				out = append(out, c.commitMissing())
				continue
			}
			if c.nextIndex < minPending {
				out = append(out, c.commitMissing())
				continue
			}
		}

		d, ok := c.pending[c.nextIndex]
		if !ok {
			// 流中不主动确认缺口：容忍窗口内的迟到包可能恰好填补它。
			// 缺口统一由 FlushEndOfStream（调用方声明投递结束）确认。
			break
		}

		sol := c.process(d)
		c.committed[c.nextIndex] = sol
		if c.store != nil {
			if _, err := c.store.Commit(c.cfg.ID, sol); err == nil {
				out = append(out, sol)
			}
		} else {
			out = append(out, sol)
		}
		delete(c.pending, c.nextIndex)
		c.nextIndex++
	}
	return out
}

func (c *Channel) commitMissing() *model.Solution {
	sol := &model.Solution{
		Channel:       c.cfg.ID,
		Index:         c.nextIndex,
		FailureReason: model.FailureNoUsableSatellites,
		Flags:         []model.QualityFlag{model.FlagEpochMissing},
	}
	c.markFailed(sol)
	c.committed[c.nextIndex] = sol
	if c.store != nil {
		_, _ = c.store.Commit(c.cfg.ID, sol)
	}
	c.nextIndex++
	return sol
}

func (c *Channel) process(d *Delivery) *model.Solution {
	ep := d.Epoch
	flags := append([]model.QualityFlag{}, d.Flags...)

	// 钟跳检测与时间基准修正：历元严格按序号处理，检测与标记始终归属真实跳变历元，
	// 不受乱序投递影响。解算一律使用标称时间网格，保证钟跳历元仍给出正常精度的定位，
	// 且时间基准异常不会被误判为卫星故障。
	solveEpoch := ep.Epoch
	if c.cfg.IntervalHint > 0 {
		solveEpoch, flags = c.detectClockJump(ep.Index, ep.Epoch, flags)
	}

	res := raim.Evaluate(ep.Obs, solveEpoch, c.cfg.Position, c.cfg.RAIM, c.drift)

	sol := &model.Solution{
		Channel: c.cfg.ID,
		Index:   ep.Index,
		Epoch:   ep.Epoch,
		Flags:   flags,
	}

	switch res.Status {
	case model.RAIMOK:
		sol.Status = model.StatusOK
	case model.RAIMFaultExcluded:
		sol.Status = model.StatusDegraded
		sol.Flags = addFlag(sol.Flags, model.FlagFaultExcluded)
	case model.RAIMFaultDetected:
		// 检测到但无法安全剔除：统一走失败清理，绝不输出无依据位置或误报故障星。
		sol.FailureReason = model.FailureNoConvergence
		c.markFailed(sol)
		return sol
	case model.RAIMUnavailable:
		if res.OK {
			sol.Status = model.StatusDegraded
			sol.Flags = addFlag(sol.Flags, model.FlagRAIMUnavailable)
		} else {
			sol.FailureReason = res.Reason
			c.markFailed(sol)
			return sol
		}
	}

	// StatusOK / StatusDegraded：填充数值输出。
	sol.PositionECEF = res.Position
	sol.ClockBiasM = res.ClockBiasM
	sol.UsedSats = res.UsedSats
	sol.ExcludedSats = res.ExcludedSats
	sol.Geo = position.ECEFToGeo(res.Position)
	sol.Quality = position.BuildQualityReport(res.UsedSats, res.Residuals, res.Covariance, res.Sigma0, sol.Geo)
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

	if sol.Integrity.HPL > sol.Integrity.HAL || sol.Integrity.VPL > sol.Integrity.VAL {
		if sol.Status == model.StatusOK {
			sol.Status = model.StatusDegraded
		}
	}

	return sol
}

// detectClockJump 以序号对应的标称时间为基准判定钟跳，返回修正后的解算历元时间。
// 正跳变与负跳变同样适用；偏差在容差以内的轻微抖动不标记，仍按标称时间网格解算。
func (c *Channel) detectClockJump(idx int, stamped time.Time, flags []model.QualityFlag) (time.Time, []model.QualityFlag) {
	if !c.anchorSet {
		// 首个处理历元可能是乱序晚到包。锚点在 Ingest 阶段已由最早见过序号的
		// 加盖时间建立（见 Ingest）；此处兜底再建立一次。
		c.anchor = stamped.Add(-time.Duration(idx) * time.Duration(c.cfg.IntervalHint))
		c.anchorSet = true
		c.anchorIndex = idx
		return c.anchor.Add(time.Duration(idx) * time.Duration(c.cfg.IntervalHint)), flags
	}
	if idx < c.anchorIndex {
		// 更早序号的晚到包到达：以其加盖时间校准时间基准。该历元不参与钟跳判定
		// （无法区分其加盖时间是真实跳变还是晚到），直接按校准后标称网格解算。
		c.anchor = stamped.Add(-time.Duration(idx) * time.Duration(c.cfg.IntervalHint))
		c.anchorIndex = idx
		return c.anchor.Add(time.Duration(idx) * time.Duration(c.cfg.IntervalHint)), flags
	}
	nominalAt := c.anchor.Add(time.Duration(idx) * time.Duration(c.cfg.IntervalHint))
	tolerance := c.cfg.ClockJumpTolerance
	if tolerance <= 0 {
		tolerance = c.cfg.IntervalHint / 2
	}
	deviation := stamped.Sub(nominalAt)
	if deviation > time.Duration(tolerance) || deviation < -time.Duration(tolerance) {
		flags = addFlag(flags, model.FlagClockJump)
		// 当前历元仍按该序号对应的标称时间网格解算（观测即按真实发射时刻构造），
		// 时间基准不随加盖时间戳推进，紧邻后续历元自然不会被连带标记。
	}
	return nominalAt, flags
}

// markFailed 统一所有失败路径的输出约定：失败历元不携带坐标、精度数值或完好性识别数值，
// 故障卫星列表为空并以 RAIMUnavailable 明确表示完好性不可用；链路标记原样保留。
func (c *Channel) markFailed(sol *model.Solution) {
	sol.Status = model.StatusFailed
	sol.PositionECEF = model.Vec3{}
	sol.ClockBiasM = 0
	sol.Geo = model.GeoPoint{}
	sol.UsedSats = nil
	sol.ExcludedSats = nil
	sol.Quality = model.QualityReport{}
	sol.Integrity = model.IntegrityReport{
		Status: model.RAIMUnavailable,
		HAL:    c.cfg.RAIM.HAL,
		VAL:    c.cfg.RAIM.VAL,
	}
}

func addFlag(flags []model.QualityFlag, target model.QualityFlag) []model.QualityFlag {
	for _, f := range flags {
		if f == target {
			return flags
		}
	}
	return append(flags, target)
}
