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
	Position         position.Config
	RAIM             raim.Config
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
	// 已提交/已见序号集合（仅保留窗口内），用于重复抑制。
	seen       map[int]bool
	maxArrival int

	lastEpoch time.Time
	clockBias float64
}

// NewChannel 构造通道，并从持久化存储恢复提交游标。
func NewChannel(cfg Config, st *store.Store) *Channel {
	c := &Channel{
		cfg:       cfg,
		store:     st,
		drift:     raim.NewDriftState(cfg.RAIM),
		pending:   map[int]*Delivery{},
		seen:      map[int]bool{},
		nextIndex: 0,
	}
	if st != nil {
		c.nextIndex = st.LastIndex(cfg.ID) + 1
	}
	return c
}

// LastCommitted 返回最近一次已提交历元序号（无记录为 -1）。
func (c *Channel) LastCommitted() int {
	return c.nextIndex - 1
}

// Ingest 接收一个可能乱序或重复的数据包。重复数据被丢弃并以返回值标记。
func (c *Channel) Ingest(d Delivery) bool {
	idx := d.Epoch.Index
	if c.seen[idx] {
		return false
	}
	c.seen[idx] = true
	if idx < c.maxArrival {
		d.Flags = append(d.Flags, model.FlagOutOfOrder)
	}
	if idx > c.maxArrival {
		c.maxArrival = idx
	}
	c.garbageCollect(idx)
	c.pending[idx] = &d
	return true
}

// garbageCollect 保证 pending/seen 只保留有界窗口内的序号。
func (c *Channel) garbageCollect(latestArrival int) {
	tolerance := c.cfg.ReorderTolerance
	if tolerance < 0 {
		tolerance = 0
	}
	lowWater := latestArrival - tolerance - 1
	for idx := range c.pending {
		if idx < lowWater || idx < c.nextIndex {
			delete(c.pending, idx)
		}
	}
	for idx := range c.seen {
		if idx < lowWater {
			delete(c.seen, idx)
		}
	}
}

// Drain 按序号处理所有就绪历元，返回新提交结果；缺失历元以 FlagEpochMissing 占位提交。
func (c *Channel) Drain() []*model.Solution {
	var out []*model.Solution
	tolerance := c.cfg.ReorderTolerance
	if tolerance < 0 {
		tolerance = 0
	}

	for {
		d, ok := c.pending[c.nextIndex]
		if !ok {
			// 找到窗口内最远到达的序号；若缺口已无法被迟到数据填补，提交缺失占位。
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

		sol := c.process(d)
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

func (c *Channel) process(d *Delivery) *model.Solution {
	ep := d.Epoch
	flags := append([]model.QualityFlag{}, d.Flags...)

	// 乱序到达标记：该历元晚于此前已处理的某历元才进入，但此处以投递顺序判断。
	// 钟跳检测（链路上未显式给出时的兜底）。
	if !c.lastEpoch.IsZero() && c.cfg.IntervalHint > 0 {
		dt := ep.Epoch.Sub(c.lastEpoch).Nanoseconds()
		nominal := c.cfg.IntervalHint
		if abs64(dt-nominal) > abs64(nominal)/2 {
			if !hasFlag(flags, model.FlagClockJump) {
				flags = append(flags, model.FlagClockJump)
			}
		}
	}
	c.lastEpoch = ep.Epoch

	res := raim.Evaluate(ep.Obs, ep.Epoch, c.cfg.Position, c.cfg.RAIM, c.drift)

	sol := &model.Solution{
		Channel: c.cfg.ID,
		Index:   ep.Index,
		Epoch:   ep.Epoch,
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
		sol.Flags = append(sol.Flags, model.FlagFaultExcluded)
	case model.RAIMFaultDetected:
		// 检测到但无法安全剔除：不输出无依据的位置。
		sol.Status = model.StatusFailed
		sol.FailureReason = model.FailureNoConvergence
	case model.RAIMUnavailable:
		if res.OK {
			sol.Status = model.StatusDegraded
			sol.Flags = append(sol.Flags, model.FlagRAIMUnavailable)
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

	if sol.Integrity.HPL > sol.Integrity.HAL || sol.Integrity.VPL > sol.Integrity.VAL {
		if sol.Status == model.StatusOK {
			sol.Status = model.StatusDegraded
		}
	}

	return sol
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

// 兼容 time 包引用（便于后续扩展）。
var _ = time.Second
