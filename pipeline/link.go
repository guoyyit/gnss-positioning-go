package pipeline

import (
	"time"

	"github.com/guoyyit/gnss-positioning-go/model"
)

// LinkConfig 描述观测链路上的异常投递计划（确定性）。
type LinkConfig struct {
	MissingWhole map[int]bool          // 整历元完全丢失（链路不投递）
	MissingSats  map[int][]model.SatID // 历元内部分卫星观测缺失
	Duplicates   map[int]int           // 历元重复投递的额外份数
	ReorderSwaps map[int]int           // 历元与其后第 k 个历元交换投递
	ClockJumps   map[int]float64       // 从该历元起接收机时间跳变，秒
}

// Delivery 为链路投递给通道的一个数据包。
type Delivery struct {
	Epoch *model.EpochObs
	Flags []model.QualityFlag
}

// Source 为观测来源。
type Source interface {
	Epoch(index int) *model.EpochObs
}

// Link 按计划对观测做异常投递。
type Link struct {
	source Source
	cfg    LinkConfig
}

// NewLink 构造异常投递链路。
func NewLink(src Source, cfg LinkConfig) *Link {
	return &Link{source: src, cfg: cfg}
}

// DeliveryPlan 返回 numEpochs 个历元的确定性投递序列：
// 缺失的整历元不出现；缺失部分卫星的历元以 FlagObservationMissing 标记；
// 重复按配置追加；乱序按交换计划重排；钟跳改变时间戳。
func (l *Link) DeliveryPlan(numEpochs int) []Delivery {
	type item struct {
		index int
	}
	order := make([]int, 0, numEpochs)
	for i := 0; i < numEpochs; i++ {
		order = append(order, i)
	}
	// 确定性乱序：交换 i 与 i+k（不越界），按 key 升序逐个应用。
	swaps := make([]int, 0, len(l.cfg.ReorderSwaps))
	for k := range l.cfg.ReorderSwaps {
		swaps = append(swaps, k)
	}
	sortInts(swaps)
	for _, i := range swaps {
		k := l.cfg.ReorderSwaps[i]
		j := i + k
		if j >= 0 && j < numEpochs {
			order[i], order[j] = order[j], order[i]
		}
	}

	// 钟跳为累积时间偏移。
	var deliveries []Delivery
	for _, idx := range order {
		if l.cfg.MissingWhole[idx] {
			continue
		}
		ep := l.source.Epoch(idx)
		if ep == nil {
			continue
		}

		var flags []model.QualityFlag
		if missing := l.cfg.MissingSats[idx]; len(missing) > 0 {
			drop := map[model.SatID]bool{}
			for _, sat := range missing {
				drop[sat] = true
			}
			kept := make([]model.Observation, 0, len(ep.Obs))
			for _, o := range ep.Obs {
				if !drop[o.Sat] {
					kept = append(kept, o)
				}
			}
			ep.Obs = kept
			flags = append(flags, model.FlagObservationMissing)
		}

		if jump, ok := l.cfg.ClockJumps[idx]; ok {
			ep = cloneEpoch(ep)
			ep.Epoch = ep.Epoch.Add(secondsDur(jump))
			flags = append(flags, model.FlagClockJump)
		}

		deliveries = append(deliveries, Delivery{Epoch: ep, Flags: flags})
		for d := 0; d < l.cfg.Duplicates[idx]; d++ {
			dupFlags := append([]model.QualityFlag{}, flags...)
			deliveries = append(deliveries, Delivery{Epoch: cloneEpoch(ep), Flags: dupFlags})
		}
	}
	return deliveries
}

func cloneEpoch(ep *model.EpochObs) *model.EpochObs {
	cp := *ep
	cp.Obs = append([]model.Observation(nil), ep.Obs...)
	return &cp
}

func secondsDur(seconds float64) time.Duration {
	return time.Duration(seconds * float64(time.Second))
}

func sortInts(a []int) {
	for i := 1; i < len(a); i++ {
		for j := i; j > 0 && a[j-1] > a[j]; j-- {
			a[j-1], a[j] = a[j], a[j-1]
		}
	}
}
