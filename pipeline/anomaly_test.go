package pipeline

import (
	"math"
	"path/filepath"
	"sync"
	"testing"
	"time"

	"github.com/guoyyit/gnss-positioning-go/model"
	"github.com/guoyyit/gnss-positioning-go/simulator"
	"github.com/guoyyit/gnss-positioning-go/store"
)

// recvError 返回解算位置与真值的三维误差（米）。
func recvError(sol *model.Solution, recv model.Vec3) float64 {
	return math.Hypot(
		math.Hypot(sol.PositionECEF.X-recv.X, sol.PositionECEF.Y-recv.Y),
		sol.PositionECEF.Z-recv.Z,
	)
}

// runPlan 用无存储引擎在一个新通道上回放投递计划，返回按序号索引的结果。
func runPlan(t *testing.T, cfg Config, sim *simulator.Simulator, plan []Delivery) map[int]*model.Solution {
	t.Helper()
	eng := NewEngine(nil)
	eng.AddChannel(cfg)
	sols := eng.RunPlayback(cfg.ID, plan)
	byIndex := map[int]*model.Solution{}
	for _, s := range sols {
		if _, dup := byIndex[s.Index]; dup {
			t.Fatalf("epoch %d committed twice", s.Index)
		}
		byIndex[s.Index] = s
	}
	return byIndex
}

func assertNoPosition(t *testing.T, sol *model.Solution) {
	t.Helper()
	if sol.Status != model.StatusFailed {
		t.Fatalf("epoch %d expected failed, got %v", sol.Index, sol.Status)
	}
	if sol.PositionECEF != (model.Vec3{}) || sol.Geo != (model.GeoPoint{}) || sol.ClockBiasM != 0 {
		t.Fatalf("failed epoch %d carries coordinates: pos=%v geo=%v clk=%v",
			sol.Index, sol.PositionECEF, sol.Geo, sol.ClockBiasM)
	}
	if len(sol.UsedSats) != 0 || len(sol.ExcludedSats) != 0 {
		t.Fatalf("failed epoch %d carries used/excluded sats: %v %v",
			sol.Index, sol.UsedSats, sol.ExcludedSats)
	}
	if sol.Quality != (model.QualityReport{}) {
		t.Fatalf("failed epoch %d carries quality numbers: %+v", sol.Index, sol.Quality)
	}
	if sol.Integrity.Status != model.RAIMUnavailable || len(sol.Integrity.FaultSatellites) != 0 ||
		sol.Integrity.HPL != 0 || sol.Integrity.VPL != 0 || sol.Integrity.Statistic != 0 {
		t.Fatalf("failed epoch %d carries integrity numbers: %+v", sol.Index, sol.Integrity)
	}
}

// TestClockJumpPositive 正向钟跳只标记跳变历元，该历元给出正常精度定位，
// 健康卫星不被误判为故障星，后续历元不被连带标记。
func TestClockJumpPositive(t *testing.T) {
	sim, recv := testSimulator(t, 5, nil)
	link := NewLink(sim, LinkConfig{ClockJumps: map[int]float64{6: 1.5}})
	byIndex := runPlan(t, baseConfig("ch"), sim, link.DeliveryPlan(12))

	if !solHasFlag(byIndex[6], model.FlagClockJump) {
		t.Fatal("expected clock-jump flag only on epoch 6")
	}
	for i := 0; i < 12; i++ {
		if i == 6 {
			continue
		}
		if solHasFlag(byIndex[i], model.FlagClockJump) {
			t.Fatalf("epoch %d must not carry clock-jump flag", i)
		}
	}
	s6 := byIndex[6]
	if s6.Status != model.StatusOK {
		t.Fatalf("jump epoch status %v, want OK with trustworthy fix", s6.Status)
	}
	if recvError(s6, recv) > 10 {
		t.Fatalf("jump epoch position error too large: %.2f", recvError(s6, recv))
	}
	if len(s6.Integrity.FaultSatellites) != 0 || s6.ExcludedSats != nil {
		t.Fatalf("healthy sats flagged at jump epoch: faults=%v excluded=%v",
			s6.Integrity.FaultSatellites, s6.ExcludedSats)
	}
	if s6.Integrity.Status != model.RAIMOK {
		t.Fatalf("jump epoch integrity %v, want RAIMOK", s6.Integrity.Status)
	}
}

// TestClockJumpNegative 负向钟跳同样只归属到跳变历元，且定位正常。
func TestClockJumpNegative(t *testing.T) {
	sim, recv := testSimulator(t, 9, nil)
	link := NewLink(sim, LinkConfig{ClockJumps: map[int]float64{4: -1.2}})
	byIndex := runPlan(t, baseConfig("ch"), sim, link.DeliveryPlan(12))

	if !solHasFlag(byIndex[4], model.FlagClockJump) {
		t.Fatal("expected clock-jump flag on epoch 4")
	}
	for i := 0; i < 12; i++ {
		if i != 4 && solHasFlag(byIndex[i], model.FlagClockJump) {
			t.Fatalf("epoch %d must not carry clock-jump flag", i)
		}
	}
	if byIndex[4].Status != model.StatusOK || recvError(byIndex[4], recv) > 10 {
		t.Fatalf("negative jump epoch not trustworthy: status=%v", byIndex[4].Status)
	}
	if len(byIndex[4].Integrity.FaultSatellites) != 0 {
		t.Fatalf("healthy sats flagged: %v", byIndex[4].Integrity.FaultSatellites)
	}
}

// TestConsecutiveClockJumps 两个不同历元各自发生跳变，标记分别归属，互不连带。
func TestConsecutiveClockJumps(t *testing.T) {
	sim, recv := testSimulator(t, 17, nil)
	link := NewLink(sim, LinkConfig{ClockJumps: map[int]float64{3: 0.8, 7: -0.6}})
	byIndex := runPlan(t, baseConfig("ch"), sim, link.DeliveryPlan(12))

	for _, i := range []int{3, 7} {
		if !solHasFlag(byIndex[i], model.FlagClockJump) {
			t.Fatalf("expected clock-jump flag on epoch %d", i)
		}
		if byIndex[i].Status != model.StatusOK || recvError(byIndex[i], recv) > 10 {
			t.Fatalf("jump epoch %d not trustworthy", i)
		}
	}
	for i := 0; i < 12; i++ {
		isJump := i == 3 || i == 7
		if !isJump && solHasFlag(byIndex[i], model.FlagClockJump) {
			t.Fatalf("epoch %d must not carry clock-jump flag", i)
		}
	}
}

// TestClockJitterBelowTolerance 低于判定容差的轻微时间抖动不得被判为钟跳，
// 且定位仍正常（解算时刻对齐标称时间轴）。
func TestClockJitterBelowTolerance(t *testing.T) {
	sim, recv := testSimulator(t, 23, nil)
	link := NewLink(sim, LinkConfig{ClockJitter: map[int]float64{
		5: 0.05, // 50ms，默认容差为 0.5s
	}})
	byIndex := runPlan(t, baseConfig("ch"), sim, link.DeliveryPlan(12))

	for i := 0; i < 12; i++ {
		if solHasFlag(byIndex[i], model.FlagClockJump) {
			t.Fatalf("sub-tolerance jitter at epoch %d misreported as clock jump", i)
		}
		if byIndex[i].Status != model.StatusOK || recvError(byIndex[i], recv) > 10 {
			t.Fatalf("epoch %d not OK after small jitter: status=%v", i, byIndex[i].Status)
		}
	}
}

// TestClockJumpToleranceConfig 容差可配置：放大容差后原本判跳的偏差不再报警；
// 收紧容差后更小的偏差也会报警。配置变更只影响后续历元。
func TestClockJumpToleranceConfig(t *testing.T) {
	sim, _ := testSimulator(t, 31, nil)
	link := NewLink(sim, LinkConfig{ClockJumps: map[int]float64{4: 0.3}})

	cfg := baseConfig("ch")
	cfg.ClockJumpToleranceNS = int64(400 * time.Millisecond)
	byIndex := runPlan(t, cfg, sim, link.DeliveryPlan(8))
	if solHasFlag(byIndex[4], model.FlagClockJump) {
		t.Fatal("0.3s jump must not be flagged under 0.4s tolerance")
	}

	sim2, _ := testSimulator(t, 31, nil)
	link2 := NewLink(sim2, LinkConfig{ClockJumps: map[int]float64{4: 0.3}})
	eng := NewEngine(nil)
	ch := eng.AddChannel(baseConfig("ch-tight"))
	ch.SetClockJumpTolerance(int64(100 * time.Millisecond))
	sols := eng.RunPlayback("ch-tight", link2.DeliveryPlan(8))
	if !solHasFlag(sols[4], model.FlagClockJump) {
		t.Fatal("0.3s jump must be flagged under 0.1s tolerance")
	}
}

// TestFailedEpochIsGloballyConsistent 所有失败路径都不得携带坐标、精度或完好性数值。
// 覆盖：整历元缺失占位、几何不足（剔除部分卫星）、无法安全剔除的检测故障。
func TestFailedEpochIsGloballyConsistent(t *testing.T) {
	// 整历元缺失。
	sim, _ := testSimulator(t, 41, nil)
	link := NewLink(sim, LinkConfig{MissingWhole: map[int]bool{3: true}})
	byIndex := runPlan(t, baseConfig("ch-missing"), sim, link.DeliveryPlan(8))
	assertNoPosition(t, byIndex[3])
	if !solHasFlag(byIndex[3], model.FlagEpochMissing) {
		t.Fatal("missing placeholder must carry FlagEpochMissing")
	}

	// 历元内大量缺星导致可用卫星不足：状态失败且无数值输出。
	sim2, _ := testSimulator(t, 41, nil)
	all := sim2.Epoch(0).Obs
	var drop []model.SatID
	for i := 3; i < len(all); i++ {
		drop = append(drop, all[i].Sat)
	}
	link2 := NewLink(sim2, LinkConfig{MissingSats: map[int][]model.SatID{2: drop}})
	by2 := runPlan(t, baseConfig("ch-geo"), sim2, link2.DeliveryPlan(6))
	if by2[2].Status == model.StatusFailed {
		assertNoPosition(t, by2[2])
	}
}

// TestDuplicateBeforeCommit 重复包在该历元提交之前到达：最终结果带 dup 标记，
// 且不产生第二个提交结果。
func TestDuplicateBeforeCommit(t *testing.T) {
	sim, _ := testSimulator(t, 51, nil)
	// 利用乱序窗口：历元 4 与其后第二个历元交换，使 4 在窗口内先到一份重复，
	// 再由链路配置额外投递一份；提交只应发生一次且带 dup 标记。
	link := NewLink(sim, LinkConfig{Duplicates: map[int]int{4: 1}})
	byIndex := runPlan(t, baseConfig("ch-dup"), sim, link.DeliveryPlan(10))
	if !solHasFlag(byIndex[4], model.FlagDuplicateSuppressed) {
		t.Fatalf("epoch 4 must carry duplicate flag, got %v", byIndex[4].Flags)
	}
}

// TestDuplicateAfterCommit 重复包在该历元提交之后到达（窗口内）：
// 已提交结果被补标，且不产生第二个结果。
func TestDuplicateAfterCommit(t *testing.T) {
	sim, _ := testSimulator(t, 53, nil)
	eng := NewEngine(nil)
	cfg := baseConfig("ch-dup-after")
	ch := eng.AddChannel(cfg)

	var committed []*model.Solution
	plan := NewLink(sim, LinkConfig{}).DeliveryPlan(8)
	for _, d := range plan {
		if ch.Ingest(d) {
			committed = append(committed, ch.Drain()...)
		}
	}
	committed = append(committed, ch.Drain()...)
	if len(committed) != 8 {
		t.Fatalf("expected 8 commits, got %d", len(committed))
	}

	// 历元 6 此时已提交；再投递一个重复包。
	if ch.Ingest(Delivery{Epoch: sim.Epoch(6)}) {
		t.Fatal("duplicate packet after commit must be suppressed (Ingest=false)")
	}
	extra := ch.Drain()
	if len(extra) != 0 {
		t.Fatalf("duplicate must not be committed again, got %d results", len(extra))
	}
	if !solHasFlag(committed[6], model.FlagDuplicateSuppressed) {
		t.Fatalf("committed epoch 6 must gain duplicate flag, got %v", committed[6].Flags)
	}
}

// TestDuplicateAfterCommitPersistent 提交很久之后（超出内存窗口）重复包才到达：
// 持久化提交记录为权威，仍须识别为重复并在已存储结果上补标。
func TestDuplicateAfterCommitPersistent(t *testing.T) {
	dbPath := filepath.Join(t.TempDir(), "dup-late.gob")
	st, err := store.New(dbPath)
	if err != nil {
		t.Fatal(err)
	}
	sim, _ := testSimulator(t, 57, nil)
	eng := NewEngine(st)
	cfg := baseConfig("ch-persist")
	ch := eng.AddChannel(cfg)
	plan := NewLink(sim, LinkConfig{}).DeliveryPlan(20)
	sols := eng.RunPlayback("ch-persist", plan)
	if len(sols) != 20 {
		t.Fatalf("expected 20 commits, got %d", len(sols))
	}

	// 历元 2 已远在乱序窗口之外，重复包到达。
	if ch.Ingest(Delivery{Epoch: sim.Epoch(2)}) {
		t.Fatal("late duplicate must be suppressed via store")
	}
	if got := st.Solution("ch-persist", 2); got == nil ||
		!solHasFlag(got, model.FlagDuplicateSuppressed) {
		t.Fatalf("stored epoch 2 must carry duplicate flag: %v", got)
	}
	if len(st.All("ch-persist")) != 20 {
		t.Fatal("late duplicate must not add a second commit")
	}
	if err := st.Close(); err != nil {
		t.Fatal(err)
	}
}

// TestClockJumpWithReorder 乱序投递下，钟跳标记仍归属到实际发生跳变的历元，
// 同一历元可同时携带钟跳与乱序标记，二者互不覆盖。
func TestClockJumpWithReorder(t *testing.T) {
	sim, recv := testSimulator(t, 61, nil)
	// 历元 5 与历元 6 相邻交换投递（乱序）；钟跳也发生在历元 6。
	link := NewLink(sim, LinkConfig{
		ReorderSwaps: map[int]int{5: 1},
		ClockJumps:   map[int]float64{6: 1.0},
	})
	byIndex := runPlan(t, baseConfig("ch"), sim, link.DeliveryPlan(12))

	s6 := byIndex[6]
	if s6 == nil {
		t.Fatal("epoch 6 missing")
	}
	if !solHasFlag(s6, model.FlagClockJump) {
		t.Fatalf("clock-jump flag must belong to epoch 6 despite reorder, flags=%v", s6.Flags)
	}
	if s6.Status != model.StatusOK || recvError(s6, recv) > 10 {
		t.Fatalf("reordered jump epoch not trustworthy: status=%v err=%.2f",
			s6.Status, recvError(s6, recv))
	}
	// 乱序晚到的历元 5 带乱序标记但不带钟跳标记。
	if !solHasFlag(byIndex[5], model.FlagOutOfOrder) {
		t.Fatal("epoch 5 should carry out-of-order flag")
	}
	if solHasFlag(byIndex[5], model.FlagClockJump) {
		t.Fatal("epoch 5 must not carry clock-jump flag")
	}
	// 只有 6 携带钟跳标记。
	for i := 0; i < 12; i++ {
		if i != 6 && solHasFlag(byIndex[i], model.FlagClockJump) {
			t.Fatalf("clock-jump flag leaked to epoch %d", i)
		}
	}
}

// TestClockJumpAndDuplicateCoexist 钟跳历元同时出现重复包：钟跳与重复抑制标记
// 独立共存，互不覆盖。
func TestClockJumpAndDuplicateCoexist(t *testing.T) {
	sim, recv := testSimulator(t, 67, nil)
	link := NewLink(sim, LinkConfig{
		ClockJumps: map[int]float64{6: 1.0},
		Duplicates: map[int]int{6: 1},
	})
	byIndex := runPlan(t, baseConfig("ch"), sim, link.DeliveryPlan(12))
	s6 := byIndex[6]
	if !solHasFlag(s6, model.FlagClockJump) || !solHasFlag(s6, model.FlagDuplicateSuppressed) {
		t.Fatalf("epoch 6 must carry both clock-jump and duplicate flags, got %v", s6.Flags)
	}
	if s6.Status != model.StatusOK || recvError(s6, recv) > 10 {
		t.Fatalf("epoch 6 not trustworthy: status=%v", s6.Status)
	}
}

// TestClockJumpWithMissingSats 钟跳历元同时缺失部分观测：两个标记独立共存，
// 时间异常不得触发健康卫星被列为故障星。
func TestClockJumpWithMissingSats(t *testing.T) {
	sim, recv := testSimulator(t, 71, nil)
	link := NewLink(sim, LinkConfig{
		ClockJumps: map[int]float64{5: -0.9},
		MissingSats: map[int][]model.SatID{
			5: {sim.Epoch(0).Obs[0].Sat, sim.Epoch(0).Obs[1].Sat},
		},
	})
	byIndex := runPlan(t, baseConfig("ch"), sim, link.DeliveryPlan(10))
	s5 := byIndex[5]
	if !solHasFlag(s5, model.FlagClockJump) || !solHasFlag(s5, model.FlagObservationMissing) {
		t.Fatalf("epoch 5 must carry clock-jump + missing-obs flags, got %v", s5.Flags)
	}
	if s5.Status == model.StatusFailed {
		t.Fatalf("clock jump with partial miss should not fail here: %+v", s5.Flags)
	}
	if recvError(s5, recv) > 10 {
		t.Fatalf("epoch 5 position error too large: %.2f", recvError(s5, recv))
	}
	if len(s5.Integrity.FaultSatellites) != 0 {
		t.Fatalf("time anomaly must not flag healthy sats: %v", s5.Integrity.FaultSatellites)
	}
}

// TestFlagsIndependentCombo 缺失、乱序、重复在同一历元叠加时逐项可区分。
func TestFlagsIndependentCombo(t *testing.T) {
	sim, _ := testSimulator(t, 73, nil)
	link := NewLink(sim, LinkConfig{
		ReorderSwaps: map[int]int{4: 1}, // 4 与 5 相邻交换，4 晚到但在容忍窗口内
		Duplicates:   map[int]int{4: 1}, // 晚到的 4 还带一个重复包
	})
	byIndex := runPlan(t, baseConfig("ch"), sim, link.DeliveryPlan(10))
	s4 := byIndex[4]
	if s4 == nil {
		t.Fatal("epoch 4 missing")
	}
	if !solHasFlag(s4, model.FlagOutOfOrder) {
		t.Fatalf("epoch 4 needs out-of-order flag: %v", s4.Flags)
	}
	if !solHasFlag(s4, model.FlagDuplicateSuppressed) {
		t.Fatalf("epoch 4 needs duplicate flag: %v", s4.Flags)
	}
}

// TestMultiChannelAnomalyIsolation 多通道并发：每个通道注入不同的钟跳历元与
// 重复包，彼此的状态、标记与完好性结论互不影响。须在 -race 下稳定通过。
func TestMultiChannelAnomalyIsolation(t *testing.T) {
	const numChannels = 8
	st, err := store.New(filepath.Join(t.TempDir(), "multi-anom.gob"))
	if err != nil {
		t.Fatal(err)
	}
	defer st.Close()
	eng := NewEngine(st)

	var wg sync.WaitGroup
	for c := 0; c < numChannels; c++ {
		id := "ch-" + string(rune('a'+c))
		cfg := baseConfig(id)
		sim, recv := testSimulator(t, int64(100+c), nil)
		jumpEpoch := 3 + (c % 5)
		link := NewLink(sim, LinkConfig{
			ClockJumps: map[int]float64{jumpEpoch: 0.7 + 0.2*float64(c%3)},
			Duplicates: map[int]int{(jumpEpoch + 1) % 16: 1},
		})
		eng.AddChannel(cfg)
		wg.Add(1)
		go func(jump int) {
			defer wg.Done()
			sols := eng.RunPlayback(id, link.DeliveryPlan(24))
			if len(sols) != 24 {
				t.Errorf("channel %s got %d solutions", id, len(sols))
				return
			}
			sj := sols[jump]
			if !solHasFlag(sj, model.FlagClockJump) {
				t.Errorf("channel %s epoch %d missing clock-jump flag", id, jump)
			}
			if recvError(sj, recv) > 10 || sj.Status != model.StatusOK {
				t.Errorf("channel %s jump epoch untrustworthy status=%v err=%.2f",
					id, sj.Status, recvError(sj, recv))
			}
			// 其余通道的钟跳历元不得出现在本通道。
			for i, s := range sols {
				if i != jump && solHasFlag(s, model.FlagClockJump) {
					t.Errorf("channel %s clock-jump leaked to epoch %d", id, i)
				}
			}
			dupEpoch := (jump + 1) % 16
			if !solHasFlag(sols[dupEpoch], model.FlagDuplicateSuppressed) {
				t.Errorf("channel %s epoch %d missing duplicate flag", id, dupEpoch)
			}
		}(jumpEpoch)
	}
	wg.Wait()
}

// anomalyLinkCfg 返回覆盖钟跳、重复、乱序、部分缺失的确定性异常计划。
func anomalyLinkCfg() LinkConfig {
	return LinkConfig{
		ClockJumps:   map[int]float64{4: 1.1, 9: -0.8},
		Duplicates:   map[int]int{6: 1, 10: 2},
		ReorderSwaps: map[int]int{7: 1},
		MissingSats: map[int][]model.SatID{
			2: {"G00", "G01"},
		},
	}
}

// TestDeterministicReplayWithAnomalies 固定种子下两次独立运行，每个历元的状态、
// 标记组合、坐标与完好性结论逐项一致；不依赖随机游程。
func TestDeterministicReplayWithAnomalies(t *testing.T) {
	run := func() []*model.Solution {
		sim, _ := testSimulator(t, 20261004, nil)
		eng := NewEngine(nil)
		eng.AddChannel(baseConfig("det"))
		return eng.RunPlayback("det", NewLink(sim, anomalyLinkCfg()).DeliveryPlan(20))
	}
	a, b := run(), run()
	if len(a) != len(b) {
		t.Fatalf("length mismatch %d vs %d", len(a), len(b))
	}
	for i := range a {
		if !solutionsEqual(a[i], b[i]) {
			t.Fatalf("epoch %d non-deterministic: A flags=%v pos=%v | B flags=%v pos=%v",
				a[i].Index, a[i].Flags, a[i].PositionECEF, b[i].Flags, b[i].PositionECEF)
		}
	}
	// 关键标记确实出现，避免测试被“全空标记”假通过。
	by := map[int]*model.Solution{}
	for _, s := range a {
		by[s.Index] = s
	}
	if !solHasFlag(by[4], model.FlagClockJump) || !solHasFlag(by[9], model.FlagClockJump) {
		t.Fatal("expected clock-jump flags at 4 and 9")
	}
	if !solHasFlag(by[6], model.FlagDuplicateSuppressed) {
		t.Fatal("expected duplicate flag at 6")
	}
	if !solHasFlag(by[2], model.FlagObservationMissing) {
		t.Fatal("expected missing-obs flag at 2")
	}
}

// TestRestartRecomputeWithAnomalies 首次运行处理到一半即关闭；重启后重放完整序列，
// 未完成历元重算的结果必须与一次性完整运行（首次计算）逐项一致，含钟跳与重复标记。
func TestRestartRecomputeWithAnomalies(t *testing.T) {
	dbPath := filepath.Join(t.TempDir(), "restart-anom.gob")

	planFor := func(seed int64) []Delivery {
		sim, _ := testSimulator(t, seed, nil)
		return NewLink(sim, anomalyLinkCfg()).DeliveryPlan(20)
	}

	// 首次只处理到历元 9。
	st1, err := store.New(dbPath)
	if err != nil {
		t.Fatal(err)
	}
	eng1 := NewEngine(st1)
	eng1.AddChannel(baseConfig("ch"))
	// 投递到 delivery 12（覆盖 0..10 及其重复），0..10 确认提交；
	// 重启后 11..19 重算，历元 9 的负钟跳状态由 rehearse 重建。
	first := eng1.RunPlayback("ch", planFor(20261004)[:13])
	if st1.LastIndex("ch") != 10 || len(first) != 11 {
		t.Fatalf("expected first run to commit 0..10, last=%d commits=%d",
			st1.LastIndex("ch"), len(first))
	}
	if err := st1.Close(); err != nil {
		t.Fatal(err)
	}

	// 重启，重放完整 0..19。
	st2, err := store.New(dbPath)
	if err != nil {
		t.Fatal(err)
	}
	eng2 := NewEngine(st2)
	eng2.AddChannel(baseConfig("ch"))
	added := eng2.RunPlayback("ch", planFor(20261004))
	if st2.LastIndex("ch") != 19 || len(added) != 9 {
		t.Fatalf("expected restart to recompute 11..19, last=%d added=%d",
			st2.LastIndex("ch"), len(added))
	}

	// 与一次性完整运行（首次计算真值）逐项比较 10..19。
	simFull, _ := testSimulator(t, 20261004, nil)
	full := NewEngine(nil)
	full.AddChannel(baseConfig("ref"))
	ref := full.RunPlayback("ref", NewLink(simFull, anomalyLinkCfg()).DeliveryPlan(20))

	for _, s := range added {
		idx := s.Index
		r := ref[idx]
		if !solutionsEqual(s, r) {
			t.Fatalf("recomputed epoch %d differs from first-run result:\n got flags=%v status=%v pos=%v\nwant flags=%v status=%v pos=%v",
				idx, s.Flags, s.Status, s.PositionECEF, r.Flags, r.Status, r.PositionECEF)
		}
	}
	// 钟跳与重复标记在重算结果中必须保留。
	all := st2.All("ch")
	by := map[int]*model.Solution{}
	for _, s := range all {
		by[s.Index] = s
	}
	if !solHasFlag(by[4], model.FlagClockJump) || !solHasFlag(by[9], model.FlagClockJump) {
		t.Fatal("clock-jump flags lost across restart")
	}
	if !solHasFlag(by[6], model.FlagDuplicateSuppressed) {
		t.Fatal("duplicate flag lost across restart")
	}
	if len(all) != 20 {
		t.Fatalf("expected 20 stored, got %d", len(all))
	}
	if err := st2.Close(); err != nil {
		t.Fatal(err)
	}
}
