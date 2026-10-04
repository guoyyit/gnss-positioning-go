package pipeline

import (
	"math"
	"path/filepath"
	"reflect"
	"sync"
	"testing"
	"time"

	"github.com/guoyyit/gnss-positioning-go/model"
	"github.com/guoyyit/gnss-positioning-go/simulator"
	"github.com/guoyyit/gnss-positioning-go/store"
)

func runPlan(t *testing.T, id string, cfg Config, sim *simulator.Simulator, linkCfg LinkConfig, epochs int) (*store.Store, map[int]*model.Solution) {
	t.Helper()
	st, err := store.New(filepath.Join(t.TempDir(), id+".gob"))
	if err != nil {
		t.Fatal(err)
	}
	eng := NewEngine(st)
	eng.AddChannel(cfg)
	sols := eng.RunPlayback(id, NewLink(sim, linkCfg).DeliveryPlan(epochs))
	byIndex := map[int]*model.Solution{}
	for _, s := range sols {
		if _, dup := byIndex[s.Index]; dup {
			t.Fatalf("epoch %d committed twice", s.Index)
		}
		byIndex[s.Index] = s
	}
	return st, byIndex
}

func posError(s *model.Solution, recv model.Vec3) float64 {
	return math.Hypot(math.Hypot(s.PositionECEF.X-recv.X, s.PositionECEF.Y-recv.Y), s.PositionECEF.Z-recv.Z)
}

// 正向与负向钟跳：标记只归属真实跳变历元，紧邻历元不被连带标记；
// 钟跳历元给出正常精度定位，且不把健康卫星报成故障星。
func TestClockJumpPositiveNegative(t *testing.T) {
	sim, recv := testSimulator(t, 17, nil)
	st, byIndex := runPlan(t, "ch", baseConfig("ch"), sim, LinkConfig{
		ClockJumps: map[int]float64{3: 1.5, 8: -2.0},
	}, 12)
	defer st.Close()

	for i := 0; i < 12; i++ {
		s, ok := byIndex[i]
		if !ok {
			t.Fatalf("epoch %d missing", i)
		}
		wantJump := i == 3 || i == 8
		if got := solHasFlag(s, model.FlagClockJump); got != wantJump {
			t.Fatalf("epoch %d clock-jump flag = %v, want %v (flags=%v)", i, got, wantJump, s.Flags)
		}
		if s.Status == model.StatusFailed {
			t.Fatalf("epoch %d must not fail on a time-base anomaly", i)
		}
		if errm := posError(s, recv); errm > 10 {
			t.Fatalf("epoch %d position error %.2fm exceeds normal noise", i, errm)
		}
		if len(s.Integrity.FaultSatellites) != 0 {
			t.Fatalf("epoch %d healthy sats mislabeled faulty: %v", i, s.Integrity.FaultSatellites)
		}
		if s.Integrity.Status != model.RAIMOK {
			t.Fatalf("epoch %d integrity status %v, want RAIMOK", i, s.Integrity.Status)
		}
	}
}

// 连续两个历元各自发生跳变，两个历元都标记，其他历元不标记。
func TestClockJumpConsecutive(t *testing.T) {
	sim, recv := testSimulator(t, 23, nil)
	st, byIndex := runPlan(t, "ch", baseConfig("ch"), sim, LinkConfig{
		ClockJumps: map[int]float64{5: 1.0, 6: 2.0},
	}, 10)
	defer st.Close()

	for i := 0; i < 10; i++ {
		s := byIndex[i]
		wantJump := i == 5 || i == 6
		if got := solHasFlag(s, model.FlagClockJump); got != wantJump {
			t.Fatalf("epoch %d flag=%v want %v", i, got, wantJump)
		}
		if errm := posError(s, recv); errm > 10 {
			t.Fatalf("epoch %d error %.2fm", i, errm)
		}
	}
}

// 小于判定容差的轻微时间抖动不得被判为钟跳；放宽容差后判定随之变化，
// 且配置更新只影响后续历元。
func TestClockJumpToleranceConfig(t *testing.T) {
	// 默认容差（IntervalHint/2 = 0.5s）：0.1s 抖动不标记。
	sim, _ := testSimulator(t, 29, nil)
	st, byIndex := runPlan(t, "ch", baseConfig("ch"), sim, LinkConfig{
		ClockJumps: map[int]float64{4: 0.1},
	}, 8)
	defer st.Close()
	for i, s := range byIndex {
		if solHasFlag(s, model.FlagClockJump) {
			t.Fatalf("epoch %d flagged on sub-tolerance jitter", i)
		}
	}

	// 放宽容差到 2s：更新前的 1.5s 跳变仍标记，更新后不再标记。
	sim2, _ := testSimulator(t, 31, nil)
	st2, err := store.New(filepath.Join(t.TempDir(), "cfg.gob"))
	if err != nil {
		t.Fatal(err)
	}
	defer st2.Close()
	eng := NewEngine(st2)
	ch := eng.AddChannel(baseConfig("ch2"))

	jumpDelivery := func(idx int, offsetSec float64) Delivery {
		ep := sim2.Epoch(idx)
		ep.Epoch = ep.Epoch.Add(time.Duration(offsetSec * float64(time.Second)))
		return Delivery{Epoch: ep}
	}

	ingestBatch := func(ds ...Delivery) {
		for _, d := range ds {
			ch.Ingest(d)
		}
		ch.Drain()
	}

	batch := make([]Delivery, 0, 7)
	for i := 0; i < 6; i++ {
		batch = append(batch, jumpDelivery(i, 0))
	}
	batch = append(batch, jumpDelivery(6, 1.5)) // 更新容差之前：默认容差 -> 标记
	ingestBatch(batch...)
	ch.SetClockJumpTolerance(int64(2 * time.Second))
	late := make([]Delivery, 0, 4)
	for i := 7; i < 10; i++ {
		late = append(late, jumpDelivery(i, 0))
	}
	late = append(late, jumpDelivery(10, 1.5)) // 放宽之后：1.5s < 2s，不标记
	ingestBatch(late...)

	s6 := st2.Solution("ch2", 6)
	s10 := st2.Solution("ch2", 10)
	if s6 == nil || !solHasFlag(s6, model.FlagClockJump) {
		t.Fatalf("epoch 6 should be flagged under default tolerance: %+v", s6)
	}
	if s10 == nil || solHasFlag(s10, model.FlagClockJump) {
		t.Fatalf("epoch 10 should not be flagged after tolerance relaxed: %+v", s10)
	}
}

// 乱序投递下，钟跳标记归属实际跳变历元，不因到达顺序改变。
func TestClockJumpWithReorder(t *testing.T) {
	sim, recv := testSimulator(t, 37, nil)
	st, byIndex := runPlan(t, "ch", baseConfig("ch"), sim, LinkConfig{
		ReorderSwaps: map[int]int{0: 2}, // 0 与 2 交换投递
		ClockJumps:   map[int]float64{3: 1.5},
	}, 8)
	defer st.Close()

	for i := 0; i < 8; i++ {
		s := byIndex[i]
		if got := solHasFlag(s, model.FlagClockJump); got != (i == 3) {
			t.Fatalf("epoch %d clock flag=%v", i, got)
		}
		if errm := posError(s, recv); errm > 10 {
			t.Fatalf("epoch %d error %.2fm", i, errm)
		}
	}
}

// 钟跳 + 整历元缺失 + 重复叠加：各标记独立可区分，同一历元可同时携带。
func TestAnomalyFlagsCombine(t *testing.T) {
	sim, recv := testSimulator(t, 41, nil)
	st, byIndex := runPlan(t, "ch", baseConfig("ch"), sim, LinkConfig{
		MissingWhole: map[int]bool{4: true},
		Duplicates:   map[int]int{6: 1},
		ClockJumps:   map[int]float64{6: 1.5},
	}, 10)
	defer st.Close()

	s4 := byIndex[4]
	if s4.Status != model.StatusFailed || !solHasFlag(s4, model.FlagEpochMissing) ||
		solHasFlag(s4, model.FlagClockJump) {
		t.Fatalf("missing epoch flags/status wrong: %+v flags=%v", s4.Status, s4.Flags)
	}

	s6 := byIndex[6]
	if !solHasFlag(s6, model.FlagClockJump) || !solHasFlag(s6, model.FlagDuplicateSuppressed) {
		t.Fatalf("epoch 6 should carry both jump and duplicate flags: %v", s6.Flags)
	}
	if errm := posError(s6, recv); errm > 10 {
		t.Fatalf("epoch 6 error %.2fm", errm)
	}
}

// 重复包在历元提交之前到达：不产生第二个结果，提交结果带重复标记。
func TestDuplicateBeforeCommit(t *testing.T) {
	sim, _ := testSimulator(t, 43, nil)
	st, err := store.New(filepath.Join(t.TempDir(), "dup-before.gob"))
	if err != nil {
		t.Fatal(err)
	}
	defer st.Close()
	ch := NewEngine(st).AddChannel(baseConfig("ch"))

	ch.Ingest(Delivery{Epoch: sim.Epoch(0)})
	ch.Ingest(Delivery{Epoch: sim.Epoch(1)})
	ch.Ingest(Delivery{Epoch: sim.Epoch(2)})
	ch.Ingest(Delivery{Epoch: cloneEpoch(sim.Epoch(2))}) // 重复：此刻 2 尚未提交
	sols := ch.Drain()

	if len(sols) != 3 {
		t.Fatalf("expected 3 commits, got %d", len(sols))
	}
	s2 := st.Solution("ch", 2)
	if s2 == nil || !solHasFlag(s2, model.FlagDuplicateSuppressed) {
		t.Fatalf("epoch 2 committed result missing duplicate flag: %+v", s2)
	}
}

// 重复包在历元提交之后到达：结果已落盘，标记被修订进最终结果，仍无第二个提交。
func TestDuplicateAfterCommit(t *testing.T) {
	sim, _ := testSimulator(t, 47, nil)
	st, byIndex := runPlan(t, "ch", baseConfig("ch"), sim, LinkConfig{
		Duplicates: map[int]int{5: 1},
	}, 8)
	defer st.Close()

	s5 := byIndex[5]
	if s5 == nil || !solHasFlag(s5, model.FlagDuplicateSuppressed) {
		t.Fatalf("epoch 5 in-run result missing duplicate flag: %+v", s5)
	}
	persisted := st.Solution("ch", 5)
	if persisted == nil || !solHasFlag(persisted, model.FlagDuplicateSuppressed) {
		t.Fatalf("epoch 5 persisted result missing duplicate flag: %+v", persisted)
	}
	if len(st.All("ch")) != 8 {
		t.Fatalf("duplicate must not create a second commit: %d stored", len(st.All("ch")))
	}
}

// 失败历元输出全局一致：无坐标、无精度/完好性数值，故障卫星列表为空。
func TestFailedOutputSanitized(t *testing.T) {
	sim, _ := testSimulator(t, 53, nil)
	st, byIndex := runPlan(t, "ch", baseConfig("ch"), sim, LinkConfig{
		MissingWhole: map[int]bool{4: true},
	}, 7)
	defer st.Close()

	s := byIndex[4]
	if s.Status != model.StatusFailed {
		t.Fatalf("want failed, got %v", s.Status)
	}
	if s.PositionECEF != (model.Vec3{}) || s.Geo != (model.GeoPoint{}) || s.ClockBiasM != 0 {
		t.Fatalf("failed epoch carries position: %+v geo=%+v bias=%v", s.PositionECEF, s.Geo, s.ClockBiasM)
	}
	if s.Quality != (model.QualityReport{}) {
		t.Fatalf("failed epoch carries quality numerics: %+v", s.Quality)
	}
	if s.Integrity.Status != model.RAIMUnavailable || len(s.Integrity.FaultSatellites) != 0 ||
		s.Integrity.Statistic != 0 || s.Integrity.HPL != 0 || s.Integrity.VPL != 0 ||
		s.Integrity.DegreesFreedom != 0 {
		t.Fatalf("failed epoch integrity not cleared: %+v", s.Integrity)
	}
	if len(s.UsedSats) != 0 || len(s.ExcludedSats) != 0 {
		t.Fatalf("failed epoch carries sat lists: %v %v", s.UsedSats, s.ExcludedSats)
	}
}

// 多通道并发：各通道钟跳/重复互不影响标记、状态与完好性结论。
func TestConcurrentAnomalyIsolation(t *testing.T) {
	st, err := store.New(filepath.Join(t.TempDir(), "multi-anom.gob"))
	if err != nil {
		t.Fatal(err)
	}
	defer st.Close()
	eng := NewEngine(st)

	const n = 6
	var wg sync.WaitGroup
	for c := 0; c < n; c++ {
		id := "ch-" + string(rune('a'+c))
		cfg := baseConfig(id)
		sim, recv := testSimulator(t, int64(200+c), nil)
		eng.AddChannel(cfg)

		linkCfg := LinkConfig{}
		if c%2 == 0 {
			linkCfg.ClockJumps = map[int]float64{5: 1.5}
		} else {
			linkCfg.Duplicates = map[int]int{7: 1}
		}
		wg.Add(1)
		go func() {
			defer wg.Done()
			sols := eng.RunPlayback(id, NewLink(sim, linkCfg).DeliveryPlan(12))
			if len(sols) != 12 {
				t.Errorf("%s got %d solutions", id, len(sols))
				return
			}
			for _, s := range sols {
				if s.Status == model.StatusFailed {
					t.Errorf("%s epoch %d unexpectedly failed", id, s.Index)
					return
				}
				if errm := posError(s, recv); errm > 10 {
					t.Errorf("%s epoch %d error %.2fm", id, s.Index, errm)
					return
				}
			}
			if c%2 == 0 {
				if !solHasFlag(sols[5], model.FlagClockJump) {
					t.Errorf("%s epoch 5 missing clock-jump flag", id)
				}
				if solHasFlag(sols[7], model.FlagDuplicateSuppressed) {
					t.Errorf("%s leaked duplicate flag from another channel", id)
				}
			} else {
				if !solHasFlag(sols[7], model.FlagDuplicateSuppressed) {
					t.Errorf("%s epoch 7 missing duplicate flag", id)
				}
				if solHasFlag(sols[5], model.FlagClockJump) {
					t.Errorf("%s leaked clock-jump flag from another channel", id)
				}
			}
		}()
	}
	wg.Wait()
}

func anomalyLinkCfg() LinkConfig {
	return LinkConfig{
		MissingWhole: map[int]bool{8: true},
		MissingSats:  map[int][]model.SatID{4: {"G00"}},
		Duplicates:   map[int]int{6: 1},
		ReorderSwaps: map[int]int{2: 2},
		ClockJumps:   map[int]float64{5: 1.5, 14: -1.5},
	}
}

// 重启重算：未完成历元（含钟跳/重复/缺失/乱序）重算后与首次完整计算逐项一致。
func TestRestartPendingAnomaliesRecompute(t *testing.T) {
	dbPath := filepath.Join(t.TempDir(), "restart-anom.gob")
	cfg := anomalyLinkCfg()

	// 参考：完整 20 历元一次性计算（含一个无法在首批 10 个投递内确认的缺口，
	// 因此首批只提交 0..8，重启后 9 作为缺失占位补提交）。
	simRef, _ := testSimulator(t, 555, nil)
	refSt, err := store.New(filepath.Join(t.TempDir(), "ref.gob"))
	if err != nil {
		t.Fatal(err)
	}
	defer refSt.Close()
	ref := NewEngine(refSt).AddChannel(baseConfig("ref"))
	for _, d := range NewLink(simRef, cfg).DeliveryPlan(20) {
		ref.Ingest(d)
	}
	refSols := ref.FlushEndOfStream()

	// 首次运行只提交到历元 9。
	sim1, _ := testSimulator(t, 555, nil)
	st1, err := store.New(dbPath)
	if err != nil {
		t.Fatal(err)
	}
	ch1 := NewEngine(st1).AddChannel(baseConfig("ch"))
	for _, d := range NewLink(sim1, cfg).DeliveryPlan(10) {
		ch1.Ingest(d)
	}
	first := ch1.Drain()
	if len(first) != 8 {
		t.Fatalf("expected 8 first commits (0..7; gap 8 awaits tolerance window), got %d", len(first))
	}
	if err := st1.Close(); err != nil {
		t.Fatal(err)
	}

	// 重启后重放完整序列：0..9 幂等跳过，10..19 重算补提交。
	sim2, _ := testSimulator(t, 555, nil)
	st2, err := store.New(dbPath)
	if err != nil {
		t.Fatal(err)
	}
	defer st2.Close()
	eng2 := NewEngine(st2)
	eng2.AddChannel(baseConfig("ch"))
	var deliveries []Delivery
	for _, d := range NewLink(sim2, cfg).DeliveryPlan(20) {
		deliveries = append(deliveries, d)
	}
	added := eng2.RunPlayback("ch", deliveries)
	if len(added) != 12 {
		t.Fatalf("expected 12 recomputed commits (missing-8 placeholder plus 9..19), got %d", len(added))
	}

	all := st2.All("ch")
	if len(all) != 20 {
		t.Fatalf("expected 20 stored solutions, got %d", len(all))
	}
	for i := 0; i < 20; i++ {
		got := st2.Solution("ch", i)
		want := refSols[i]
		got.Channel = ""
		want.Channel = ""
		if !deepSolutionEqual(got, want) {
			t.Fatalf("epoch %d differs after restart:\n got=%+v\nwant=%+v", i, got, want)
		}
	}
}

// 固定种子与相同输入序列下，含异常标记的结果逐历元逐项一致（深比较）。
func TestAnomalyDeterministicReplay(t *testing.T) {
	run := func(db string) []*model.Solution {
		sim, _ := testSimulator(t, 666, nil)
		st, err := store.New(filepath.Join(t.TempDir(), db))
		if err != nil {
			t.Fatal(err)
		}
		defer st.Close()
		ch := NewEngine(st).AddChannel(baseConfig("ch"))
		var out []*model.Solution
		for _, d := range NewLink(sim, anomalyLinkCfg()).DeliveryPlan(20) {
			ch.Ingest(d)
			out = append(out, ch.Drain()...)
		}
		out = append(out, ch.Drain()...)
		return out
	}

	a := run("a.gob")
	b := run("b.gob")
	if len(a) != len(b) {
		t.Fatalf("length %d vs %d", len(a), len(b))
	}
	for i := range a {
		if !deepSolutionEqual(a[i], b[i]) {
			t.Fatalf("epoch %d differs:\nA=%+v\nB=%+v", a[i].Index, a[i], b[i])
		}
	}
}

func deepSolutionEqual(a, b *model.Solution) bool {
	if a == nil || b == nil {
		return a == b
	}
	a.Flags, b.Flags = normalizeFlags(a.Flags), normalizeFlags(b.Flags)
	a.UsedSats, b.UsedSats = normalizeSats(a.UsedSats), normalizeSats(b.UsedSats)
	a.ExcludedSats, b.ExcludedSats = normalizeSats(a.ExcludedSats), normalizeSats(b.ExcludedSats)
	a.Integrity.FaultSatellites = normalizeSats(a.Integrity.FaultSatellites)
	b.Integrity.FaultSatellites = normalizeSats(b.Integrity.FaultSatellites)
	if a.Index != b.Index || a.Status != b.Status || a.FailureReason != b.FailureReason ||
		a.PositionECEF != b.PositionECEF || a.ClockBiasM != b.ClockBiasM || a.Geo != b.Geo ||
		!reflect.DeepEqual(a.Flags, b.Flags) ||
		!reflect.DeepEqual(a.UsedSats, b.UsedSats) ||
		!reflect.DeepEqual(a.ExcludedSats, b.ExcludedSats) {
		return false
	}
	if a.Quality != b.Quality {
		return false
	}
	return reflect.DeepEqual(a.Integrity, b.Integrity)
}

func normalizeFlags(f []model.QualityFlag) []model.QualityFlag {
	if len(f) == 0 {
		return nil
	}
	return f
}

func normalizeSats(s []model.SatID) []model.SatID {
	if len(s) == 0 {
		return nil
	}
	return s
}
