package pipeline

import (
	"math"
	"path/filepath"
	"sync"
	"testing"
	"time"

	"github.com/guoyyit/gnss-positioning-go/model"
	"github.com/guoyyit/gnss-positioning-go/position"
	"github.com/guoyyit/gnss-positioning-go/raim"
	"github.com/guoyyit/gnss-positioning-go/simulator"
	"github.com/guoyyit/gnss-positioning-go/store"
)

var t0 = time.Date(2026, 1, 1, 0, 0, 0, 0, time.UTC)

func testSimulator(t *testing.T, seed int64, faults []simulator.Fault) (*simulator.Simulator, model.Vec3) {
	t.Helper()
	ephs := simulator.GenerateEphemerides(t0, 6, 4, 26560000.0, 55.0*math.Pi/180.0)
	recv := model.Vec3{X: -2177000, Y: 4380000, Z: 4070000}
	sim := simulator.New(simulator.Config{
		Seed: seed, StartTime: t0, Interval: time.Second, NumEpochs: 40,
		NoiseSigma: 1.0, Faults: faults,
	}, recv, ephs)
	return sim, recv
}

func baseConfig(id string) Config {
	return Config{
		ID:               id,
		IntervalHint:     int64(time.Second),
		ReorderTolerance: 2,
		Position:         position.DefaultConfig(),
		RAIM:             raim.DefaultConfig(),
	}
}

func solHasFlag(sol *model.Solution, flag model.QualityFlag) bool {
	for _, f := range sol.Flags {
		if f == flag {
			return true
		}
	}
	return false
}

func TestLinkAnomalies(t *testing.T) {
	sim, recv := testSimulator(t, 5, nil)
	link := NewLink(sim, LinkConfig{
		MissingWhole: map[int]bool{7: true},
		MissingSats:  map[int][]model.SatID{3: {"G00", "G01", "G02"}},
		Duplicates:   map[int]int{4: 2},
		ReorderSwaps: map[int]int{1: 9}, // 1 与 10 交换；10 先到且越窗 => 1 真正晚到
		ClockJumps:   map[int]float64{6: 1.5},
	})

	st, err := store.New(filepath.Join(t.TempDir(), "state.gob"))
	if err != nil {
		t.Fatal(err)
	}
	defer st.Close()

	eng := NewEngine(st)
	eng.AddChannel(baseConfig("ch-a"))
	solutions := eng.RunPlayback("ch-a", link.DeliveryPlan(12))

	byIndex := map[int]*model.Solution{}
	for _, s := range solutions {
		if _, dup := byIndex[s.Index]; dup {
			t.Fatalf("epoch %d committed twice", s.Index)
		}
		byIndex[s.Index] = s
	}

	if _, ok := byIndex[3]; !ok {
		t.Fatal("epoch 3 missing")
	}
	if !solHasFlag(byIndex[3], model.FlagObservationMissing) {
		t.Fatal("expected partial-observation flag on epoch 3")
	}
	if byIndex[4] == nil {
		t.Fatal("epoch 4 missing")
	}
	if byIndex[5] == nil {
		t.Fatal("epoch 5 missing")
	}
	if byIndex[1] == nil || !solHasFlag(byIndex[1], model.FlagOutOfOrder) {
		t.Fatalf("expected out-of-order flag on late epoch 1, flags=%v", byIndex[1].Flags)
	}
	if !solHasFlag(byIndex[6], model.FlagClockJump) {
		t.Fatal("expected clock-jump flag on epoch 6")
	}
	if byIndex[7] == nil || byIndex[7].Status != model.StatusFailed ||
		!solHasFlag(byIndex[7], model.FlagEpochMissing) {
		t.Fatal("epoch 7 should be a committed failed placeholder")
	}

	// 正常历元定位正确。
	s4 := byIndex[4]
	if s4.Status != model.StatusOK {
		t.Fatalf("epoch 4 status %v", s4.Status)
	}
	errm := math.Hypot(math.Hypot(s4.PositionECEF.X-recv.X, s4.PositionECEF.Y-recv.Y), s4.PositionECEF.Z-recv.Z)
	if errm > 10 {
		t.Fatalf("epoch 4 position error %.2f", errm)
	}
	if math.Abs(s4.Geo.HeightM) > 10000 {
		t.Fatalf("implausible height %v", s4.Geo)
	}
}

func TestDeterminism(t *testing.T) {
	run := func() []*model.Solution {
		sim, _ := testSimulator(t, 123, []simulator.Fault{{
			Kind: simulator.FaultStep, Satellites: []model.SatID{"G03"},
			StartEpoch: 3, EndEpoch: 10, Magnitude: 120,
		}})
		link := NewLink(sim, LinkConfig{Duplicates: map[int]int{2: 1}})
		st, _ := store.New(filepath.Join(t.TempDir(), "a.gob"))
		defer st.Close()
		eng := NewEngine(st)
		eng.AddChannel(baseConfig("ch"))
		return eng.RunPlayback("ch", link.DeliveryPlan(15))
	}

	a := run()
	b := run()
	if len(a) != len(b) {
		t.Fatalf("length mismatch %d vs %d", len(a), len(b))
	}
	for i := range a {
		if a[i].Index != b[i].Index {
			t.Fatalf("index order differs at %d: %d vs %d", i, a[i].Index, b[i].Index)
		}
		if !solutionsEqual(a[i], b[i]) {
			t.Fatalf("epoch %d differs: A=%+v integrity=%+v flags=%v ||| B=%+v integrity=%+v flags=%v",
				a[i].Index, a[i].PositionECEF, a[i].Integrity, a[i].Flags,
				b[i].PositionECEF, b[i].Integrity, b[i].Flags)
		}
	}
}

func solutionsEqual(a, b *model.Solution) bool {
	if a.Index != b.Index || a.Status != b.Status || a.FailureReason != b.FailureReason {
		return false
	}
	if a.PositionECEF != b.PositionECEF || a.ClockBiasM != b.ClockBiasM {
		return false
	}
	if a.Integrity.Status != b.Integrity.Status || a.Integrity.Statistic != b.Integrity.Statistic {
		return false
	}
	if len(a.Flags) != len(b.Flags) {
		return false
	}
	for i := range a.Flags {
		if a.Flags[i] != b.Flags[i] {
			return false
		}
	}
	return true
}

func TestConcurrentChannelsIsolation(t *testing.T) {
	const numChannels = 8
	st, err := store.New(filepath.Join(t.TempDir(), "multi.gob"))
	if err != nil {
		t.Fatal(err)
	}
	defer st.Close()
	eng := NewEngine(st)

	var wg sync.WaitGroup
	for c := 0; c < numChannels; c++ {
		id := "ch-" + string(rune('a'+c))
		seed := int64(100 + c)
		cfg := baseConfig(id)
		// 每个通道注入不同的故障星，验证互不干扰。
		faults := []simulator.Fault{{
			Kind:       simulator.FaultStep,
			Satellites: []model.SatID{model.SatID("G0" + string(rune('0'+c%6)))},
			StartEpoch: 2, EndEpoch: 20, Magnitude: 130,
		}}
		sim, recv := testSimulator(t, seed, faults)
		eng.AddChannel(cfg)
		link := NewLink(sim, LinkConfig{})
		wg.Add(1)
		go func() {
			defer wg.Done()
			sols := eng.RunPlayback(id, link.DeliveryPlan(20))
			if len(sols) != 20 {
				t.Errorf("channel %s got %d solutions", id, len(sols))
				return
			}
			s := sols[10]
			if s.Status != model.StatusDegraded || s.Integrity.Status != model.RAIMFaultExcluded {
				t.Errorf("channel %s expected excluded status, got %v/%v", id, s.Status, s.Integrity.Status)
				return
			}
			errm := math.Hypot(math.Hypot(s.PositionECEF.X-recv.X, s.PositionECEF.Y-recv.Y), s.PositionECEF.Z-recv.Z)
			if errm > 10 {
				t.Errorf("channel %s error %.2f", id, errm)
			}
		}()
	}
	wg.Wait()

	if len(eng.Channels()) != numChannels {
		t.Fatalf("expected %d channels", numChannels)
	}
}

func TestRestartRecovery(t *testing.T) {
	dbPath := filepath.Join(t.TempDir(), "restart.gob")

	planFor := func(sim *simulator.Simulator) []Delivery {
		return NewLink(sim, LinkConfig{}).DeliveryPlan(16)
	}

	sim1, _ := testSimulator(t, 77, nil)
	st1, err := store.New(dbPath)
	if err != nil {
		t.Fatal(err)
	}
	eng1 := NewEngine(st1)
	eng1.AddChannel(baseConfig("ch"))
	first := eng1.RunPlayback("ch", planFor(sim1))
	if len(first) != 16 {
		t.Fatalf("expected 16, got %d", len(first))
	}
	if err := st1.Close(); err != nil {
		t.Fatal(err)
	}

	// 重启：新建存储与引擎，重复投递全部 16 个历元，不得重复提交或跳过。
	sim2, _ := testSimulator(t, 77, nil)
	st2, err := store.New(dbPath)
	if err != nil {
		t.Fatal(err)
	}
	if last := st2.LastIndex("ch"); last != 15 {
		t.Fatalf("expected recovered last index 15, got %d", last)
	}
	eng2 := NewEngine(st2)
	eng2.AddChannel(baseConfig("ch"))
	again := eng2.RunPlayback("ch", planFor(sim2))
	if len(again) != 0 {
		t.Fatalf("expected no duplicate commits after restart, got %d", len(again))
	}

	// 投递新的历元 16..19，应在恢复点之后继续。
	var more []Delivery
	link2 := NewLink(sim2, LinkConfig{})
	for i := 16; i < 20; i++ {
		ep := sim2.Epoch(i)
		more = append(more, Delivery{Epoch: ep})
	}
	_ = link2
	added := eng2.RunPlayback("ch", more)
	if len(added) != 4 {
		t.Fatalf("expected 4 new commits, got %d", len(added))
	}
	for i, s := range added {
		if s.Index != 16+i {
			t.Fatalf("unexpected index %d at %d", s.Index, i)
		}
	}
	all := st2.All("ch")
	if len(all) != 20 {
		t.Fatalf("expected 20 total stored, got %d", len(all))
	}
	if err := st2.Close(); err != nil {
		t.Fatal(err)
	}
}
