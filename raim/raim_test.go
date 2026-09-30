package raim

import (
	"math"
	"testing"
	"time"

	"github.com/guoyyit/gnss-positioning-go/model"
	"github.com/guoyyit/gnss-positioning-go/position"
	"github.com/guoyyit/gnss-positioning-go/simulator"
)

func setup(t *testing.T, seed int64, faults []simulator.Fault) (*simulator.Simulator, model.Vec3, time.Time) {
	t.Helper()
	t0 := time.Date(2026, 1, 1, 0, 0, 0, 0, time.UTC)
	ephs := simulator.GenerateEphemerides(t0, 6, 4, 26560000.0, 55.0*math.Pi/180.0)
	recv := model.Vec3{X: -2177000, Y: 4380000, Z: 4070000}
	sim := simulator.New(simulator.Config{
		Seed:       seed,
		StartTime:  t0,
		Interval:   time.Second,
		NumEpochs:  12,
		NoiseSigma: 1.0,
		Faults:     faults,
	}, recv, ephs)
	return sim, recv, t0
}

func TestNoFalseAlarm(t *testing.T) {
	sim, _, _ := setup(t, 7, nil)
	ep := sim.Epoch(3)
	d := NewDriftState(DefaultConfig())
	res := Evaluate(ep.Obs, ep.Epoch, position.DefaultConfig(), DefaultConfig(), d)
	if res.Status != model.RAIMOK {
		t.Fatalf("expected RAIMOK, got %v stat=%.2f thr=%.2f", res.Status, res.Statistic, res.Threshold)
	}
	if len(res.FaultSatellites) != 0 {
		t.Fatalf("healthy sats flagged: %v", res.FaultSatellites)
	}
}

func TestSingleStepFault(t *testing.T) {
	faultSat := model.SatID("G00")
	sim, recv, _ := setup(t, 7, []simulator.Fault{{
		Kind: simulator.FaultStep, Satellites: []model.SatID{faultSat},
		StartEpoch: 0, EndEpoch: 11, Magnitude: 150,
	}})
	ep := sim.Epoch(4)
	d := NewDriftState(DefaultConfig())
	res := Evaluate(ep.Obs, ep.Epoch, position.DefaultConfig(), DefaultConfig(), d)

	if res.Status != model.RAIMFaultExcluded {
		t.Fatalf("expected exclusion, got %v stat=%.2f thr=%.2f", res.Status, res.Statistic, res.Threshold)
	}
	if len(res.FaultSatellites) != 1 || res.FaultSatellites[0] != faultSat {
		t.Fatalf("wrong fault identification: %v", res.FaultSatellites)
	}
	err := math.Hypot(math.Hypot(res.Position.X-recv.X, res.Position.Y-recv.Y), res.Position.Z-recv.Z)
	t.Logf("recovered error %.3f m, HPL %.2f VPL %.2f, used %d", err, res.HPL, res.VPL, len(res.UsedSats))
	if err > 10 {
		t.Fatalf("post-exclusion error too large: %.3f", err)
	}
}

func TestMultiFault(t *testing.T) {
	f1, f2 := model.SatID("G00"), model.SatID("G13")
	sim, recv, _ := setup(t, 11, []simulator.Fault{{
		Kind: simulator.FaultMulti, Satellites: []model.SatID{f1, f2},
		StartEpoch: 0, EndEpoch: 11, Magnitude: 200,
	}})
	ep := sim.Epoch(5)
	d := NewDriftState(DefaultConfig())
	res := Evaluate(ep.Obs, ep.Epoch, position.DefaultConfig(), DefaultConfig(), d)

	if res.Status != model.RAIMFaultExcluded {
		t.Fatalf("expected exclusion, got %v", res.Status)
	}
	got := map[model.SatID]bool{}
	for _, s := range res.FaultSatellites {
		got[s] = true
	}
	if !got[f1] || !got[f2] {
		t.Fatalf("expected both faults identified, got %v", res.FaultSatellites)
	}
	err := math.Hypot(math.Hypot(res.Position.X-recv.X, res.Position.Y-recv.Y), res.Position.Z-recv.Z)
	if err > 10 {
		t.Fatalf("post-exclusion error too large: %.3f", err)
	}
}

func TestSlowDrift(t *testing.T) {
	faultSat := model.SatID("G02")
	sim, recv, _ := setup(t, 21, []simulator.Fault{{
		Kind: simulator.FaultDrift, Satellites: []model.SatID{faultSat},
		StartEpoch: 0, EndEpoch: 30, Rate: 12,
	}})

	cfg := DefaultConfig()
	cfg.DriftWindow = 6
	cfg.DriftThreshold = 3.0
	d := NewDriftState(cfg)

	var res Result
	detected := false
	for i := 0; i <= 10; i++ {
		ep := sim.Epoch(i)
		res = Evaluate(ep.Obs, ep.Epoch, position.DefaultConfig(), cfg, d)
		if len(res.FaultSatellites) == 1 && res.FaultSatellites[0] == faultSat {
			detected = true
		}
		if res.Status == model.RAIMFaultExcluded {
			break
		}
	}
	if !detected {
		t.Fatalf("fault satellite never identified, got %v", res.FaultSatellites)
	}
	if res.Status != model.RAIMFaultExcluded {
		t.Fatalf("expected exclusion and recovery, got status %v", res.Status)
	}
	err := math.Hypot(math.Hypot(res.Position.X-recv.X, res.Position.Y-recv.Y), res.Position.Z-recv.Z)
	if err > 10 {
		t.Fatalf("post-drift-exclusion error too large: %.3f", err)
	}
}

func TestSlowDriftDetectorSmallBias(t *testing.T) {
	// 偏差低于全局卡方门限但持续线性增长：专门的漂移检测器应在窗口填满后报警。
	faultSat := model.SatID("G02")
	sim, _, _ := setup(t, 21, []simulator.Fault{{
		Kind: simulator.FaultDrift, Satellites: []model.SatID{faultSat},
		StartEpoch: 0, EndEpoch: 30, Rate: 3,
	}})

	cfg := DefaultConfig()
	cfg.DriftWindow = 8
	cfg.DriftThreshold = 3.0
	d := NewDriftState(cfg)

	found := false
	for i := 0; i <= cfg.DriftWindow+2; i++ {
		ep := sim.Epoch(i)
		r := position.Solve(ep.Obs, ep.Epoch, position.DefaultConfig())
		if !r.OK {
			t.Fatalf("solve failed at epoch %d", i)
		}
		sat, ok := d.push(r.Residuals, 1.0)
		if ok && sat == faultSat {
			found = true
			break
		}
	}
	if !found {
		t.Fatal("drift detector failed to flag slowly drifting satellite")
	}

	// 无故障时同样长度的序列不得误报。
	cleanSim, _, _ := setup(t, 99, nil)
	d2 := NewDriftState(cfg)
	for i := 0; i <= cfg.DriftWindow+4; i++ {
		ep := cleanSim.Epoch(i)
		r := position.Solve(ep.Obs, ep.Epoch, position.DefaultConfig())
		if sat, ok := d2.push(r.Residuals, 1.0); ok {
			t.Fatalf("false drift alarm on healthy satellite %s at epoch %d", sat, i)
		}
	}
}
