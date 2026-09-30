package gnss

import (
	"math"
	"testing"
)

func raimChannelCfg(id string, maxFaults int) ChannelConfig {
	return ChannelConfig{
		ID:                 id,
		ClockJumpThreshold: 100000,
		HistoryEpochs:      8,
		Solver: SolverConfig{
			MinSatellites:  4,
			MaxIterations:  30,
			ConvergenceTol: 1e-6,
			SigmaUERE:      1.2,
			RAIM: RAIMConfig{
				Enabled:           true,
				MaxFaults:         maxFaults,
				SlipThreshold:     6,
				DriftWindowEpochs: 5,
				DriftRateLimit:    2.0,
				HorizontalAlert:   500,
				VerticalAlert:     500,
				SigmaUERE:         1.2,
			},
		},
	}
}

func runFaultScenario(t *testing.T, name string, faults []FaultSpec, expectPRNs map[int]bool, epochs int) {
	t.Helper()
	cfg := testSimConfig()
	cfg.Faults = faults
	sim := NewSimulator(cfg)
	ch := NewChannel(raimChannelCfg(name, 2))
	detected := map[int]bool{}
	for epoch := 0; epoch < epochs; epoch++ {
		data := sim.NextEpoch()
		sol, err := ch.Handle(data)
		if err != nil {
			t.Fatalf("epoch %d: %v", epoch, err)
		}
		for _, prn := range sol.RAIM.FaultPRNs {
			detected[prn] = true
		}
		err3D := vecNorm(vecSub(sol.Position, sim.TruePosition()))
		if sol.Status == StatusOK && err3D > 25 {
			t.Fatalf("epoch %d: recovered position error %.1f m", epoch, err3D)
		}
	}
	for prn := range expectPRNs {
		if !detected[prn] {
			t.Fatalf("%s: expected fault PRN %d to be detected, got %v", name, prn, detected)
		}
	}
}

func TestRAIMSingleFault(t *testing.T) {
	runFaultScenario(t, "single",
		[]FaultSpec{{PRN: 7, StartEpoch: 3, EndEpoch: 30, Bias: 2000}},
		map[int]bool{7: true}, 15)
}

func TestRAIMMultiFault(t *testing.T) {
	runFaultScenario(t, "multi",
		[]FaultSpec{
			{PRN: 3, StartEpoch: 4, EndEpoch: 10, Bias: 3000},
			{PRN: 12, StartEpoch: 4, EndEpoch: 10, Bias: -3000},
		},
		map[int]bool{3: true, 12: true}, 13)
}

func TestRAIMSlowDrift(t *testing.T) {
	cfg := testSimConfig()
	cfg.Faults = []FaultSpec{{PRN: 2, StartEpoch: 2, EndEpoch: 60, Rate: 0.03}}
	sim := NewSimulator(cfg)
	chCfg := raimChannelCfg("drift", 2)
	chCfg.Solver.RAIM.DriftRateLimit = 0.5
	chCfg.Solver.RAIM.DriftWindowEpochs = 6
	chCfg.Solver.RAIM.SlipThreshold = 2.5
	ch := NewChannel(chCfg)
	detected := false
	for epoch := 0; epoch < 40; epoch++ {
		data := sim.NextEpoch()
		sol, err := ch.Handle(data)
		if err != nil {
			t.Fatal(err)
		}
		for _, prn := range sol.RAIM.FaultPRNs {
			if prn == 2 {
				detected = true
			}
		}
		if err3D := vecNorm(vecSub(sol.Position, sim.TruePosition())); err3D > 25 {
			t.Fatalf("epoch %d: position error %.1f m", epoch, err3D)
		}
	}
	if !detected {
		t.Fatal("slow drift of PRN 2 never detected")
	}
}

func TestRAIMHealthyNotExcluded(t *testing.T) {
	sim := NewSimulator(testSimConfig())
	ch := NewChannel(raimChannelCfg("healthy", 2))
	for epoch := 0; epoch < 20; epoch++ {
		data := sim.NextEpoch()
		sol, err := ch.Handle(data)
		if err != nil {
			t.Fatal(err)
		}
		if sol.RAIM.Detected {
			t.Fatalf("epoch %d: false alarm, fault PRNs %v", epoch, sol.RAIM.FaultPRNs)
		}
		if len(sol.ExcludedPRNs) != 0 {
			t.Fatalf("epoch %d: healthy satellite excluded: %v", epoch, sol.ExcludedPRNs)
		}
		if err3D := vecNorm(vecSub(sol.Position, sim.TruePosition())); err3D > 20 {
			t.Fatalf("epoch %d: position error %.1f m", epoch, err3D)
		}
	}
}

func TestProtectionLevelsPositive(t *testing.T) {
	sim := NewSimulator(testSimConfig())
	ch := NewChannel(raimChannelCfg("pl", 2))
	data := sim.NextEpoch()
	sol, _ := ch.Handle(data)
	if !sol.RAIM.Available {
		t.Skip("not enough visible satellites for RAIM availability")
	}
	if sol.RAIM.HPL <= 0 || sol.RAIM.VPL <= 0 {
		t.Fatalf("non-positive protection levels: HPL=%.2f VPL=%.2f", sol.RAIM.HPL, sol.RAIM.VPL)
	}
	if math.IsNaN(sol.RAIM.HPL) || math.IsInf(sol.RAIM.HPL, 0) {
		t.Fatalf("invalid HPL %.3f", sol.RAIM.HPL)
	}
}
