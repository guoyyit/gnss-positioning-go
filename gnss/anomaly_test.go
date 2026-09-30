package gnss

import "testing"

func plainChannelCfg(id string) ChannelConfig {
	return ChannelConfig{
		ID:                 id,
		ClockJumpThreshold: 100000,
		HistoryEpochs:      8,
		Solver: SolverConfig{
			MinSatellites:  4,
			MaxIterations:  30,
			ConvergenceTol: 1e-6,
			SigmaUERE:      1.2,
		},
	}
}

func TestMissingObservationFlagged(t *testing.T) {
	sim := NewSimulator(testSimConfig())
	ch := NewChannel(plainChannelCfg("miss"))
	data := sim.NextEpoch()
	if len(data.Obs) < 6 {
		t.Skip("need at least 6 visible satellites")
	}
	dropped := data.Obs[:len(data.Obs)-2]
	droppedData := EpochData{Epoch: data.Epoch, Time: data.Time, Obs: dropped, Eph: data.Eph}
	sol, err := ch.Handle(droppedData)
	if err != nil {
		t.Fatal(err)
	}
	if sol.Status != StatusOK {
		t.Fatalf("expected solution with fewer sats, got %s: %s", sol.Status, sol.Reason)
	}

	onlyThree := EpochData{Epoch: 1, Time: data.Time, Obs: data.Obs[:3], Eph: data.Eph}
	fail, err := ch.Handle(onlyThree)
	if err != nil {
		t.Fatal(err)
	}
	if fail.Status != StatusInsufficientGeometry {
		t.Fatalf("expected insufficient geometry, got %s", fail.Status)
	}
	if vecNorm(fail.Position) != 0 {
		t.Fatal("failed epoch must not carry an unsubstantiated position")
	}
}

func TestDuplicateEpochIdempotent(t *testing.T) {
	sim := NewSimulator(testSimConfig())
	ch := NewChannel(plainChannelCfg("dup"))
	data := sim.NextEpoch()
	first, err := ch.Handle(data)
	if err != nil {
		t.Fatal(err)
	}
	second, err := ch.Handle(data)
	if err != nil {
		t.Fatal(err)
	}
	if first.Position != second.Position {
		t.Fatal("repeated epoch returned a different solution")
	}

	dupPRN := EpochData{
		Epoch: 5,
		Time:  data.Time,
		Obs:   append(append([]Observation{}, data.Obs...), data.Obs[0]),
		Eph:   data.Eph,
	}
	sol, err := ch.Handle(dupPRN)
	if err != nil {
		t.Fatal(err)
	}
	if sol.Flags&FlagDuplicate == 0 {
		t.Fatal("duplicate PRN observation not flagged")
	}
}

func TestOutOfOrderFlagged(t *testing.T) {
	sim := NewSimulator(testSimConfig())
	ch := NewChannel(plainChannelCfg("ooo"))
	e0 := sim.NextEpoch()
	e1 := sim.NextEpoch()
	if _, err := ch.Handle(e0); err != nil {
		t.Fatal(err)
	}
	if _, err := ch.Handle(e1); err != nil {
		t.Fatal(err)
	}
	late := EpochData{Epoch: 0, Time: GNSSTime{Seconds: -10}, Obs: e0.Obs, Eph: e0.Eph}
	sol, err := ch.Handle(late)
	if err != nil {
		t.Fatal(err)
	}
	if sol.Flags&FlagOutOfOrder == 0 {
		t.Fatal("out-of-order epoch not flagged")
	}
}

func TestClockJumpFlagged(t *testing.T) {
	cfg := testSimConfig()
	cfg.ClockJumps = map[int]float64{3: 5000}
	sim := NewSimulator(cfg)
	chCfg := plainChannelCfg("jump")
	chCfg.ClockJumpThreshold = 2000
	ch := NewChannel(chCfg)
	var sawJump bool
	for epoch := 0; epoch < 6; epoch++ {
		sol, err := ch.Handle(sim.NextEpoch())
		if err != nil {
			t.Fatal(err)
		}
		if sol.Flags&FlagClockJump != 0 {
			sawJump = true
		}
		if epoch != 3 && sol.Flags&FlagClockJump != 0 {
			t.Fatalf("epoch %d falsely flagged clock jump", epoch)
		}
		if sol.Status == StatusOK && vecNorm(vecSub(sol.Position, sim.TruePosition())) > 20 {
			t.Fatalf("epoch %d position error after clock handling", epoch)
		}
	}
	if !sawJump {
		t.Fatal("clock jump at epoch 3 was not flagged")
	}
}
