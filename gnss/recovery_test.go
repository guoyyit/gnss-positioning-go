package gnss

import (
	"path/filepath"
	"testing"
)

func generateEpochs(cfg SimConfig, n int) []EpochData {
	sim := NewSimulator(cfg)
	out := make([]EpochData, n)
	for i := 0; i < n; i++ {
		out[i] = sim.NextEpoch()
	}
	return out
}

func TestRestartRecoveryNoDuplicateNoSkip(t *testing.T) {
	tmp := t.TempDir()
	statePath := filepath.Join(tmp, "state")
	cfg := raimChannelCfg("recover", 2)
	epochs := generateEpochs(testSimConfig(), 12)

	svc, err := NewService(statePath)
	if err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 8; i++ {
		if _, err := svc.Submit(cfg, epochs[i]); err != nil {
			t.Fatal(err)
		}
	}
	if err := svc.Close(); err != nil {
		t.Fatal(err)
	}

	svc2, err := NewService(statePath)
	if err != nil {
		t.Fatal(err)
	}
	// Resubmit an already committed epoch: must be idempotent.
	replayed, err := svc2.Submit(cfg, epochs[3])
	if err != nil {
		t.Fatal(err)
	}
	// Continue with the unfinished epochs; none must be skipped or duplicated.
	for i := 8; i < len(epochs); i++ {
		if _, err := svc2.Submit(cfg, epochs[i]); err != nil {
			t.Fatal(err)
		}
	}
	history, err := svc2.History("recover")
	if err != nil {
		t.Fatal(err)
	}
	if len(history) != len(epochs) {
		t.Fatalf("expected %d committed epochs after restart, got %d", len(epochs), len(history))
	}
	for i := range epochs {
		if _, ok := history[i]; !ok {
			t.Fatalf("epoch %d missing after restart", i)
		}
	}
	if replayed.Epoch != 3 {
		t.Fatal("replayed epoch returned wrong index")
	}
	if err := svc2.Close(); err != nil {
		t.Fatal(err)
	}
}

func TestDeterministicReplay(t *testing.T) {
	cfg := raimChannelCfg("det", 2)
	solutions := func() []Solution {
		sim := NewSimulator(testSimConfig())
		ch := NewChannel(cfg)
		out := make([]Solution, 15)
		for i := range out {
			sol, err := ch.Handle(sim.NextEpoch())
			if err != nil {
				t.Fatal(err)
			}
			out[i] = sol
		}
		return out
	}
	first := solutions()
	second := solutions()
	for i := range first {
		a, b := first[i], second[i]
		if a.Position != b.Position ||
			a.ClockBias != b.ClockBias ||
			a.Status != b.Status ||
			a.Flags != b.Flags ||
			a.Quality.PDOP != b.Quality.PDOP ||
			a.Quality.RMSResidual != b.Quality.RMSResidual ||
			a.RAIM.GlobalStat != b.RAIM.GlobalStat {
			t.Fatalf("epoch %d not deterministic", i)
		}
	}
}
