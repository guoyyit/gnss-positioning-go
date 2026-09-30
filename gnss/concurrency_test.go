package gnss

import (
	"path/filepath"
	"sync"
	"testing"
)

func TestConcurrentChannelsIndependent(t *testing.T) {
	tmp := t.TempDir()
	svc, err := NewService(filepath.Join(tmp, "state"))
	if err != nil {
		t.Fatal(err)
	}
	defer svc.Close()

	const nChannels = 8
	const nEpochs = 20
	var wg sync.WaitGroup
	errs := make(chan error, nChannels)
	for c := 0; c < nChannels; c++ {
		wg.Add(1)
		go func(idx int) {
			defer wg.Done()
			sim := NewSimulator(testSimConfig())
			sim.cfg.Seed = int64(100 + idx)
			sim = NewSimulator(sim.cfg)
			cfg := raimChannelCfg(channelName(idx), 2)
			for epoch := 0; epoch < nEpochs; epoch++ {
				sol, err := svc.Submit(cfg, sim.NextEpoch())
				if err != nil {
					errs <- err
					return
				}
				if sol.Status != StatusOK && sol.Status != StatusRAIMDetectionFailure {
					errs <- err
					return
				}
			}
		}(c)
	}
	wg.Wait()
	close(errs)
	for err := range errs {
		t.Fatal(err)
	}

	for c := 0; c < nChannels; c++ {
		history, err := svc.History(channelName(c))
		if err != nil {
			t.Fatal(err)
		}
		if len(history) != nEpochs {
			t.Fatalf("channel %d has %d committed epochs, want %d", c, len(history), nEpochs)
		}
	}
}

func channelName(idx int) string {
	return string(rune('a' + idx))
}

func TestConcurrentSubmitSameEpochNoDuplicate(t *testing.T) {
	tmp := t.TempDir()
	svc, err := NewService(filepath.Join(tmp, "state"))
	if err != nil {
		t.Fatal(err)
	}
	defer svc.Close()

	sim := NewSimulator(testSimConfig())
	data := sim.NextEpoch()
	cfg := plainChannelCfg("race")
	var wg sync.WaitGroup
	results := make(chan Solution, 16)
	for i := 0; i < 16; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			sol, err := svc.Submit(cfg, data)
			if err != nil {
				t.Error(err)
				return
			}
			results <- sol
		}()
	}
	wg.Wait()
	close(results)
	first := <-results
	for sol := range results {
		if sol.Position != first.Position {
			t.Fatal("concurrent duplicate submissions produced different results")
		}
	}
	history, _ := svc.History("race")
	if len(history) != 1 {
		t.Fatalf("expected exactly 1 committed epoch, got %d", len(history))
	}
}
