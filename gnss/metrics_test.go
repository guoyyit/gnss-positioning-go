package gnss

import (
	"math"
	"testing"
)

func TestQualityMetricsStableAndGeometrySensitive(t *testing.T) {
	cfg := plainChannelCfg("qual")
	run := func(sats int) []Quality {
		simCfg := testSimConfig()
		simCfg.NumSatellites = sats
		sim := NewSimulator(simCfg)
		ch := NewChannel(cfg)
		out := make([]Quality, 10)
		for i := range out {
			sol, err := ch.Handle(sim.NextEpoch())
			if err != nil {
				t.Fatal(err)
			}
			out[i] = sol.Quality
		}
		return out
	}
	richA := run(24)
	richB := run(24)
	for i := range richA {
		if richA[i].PDOP != richB[i].PDOP || richA[i].HDOP != richB[i].HDOP ||
			richA[i].RMSResidual != richB[i].RMSResidual {
			t.Fatalf("epoch %d metrics not reproducible", i)
		}
	}

	// Poorer geometry (fewer satellites) must yield no smaller mean PDOP.
	poor := run(9)
	mean := func(q []Quality) float64 {
		s := 0.0
		for _, v := range q {
			s += v.PDOP
		}
		return s / float64(len(q))
	}
	if mean(poor) <= mean(richA) {
		t.Fatalf("PDOP did not degrade with geometry: rich=%.2f poor=%.2f", mean(richA), mean(poor))
	}

	for i, q := range richA {
		if q.N < 4 || q.GDOP < q.PDOP || q.PDOP < q.HDOP || q.PDOP < q.VDOP || q.TDOP <= 0 {
			t.Fatalf("epoch %d inconsistent DOP: %+v", i, q)
		}
		if q.RMSResidual <= 0 || q.MaxResidual < q.RMSResidual {
			t.Fatalf("epoch %d inconsistent residuals: rms=%.3f max=%.3f", i, q.RMSResidual, q.MaxResidual)
		}
		var covNonZero bool
		for r := 0; r < 4; r++ {
			for c := 0; c < 4; c++ {
				if math.Abs(q.Covariance[r][c]) > 0 {
					covNonZero = true
				}
			}
		}
		if !covNonZero {
			t.Fatalf("epoch %d covariance is all zero", i)
		}
	}
}

func TestBoundedStateUnderLongRun(t *testing.T) {
	simCfg := testSimConfig()
	simCfg.Faults = []FaultSpec{{PRN: 3, StartEpoch: 10, EndEpoch: 15, Bias: 1500}}
	sim := NewSimulator(simCfg)
	chCfg := plainChannelCfg("bounded")
	chCfg.HistoryEpochs = 8
	ch := NewChannel(chCfg)
	const epochs = 500
	for i := 0; i < epochs; i++ {
		if _, err := ch.Handle(sim.NextEpoch()); err != nil {
			t.Fatal(err)
		}
	}
	if len(ch.recent) > chCfg.HistoryEpochs*4 {
		t.Fatalf("recent solutions unbounded: %d entries", len(ch.recent))
	}
	capacity := chCfg.HistoryEpochs + 4
	for prn, series := range ch.history {
		if len(series) > capacity {
			t.Fatalf("PRN %d residual history unbounded: %d", prn, len(series))
		}
	}
	if len(ch.excluded) > chCfg.HistoryEpochs*2 {
		t.Fatalf("exclusion set unbounded: %d", len(ch.excluded))
	}
}
