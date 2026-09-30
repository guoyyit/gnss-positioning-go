package gnss

import (
	"math"
	"testing"
)

func testSimConfig() SimConfig {
	return SimConfig{
		Seed:          42,
		StartTime:     GNSSTime{Seconds: 0},
		EpochStep:     30,
		NumSatellites: 24,
		ReceiverLLA: LLA{
			Lat: 39.9078 * math.Pi / 180,
			Lon: 116.4074 * math.Pi / 180,
			Alt: 50,
		},
		RxClockBias0:   50000,
		RxClockDrift:   100,
		NoiseSigma:     1.0,
		MultipathSigma: 0.5,
	}
}

func TestSinglePointPositioningAccuracy(t *testing.T) {
	sim := NewSimulator(testSimConfig())
	truePos := sim.TruePosition()
	cfg := SolverConfig{
		MinSatellites:  4,
		MaxIterations:  30,
		ConvergenceTol: 1e-6,
		SigmaUERE:      1.2,
	}
	for epoch := 0; epoch < 10; epoch++ {
		data := sim.NextEpoch()
		if len(data.Obs) < 4 {
			t.Fatalf("epoch %d: only %d visible satellites", epoch, len(data.Obs))
		}
		sol := SolveEpoch(data, cfg)
		if sol.Status != StatusOK {
			t.Fatalf("epoch %d: solve failed: %s (%s)", epoch, sol.Status, sol.Reason)
		}
		err3D := vecNorm(vecSub(sol.Position, truePos))
		if err3D > 20 {
			t.Fatalf("epoch %d: 3D error %.2f m exceeds 20 m", epoch, err3D)
		}
		clockErr := math.Abs(sol.ClockBias - sim.TrueClockBias(epoch))
		if clockErr > 20 {
			t.Fatalf("epoch %d: clock error %.2f m exceeds 20 m", epoch, clockErr)
		}
		if math.Abs(sol.LLA.Alt-sim.cfg.ReceiverLLA.Alt) > 30 {
			t.Fatalf("epoch %d: altitude error %.2f m", epoch, sol.LLA.Alt-sim.cfg.ReceiverLLA.Alt)
		}
		if sol.Quality.PDOP <= 0 || sol.Quality.PDOP > 10 {
			t.Fatalf("epoch %d: implausible PDOP %.2f", epoch, sol.Quality.PDOP)
		}
	}
}
