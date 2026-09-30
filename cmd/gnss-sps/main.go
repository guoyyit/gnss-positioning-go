package main

import (
	"flag"
	"fmt"
	"math"
	"os"

	"github.com/guoyyit/gnss-positioning-go/gnss"
)

func main() {
	var (
		epochs   = flag.Int("epochs", 30, "number of epochs to simulate")
		seed     = flag.Int64("seed", 42, "deterministic simulator seed")
		stateDir = flag.String("state", ".gnss-state", "persistence directory for committed solutions")
		faultPRN = flag.Int("fault-prn", 0, "PRN with an injected step fault (0 disables)")
		faultAt  = flag.Int("fault-epoch", 8, "start epoch of the injected fault")
		faultM   = flag.Float64("fault-m", 2000, "fault step size in meters")
		noRAIM   = flag.Bool("no-raim", false, "disable receiver autonomous integrity monitoring")
		reset    = flag.Bool("reset", false, "clear previous persistence state before starting")
	)
	flag.Parse()

	if *reset {
		if err := os.RemoveAll(*stateDir); err != nil {
			fmt.Fprintf(os.Stderr, "reset failed: %v\n", err)
			os.Exit(1)
		}
	}

	cfg := gnss.SimConfig{
		Seed:          *seed,
		StartTime:     gnss.GNSSTime{Seconds: 0},
		EpochStep:     30,
		NumSatellites: 24,
		ReceiverLLA: gnss.LLA{
			Lat: 39.9078 * math.Pi / 180,
			Lon: 116.4074 * math.Pi / 180,
			Alt: 50,
		},
		RxClockBias0:   50000,
		RxClockDrift:   100,
		NoiseSigma:     1.0,
		MultipathSigma: 0.5,
	}
	if *faultPRN > 0 {
		cfg.Faults = []gnss.FaultSpec{{
			PRN:        *faultPRN,
			StartEpoch: *faultAt,
			EndEpoch:   *epochs,
			Bias:       *faultM,
		}}
	}

	chCfg := gnss.ChannelConfig{
		ID:                 "demo",
		ClockJumpThreshold: 5000,
		HistoryEpochs:      8,
		Solver: gnss.SolverConfig{
			MinSatellites:  4,
			MaxIterations:  30,
			ConvergenceTol: 1e-6,
			SigmaUERE:      1.2,
			RAIM: gnss.RAIMConfig{
				Enabled:           !*noRAIM,
				MaxFaults:         2,
				SlipThreshold:     6,
				DriftWindowEpochs: 6,
				DriftRateLimit:    1,
				HorizontalAlert:   500,
				VerticalAlert:     500,
				SigmaUERE:         1.2,
			},
		},
	}

	svc, err := gnss.NewService(*stateDir)
	if err != nil {
		fmt.Fprintf(os.Stderr, "service: %v\n", err)
		os.Exit(1)
	}
	defer svc.Close()

	sim := gnss.NewSimulator(cfg)
	fmt.Printf("%-5s %-24s %6s %8s %8s %8s %8s %s\n",
		"ep", "status", "n", "err3d/m", "pdop", "rms/m", "HPL/m", "faults/exclusions/flags")
	for i := 0; i < *epochs; i++ {
		data := sim.NextEpoch()
		sol, err := svc.Submit(chCfg, data)
		if err != nil {
			fmt.Fprintf(os.Stderr, "epoch %d: %v\n", i, err)
			os.Exit(1)
		}
		err3D := math.NaN()
		if sol.Status == gnss.StatusOK {
			err3D = norm3(sub(sol.Position, sim.TruePosition()))
		}
		fmt.Printf("%-5d %-24s %6d %8.2f %8.2f %8.2f %8.1f faults=%v excluded=%v flags=%s\n",
			i, sol.Status.String(), sol.Quality.N, err3D, sol.Quality.PDOP,
			sol.Quality.RMSResidual, sol.RAIM.HPL,
			sol.RAIM.FaultPRNs, sol.ExcludedPRNs, flagsString(sol.Flags))
	}
}

func sub(a, b gnss.Vec3) gnss.Vec3 {
	return gnss.Vec3{X: a.X - b.X, Y: a.Y - b.Y, Z: a.Z - b.Z}
}

func norm3(v gnss.Vec3) float64 {
	return math.Sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z)
}

func flagsString(f gnss.QualityFlag) string {
	if f == gnss.FlagNone {
		return "none"
	}
	names := []struct {
		bit gnss.QualityFlag
		tag string
	}{
		{gnss.FlagMissingObs, "missing"},
		{gnss.FlagDuplicate, "duplicate"},
		{gnss.FlagOutOfOrder, "out_of_order"},
		{gnss.FlagClockJump, "clock_jump"},
		{gnss.FlagRAIMExclusion, "raim_exclusion"},
		{gnss.FlagRAIMUnavailable, "raim_unavailable"},
		{gnss.FlagDegradedGeometry, "degraded_geometry"},
	}
	out := ""
	for _, n := range names {
		if f&n.bit != 0 {
			if out != "" {
				out += ","
			}
			out += n.tag
		}
	}
	return out
}
