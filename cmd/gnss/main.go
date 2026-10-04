package main

import (
	"flag"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"time"

	"github.com/guoyyit/gnss-positioning-go/model"
	"github.com/guoyyit/gnss-positioning-go/pipeline"
	"github.com/guoyyit/gnss-positioning-go/position"
	"github.com/guoyyit/gnss-positioning-go/raim"
	"github.com/guoyyit/gnss-positioning-go/simulator"
	"github.com/guoyyit/gnss-positioning-go/store"
)

func main() {
	seed := flag.Int64("seed", 20260930, "随机种子（确定性回放）")
	epochs := flag.Int("epochs", 12, "模拟历元数")
	dbPath := flag.String("store", filepath.Join(os.TempDir(), "gnss-state.gob"), "持久化状态文件路径")
	flag.Parse()

	t0 := time.Date(2026, 1, 1, 0, 0, 0, 0, time.UTC)
	ephs := simulator.GenerateEphemerides(t0, 6, 4, 26560000.0, 55.0*math.Pi/180.0)
	// 上海附近的一个地表接收机（ECEF，米）。
	receiver := model.Vec3{X: -2850000, Y: 4660000, Z: 3270000}

	sim := simulator.New(simulator.Config{
		Seed:           *seed,
		StartTime:      t0,
		Interval:       time.Second,
		NumEpochs:      *epochs,
		NoiseSigma:     1.5,
		MultipathSigma: 0.8,
		TropoDelay:     3.2,
		IonoDelay:      2.1,
		Faults: []simulator.Fault{{
			Kind: simulator.FaultStep, Satellites: []model.SatID{"G11"},
			StartEpoch: 4, EndEpoch: 8, Magnitude: 150,
		}},
	}, receiver, ephs)

	link := pipeline.NewLink(sim, pipeline.LinkConfig{
		MissingWhole: map[int]bool{9: true},
		MissingSats:  map[int][]model.SatID{2: {"G00"}},
		Duplicates:   map[int]int{3: 1},
		ClockJumps:   map[int]float64{6: 1.5},
	})

	st, err := store.New(*dbPath)
	if err != nil {
		fmt.Fprintf(os.Stderr, "open store: %v\n", err)
		os.Exit(1)
	}
	defer st.Close()

	eng := pipeline.NewEngine(st)
	pcfg := position.DefaultConfig()
	pcfg.Delay = position.DelayModel{TropoZenith: 3.2, IonoZenith: 2.1}
	pcfg.SigmaRange = 3.0
	eng.AddChannel(pipeline.Config{
		ID:           "demo",
		IntervalHint: int64(time.Second),
		Position:     pcfg,
		RAIM:         raim.DefaultConfig(),
	})

	fmt.Printf("接收机真实位置(ECEF): X=%.1f Y=%.1f Z=%.1f\n", receiver.X, receiver.Y, receiver.Z)
	fmt.Println("epoch | status | lat(°) lon(°) h(m) | err(m) | PDOP | RAIM | faults | flags")
	solutions := eng.RunPlayback("demo", link.DeliveryPlan(*epochs))
	for _, s := range solutions {
		if s.Status == model.StatusFailed {
			fmt.Printf("%5d | %6s | %23s | %6s | %4s | %4s | %v | %v\n",
				s.Index, statusName(s.Status), "-", "-", "-",
				raimName(s.Integrity.Status), s.Integrity.FaultSatellites, flagNames(s.Flags))
			continue
		}
		errm := math.Hypot(math.Hypot(s.PositionECEF.X-receiver.X, s.PositionECEF.Y-receiver.Y),
			s.PositionECEF.Z-receiver.Z)
		fmt.Printf("%5d | %6s | %9.5f %10.5f %8.2f | %6.2f | %4.2f | %4s | %v | %v\n",
			s.Index, statusName(s.Status), s.Geo.LatitudeDeg, s.Geo.LongitudeDeg, s.Geo.HeightM,
			errm, s.Quality.PDOP, raimName(s.Integrity.Status), s.Integrity.FaultSatellites, flagNames(s.Flags))
	}

	fmt.Printf("\n已持久化 %d 个历元到 %s（重启后重放相同输入不会重复提交）\n",
		len(st.All("demo")), *dbPath)
}

func statusName(s model.SolutionStatus) string {
	switch s {
	case model.StatusOK:
		return "OK"
	case model.StatusDegraded:
		return "DEGRADED"
	default:
		return "FAILED"
	}
}

func raimName(s model.RAIMStatus) string {
	switch s {
	case model.RAIMOK:
		return "OK"
	case model.RAIMFaultDetected:
		return "DETECT"
	case model.RAIMFaultExcluded:
		return "EXCLUDE"
	default:
		return "N/A"
	}
}

func flagNames(flags []model.QualityFlag) []string {
	names := map[model.QualityFlag]string{
		model.FlagObservationMissing:  "missing-obs",
		model.FlagEpochMissing:        "missing-epoch",
		model.FlagDuplicateSuppressed: "dup",
		model.FlagOutOfOrder:          "reordered",
		model.FlagClockJump:           "clock-jump",
		model.FlagFaultExcluded:       "fault-excluded",
		model.FlagRAIMUnavailable:     "raim-unavailable",
	}
	out := make([]string, 0, len(flags))
	for _, f := range flags {
		if n, ok := names[f]; ok {
			out = append(out, n)
		}
	}
	return out
}
