package integration

import (
	"math"
	"testing"
	"time"

	"github.com/guoyyit/gnss-positioning-go/model"
	"github.com/guoyyit/gnss-positioning-go/position"
	"github.com/guoyyit/gnss-positioning-go/simulator"
)

func TestPVTSmoke(t *testing.T) {
	t0 := time.Date(2026, 1, 1, 0, 0, 0, 0, time.UTC)
	ephs := simulator.GenerateEphemerides(t0, 6, 4, 26560000.0, 55.0*math.Pi/180.0)
	recv := model.Vec3{X: -2177000, Y: 4380000, Z: 4070000}
	sim := simulator.New(simulator.Config{Seed: 42, StartTime: t0, Interval: time.Second, NoiseSigma: 1.0}, recv, ephs)
	ep := sim.Epoch(0)
	res := position.Solve(ep.Obs, ep.Epoch, position.DefaultConfig())
	if !res.OK {
		t.Fatalf("solve failed: %v", res.Reason)
	}
	err := math.Hypot(math.Hypot(res.Position.X-recv.X, res.Position.Y-recv.Y), res.Position.Z-recv.Z)
	if err > 10 {
		t.Fatalf("position error %.3f", err)
	}
}

func TestPVTDelayModelConverges(t *testing.T) {
	t0 := time.Date(2026, 1, 1, 0, 0, 0, 0, time.UTC)
	ephs := simulator.GenerateEphemerides(t0, 6, 4, 26560000.0, 55.0*math.Pi/180.0)
	recv := model.Vec3{X: -2177000, Y: 4380000, Z: 4070000}
	sim := simulator.New(simulator.Config{
		Seed: 42, StartTime: t0, Interval: time.Second, NoiseSigma: 1.0,
		TropoDelay: 3.2, IonoDelay: 2.1,
	}, recv, ephs)
	cfg := position.DefaultConfig()
	cfg.Delay = position.DelayModel{TropoZenith: 3.2, IonoZenith: 2.1}
	ep := sim.Epoch(0)
	res := position.Solve(ep.Obs, ep.Epoch, cfg)
	if !res.OK {
		t.Fatalf("solve with delay model failed: %v iters %d", res.Reason, res.Iterations)
	}
	err := math.Hypot(math.Hypot(res.Position.X-recv.X, res.Position.Y-recv.Y), res.Position.Z-recv.Z)
	if err > 10 {
		t.Fatalf("position error with delay model %.3f", err)
	}
}

func TestDOPVariesWithGeometry(t *testing.T) {
	t0 := time.Date(2026, 1, 1, 0, 0, 0, 0, time.UTC)
	recv := model.Vec3{X: -2177000, Y: 4380000, Z: 4070000}

	run := func(perPlane, planes int) float64 {
		ephs := simulator.GenerateEphemerides(t0, perPlane, planes, 26560000.0, 55.0*math.Pi/180.0)
		sim := simulator.New(simulator.Config{Seed: 42, StartTime: t0, Interval: time.Second}, recv, ephs)
		ep := sim.Epoch(0)
		r := position.Solve(ep.Obs, ep.Epoch, position.DefaultConfig())
		if !r.OK {
			t.Fatalf("solve failed for %dx%d", perPlane, planes)
		}
		geo := position.ECEFToGeo(r.Position)
		qr := position.BuildQualityReport(r.Used, r.Residuals, r.Covariance, r.Sigma0, geo)
		return qr.PDOP
	}

	rich := run(8, 4)
	poor := run(3, 2)
	if rich >= poor {
		t.Fatalf("expected richer geometry to have lower PDOP, got rich=%.2f poor=%.2f", rich, poor)
	}
}
