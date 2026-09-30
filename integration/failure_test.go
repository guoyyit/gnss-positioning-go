package integration

import (
	"math"
	"testing"
	"time"

	"github.com/guoyyit/gnss-positioning-go/model"
	"github.com/guoyyit/gnss-positioning-go/position"
	"github.com/guoyyit/gnss-positioning-go/simulator"
)

func TestFailureReasonsDistinguishable(t *testing.T) {
	t0 := time.Date(2026, 1, 1, 0, 0, 0, 0, time.UTC)
	recv := model.Vec3{X: -2850000, Y: 4660000, Z: 3270000}

	// 几何不足：仅 3 颗可见星。
	ephs3 := simulator.GenerateEphemerides(t0, 3, 1, 26560000.0, 0.96)
	sim3 := simulator.New(simulator.Config{Seed: 1, StartTime: t0}, recv, ephs3)
	r3 := position.Solve(sim3.Epoch(0).Obs, t0, position.DefaultConfig())
	if r3.OK || r3.Reason != model.FailureInsufficientGeometry {
		t.Fatalf("expected insufficient geometry, got ok=%v reason=%v", r3.OK, r3.Reason)
	}

	// 无可用卫星：全部伪距无效。
	ephs := simulator.GenerateEphemerides(t0, 6, 4, 26560000.0, 0.96)
	sim := simulator.New(simulator.Config{Seed: 1, StartTime: t0}, recv, ephs)
	bad := sim.Epoch(0)
	for i := range bad.Obs {
		bad.Obs[i].Pseudorange = 0
	}
	rNone := position.Solve(bad.Obs, t0, position.DefaultConfig())
	if rNone.OK || rNone.Reason != model.FailureInsufficientGeometry {
		t.Fatalf("expected failure with no usable obs, got ok=%v reason=%v", rNone.OK, rNone.Reason)
	}
}

func TestGeoHeight(t *testing.T) {
	// 取 WGS84 椭球面上的一点 -> 反算高程应接近 0。
	lat, lon := 31.23*math.Pi/180, 121.47*math.Pi/180
	a, e2 := position.WGS84A, position.WGS84E2
	n := a / math.Sqrt(1-e2*math.Sin(lat)*math.Sin(lat))
	surface := model.Vec3{
		X: n * math.Cos(lat) * math.Cos(lon),
		Y: n * math.Cos(lat) * math.Sin(lon),
		Z: n * (1 - e2) * math.Sin(lat),
	}
	geo := position.ECEFToGeo(surface)
	if math.Abs(geo.HeightM) > 50 {
		t.Fatalf("surface point height %.2f not near ellipsoid", geo.HeightM)
	}
	if geo.LatitudeDeg < -90 || geo.LatitudeDeg > 90 ||
		geo.LongitudeDeg < -180 || geo.LongitudeDeg > 180 {
		t.Fatalf("geo out of range: %+v", geo)
	}
}

func TestCovariancePositiveDefinite(t *testing.T) {
	t0 := time.Date(2026, 1, 1, 0, 0, 0, 0, time.UTC)
	recv := model.Vec3{X: -2850000, Y: 4660000, Z: 3270000}
	ephs := simulator.GenerateEphemerides(t0, 6, 4, 26560000.0, 0.96)
	sim := simulator.New(simulator.Config{Seed: 5, StartTime: t0}, recv, ephs)
	r := position.Solve(sim.Epoch(0).Obs, t0, position.DefaultConfig())
	if !r.OK {
		t.Fatal(r.Reason)
	}
	for i := 0; i < 4; i++ {
		if r.Covariance[i][i] <= 0 {
			t.Fatalf("covariance diagonal non-positive at %d: %v", i, r.Covariance)
		}
	}
}
