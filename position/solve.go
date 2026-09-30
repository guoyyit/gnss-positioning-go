package position

import (
	"math"
	"time"

	"github.com/guoyyit/gnss-positioning-go/ephemeris"
	"github.com/guoyyit/gnss-positioning-go/model"
)

// Config 为定位解算配置。
type Config struct {
	MinSatellites int
	MaxIterations int
	ToleranceM    float64
	SigmaRange    float64 // 伪距测量标准差，米
	Delay         DelayModel
}

// DefaultConfig 返回默认解算配置。
func DefaultConfig() Config {
	return Config{
		MinSatellites: 4,
		MaxIterations: 20,
		ToleranceM:    1e-4,
		SigmaRange:    3.0,
	}
}

// RawResult 为最小二乘解算的原始输出。
type RawResult struct {
	OK         bool
	Reason     model.FailureReason
	Position   model.Vec3
	ClockBiasM float64
	Used       []model.SatID
	Residuals  map[model.SatID]float64
	Covariance [4][4]float64
	Sigma0     float64
	Iterations int
}

// Solve 对给定历元观测执行迭代加权最小二乘 PVT 解算。
func Solve(obs []model.Observation, epoch time.Time, cfg Config) RawResult {
	healthy := make([]model.Observation, 0, len(obs))
	for _, o := range obs {
		if o.Ephemeris.Healthy && o.Pseudorange > 0 {
			healthy = append(healthy, o)
		}
	}

	used := make([]model.SatID, 0, len(healthy))
	for _, o := range healthy {
		used = append(used, o.Sat)
	}

	if len(healthy) < cfg.MinSatellites {
		return RawResult{
			OK:     false,
			Reason: model.FailureInsufficientGeometry,
			Used:   used,
		}
	}

	if cfg.SigmaRange <= 0 {
		cfg.SigmaRange = DefaultConfig().SigmaRange
	}

	recv := model.Vec3{}
	clockBias := 0.0

	type satGeom struct {
		obs    model.Observation
		pos    model.Vec3
		satClk float64
		los    model.Vec3
		pred   float64
		weight float64
	}

	n := len(healthy)
	geoms := make([]satGeom, n)

	var converged bool
	var iter int
	for iter = 0; iter < cfg.MaxIterations; iter++ {
		grows := make([]float64, 0, n*4)
		bvals := make([]float64, 0, n)
		validWeights := make([]float64, 0, n)

		for i, o := range healthy {
			travel := o.Pseudorange / ephemeris.SpeedOfLight
			tx := epoch.Add(-seconds(travel))
			satPos, err := ephemeris.PositionAt(&o.Ephemeris, tx)
			if err != nil {
				continue
			}
			satPos = ephemeris.RotateEarth(satPos, travel)
			satClk := ephemeris.ClockBias(&o.Ephemeris, tx)

			dx := satPos.X - recv.X
			dy := satPos.Y - recv.Y
			dz := satPos.Z - recv.Z
			geomRange := math.Sqrt(dx*dx + dy*dy + dz*dz)
			pred := geomRange + clockBias - satClk + cfg.Delay.DelayAtSat(satPos, recv)

			grows = append(grows, -dx/geomRange, -dy/geomRange, -dz/geomRange, 1)
			bvals = append(bvals, o.Pseudorange-pred)
			validWeights = append(validWeights, 1.0/(cfg.SigmaRange*cfg.SigmaRange))

			geoms[i] = satGeom{
				obs:    o,
				pos:    satPos,
				satClk: satClk,
				los:    model.Vec3{X: dx / geomRange, Y: dy / geomRange, Z: dz / geomRange},
				pred:   pred,
				weight: 1.0 / (cfg.SigmaRange * cfg.SigmaRange),
			}
		}

		if len(grows)/4 < cfg.MinSatellites {
			return RawResult{
				OK:     false,
				Reason: model.FailureNoUsableSatellites,
				Used:   used,
			}
		}

		delta, ok := solveWeightedNormal(grows, bvals, validWeights)
		if !ok {
			return RawResult{
				OK:     false,
				Reason: model.FailureInsufficientGeometry,
				Used:   used,
			}
		}

		recv.X += delta[0]
		recv.Y += delta[1]
		recv.Z += delta[2]
		clockBias += delta[3]

		if math.Hypot(math.Hypot(delta[0], delta[1]), delta[2]) < cfg.ToleranceM {
			converged = true
			break
		}
	}

	if !converged {
		return RawResult{
			OK:         false,
			Reason:     model.FailureNoConvergence,
			Position:   recv,
			ClockBiasM: clockBias,
			Used:       used,
			Iterations: iter,
		}
	}

	// 收敛后重算几何、残差与协方差。
	gmat := make([]float64, n*4)
	resid := make([]float64, n)
	residuals := make(map[model.SatID]float64, n)
	wrss := 0.0
	for i := range geoms {
		g := &geoms[i]
		dx := g.pos.X - recv.X
		dy := g.pos.Y - recv.Y
		dz := g.pos.Z - recv.Z
		geomRange := math.Sqrt(dx*dx + dy*dy + dz*dz)
		los := model.Vec3{X: dx / geomRange, Y: dy / geomRange, Z: dz / geomRange}
		v := g.obs.Pseudorange - (geomRange + clockBias - g.satClk + cfg.Delay.DelayAtSat(g.pos, recv))

		gmat[i*4+0] = -los.X
		gmat[i*4+1] = -los.Y
		gmat[i*4+2] = -los.Z
		gmat[i*4+3] = 1
		resid[i] = v
		residuals[g.obs.Sat] = v
		wrss += g.weight * v * v
		g.los = los
	}

	normal := make([]float64, 16)
	for i := 0; i < n; i++ {
		w := geoms[i].weight
		for a := 0; a < 4; a++ {
			for b := 0; b < 4; b++ {
				normal[a*4+b] += gmat[i*4+a] * w * gmat[i*4+b]
			}
		}
	}

	cov, ok := invertSymmetric4(normal)
	if !ok {
		return RawResult{
			OK:     false,
			Reason: model.FailureInsufficientGeometry,
			Used:   used,
		}
	}
	// 加权法方程逆为无量纲缩放因子，乘以 σ_range² 得到米²单位的位置/钟差协方差。
	variance := cfg.SigmaRange * cfg.SigmaRange
	for a := 0; a < 4; a++ {
		for b := 0; b < 4; b++ {
			cov[a][b] *= variance
		}
	}

	df := n - 4
	sigma0 := 0.0
	if df > 0 {
		sigma0 = math.Sqrt(wrss / float64(df))
	}

	return RawResult{
		OK:         true,
		Reason:     model.FailureNone,
		Position:   recv,
		ClockBiasM: clockBias,
		Used:       used,
		Residuals:  residuals,
		Covariance: cov,
		Sigma0:     sigma0,
		Iterations: iter + 1,
	}
}

func seconds(d float64) time.Duration {
	return time.Duration(d * float64(time.Second))
}

// solveWeightedNormal 求解 (GᵀWG)δ = GᵀWb，4 个未知数。
func solveWeightedNormal(gmat, bvec, weights []float64) ([4]float64, bool) {
	n := len(weights)
	normal := make([]float64, 16)
	rhs := [4]float64{}
	for i := 0; i < n; i++ {
		w := weights[i]
		for a := 0; a < 4; a++ {
			rhs[a] += gmat[i*4+a] * w * bvec[i]
			for b := 0; b < 4; b++ {
				normal[a*4+b] += gmat[i*4+a] * w * gmat[i*4+b]
			}
		}
	}

	var delta [4]float64
	x, ok := gaussianSolve4(normal, rhs[:])
	if !ok {
		return delta, false
	}
	copy(delta[:], x)
	return delta, true
}

func gaussianSolve4(a []float64, b []float64) ([]float64, bool) {
	n := 4
	aug := make([]float64, n*(n+1))
	for i := 0; i < n; i++ {
		for j := 0; j < n; j++ {
			aug[i*(n+1)+j] = a[i*n+j]
		}
		aug[i*(n+1)+n] = b[i]
	}

	for col := 0; col < n; col++ {
		pivot := col
		for r := col + 1; r < n; r++ {
			if math.Abs(aug[r*(n+1)+col]) > math.Abs(aug[pivot*(n+1)+col]) {
				pivot = r
			}
		}
		if math.Abs(aug[pivot*(n+1)+col]) < 1e-14 {
			return nil, false
		}
		if pivot != col {
			for k := 0; k <= n; k++ {
				aug[col*(n+1)+k], aug[pivot*(n+1)+k] = aug[pivot*(n+1)+k], aug[col*(n+1)+k]
			}
		}
		for r := 0; r < n; r++ {
			if r == col {
				continue
			}
			f := aug[r*(n+1)+col] / aug[col*(n+1)+col]
			for k := col; k <= n; k++ {
				aug[r*(n+1)+k] -= f * aug[col*(n+1)+k]
			}
		}
	}

	x := make([]float64, n)
	for i := 0; i < n; i++ {
		x[i] = aug[i*(n+1)+n] / aug[i*(n+1)+i]
	}
	return x, true
}

// invertSymmetric4 求 4x4 对称正定矩阵的逆（协方差矩阵）。
func invertSymmetric4(m []float64) ([4][4]float64, bool) {
	var cov [4][4]float64
	for col := 0; col < 4; col++ {
		e := make([]float64, 4)
		e[col] = 1
		x, ok := gaussianSolve4(m, e)
		if !ok {
			return cov, false
		}
		for r := 0; r < 4; r++ {
			cov[r][col] = x[r]
		}
	}
	return cov, true
}
