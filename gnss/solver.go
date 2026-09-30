package gnss

import "math"

// SolverConfig configures the iterative least-squares position solver.
type SolverConfig struct {
	MinSatellites  int
	MaxIterations  int
	ConvergenceTol float64
	SigmaUERE      float64
	RAIM           RAIMConfig
}

type satTerm struct {
	prn       int
	satPos    Vec3
	rangeMeas float64
}

type lsResult struct {
	ok          bool
	converged   bool
	rankOK      bool
	pos         Vec3
	clockBias   float64
	prns        []int
	residuals   []float64
	rmsResidual float64
	maxResidual float64
	cov         [4][4]float64
	dop         Quality
}

// SolveEpoch computes a single-point position solution for one epoch.
func SolveEpoch(data EpochData, cfg SolverConfig) Solution {
	minSats := cfg.MinSatellites
	if minSats < 4 {
		minSats = 4
	}
	terms, flags := prepareTerms(data)
	if len(terms) < minSats {
		return Solution{
			Epoch:  data.Epoch,
			Time:   data.Time,
			Status: StatusInsufficientGeometry,
			Reason: "fewer than four valid observations",
			Flags:  flags,
		}
	}

	res := leastSquares(terms, cfg)
	sol := Solution{
		Epoch: data.Epoch,
		Time:  data.Time,
		Flags: flags,
	}
	if !res.rankOK {
		sol.Status = StatusInsufficientGeometry
		sol.Reason = "observation geometry matrix is rank deficient"
		return sol
	}
	if !res.converged {
		sol.Status = StatusNoConvergence
		sol.Reason = "least-squares iteration did not converge"
		return sol
	}
	sol.Status = StatusOK
	sol.Position = res.pos
	sol.LLA = ECEFToLLA(res.pos)
	sol.ClockBias = res.clockBias
	sol.UsedPRNs = res.prns
	sol.Quality = res.dop
	return sol
}

func prepareTerms(data EpochData) ([]satTerm, QualityFlag) {
	var flags QualityFlag
	ephByPRN := make(map[int]Ephemeris, len(data.Eph))
	for _, e := range data.Eph {
		ephByPRN[e.PRN] = e
	}
	seen := make(map[int]bool, len(data.Obs))
	terms := make([]satTerm, 0, len(data.Obs))
	for _, o := range data.Obs {
		if seen[o.PRN] {
			flags |= FlagDuplicate
			continue
		}
		seen[o.PRN] = true
		e, ok := ephByPRN[o.PRN]
		if !ok || !e.Healthy || o.Pseudorange <= 0 {
			flags |= FlagMissingObs
			continue
		}
		terms = append(terms, satTerm{
			prn:       o.PRN,
			satPos:    SatellitePosition(e, o.Time),
			rangeMeas: o.Pseudorange + SatelliteClockBias(e, o.Time),
		})
	}
	return terms, flags
}

func leastSquares(terms []satTerm, cfg SolverConfig) lsResult {
	res := lsResult{prns: make([]int, 0, len(terms))}
	n := len(terms)
	if n < 4 {
		return res
	}
	maxIter := cfg.MaxIterations
	if maxIter <= 0 {
		maxIter = 20
	}
	tol := cfg.ConvergenceTol
	if tol <= 0 {
		tol = 1e-4
	}

	pos := Vec3{}
	clockBias := 0.0
	var dx [4]float64
	for iter := 0; iter < maxIter; iter++ {
		h := make([][4]float64, n)
		y := make([]float64, n)
		for i, term := range terms {
			geo := vecSub(term.satPos, pos)
			r := vecNorm(geo)
			h[i] = [4]float64{-geo.X / r, -geo.Y / r, -geo.Z / r, 1}
			y[i] = term.rangeMeas - (r + clockBias)
		}
		var ata [4][4]float64
		var aty [4]float64
		for i := 0; i < n; i++ {
			for j := 0; j < 4; j++ {
				aty[j] += h[i][j] * y[i]
				for k := 0; k < 4; k++ {
					ata[j][k] += h[i][j] * h[i][k]
				}
			}
		}
		var ok bool
		dx, ok = solveSym4(ata, aty)
		if !ok {
			return res
		}
		pos = vecAdd(pos, Vec3{X: dx[0], Y: dx[1], Z: dx[2]})
		clockBias += dx[3]
		if math.Sqrt(dx[0]*dx[0]+dx[1]*dx[1]+dx[2]*dx[2]) < tol && math.Abs(dx[3]) < tol {
			res.converged = true
			break
		}
	}

	res.pos = pos
	res.clockBias = clockBias
	res.prns = res.prns[:0]
	residuals := make([]float64, n)
	var ata [4][4]float64
	sumSq := 0.0
	for i, term := range terms {
		geo := vecSub(term.satPos, pos)
		r := vecNorm(geo)
		residuals[i] = term.rangeMeas - (r + clockBias)
		sumSq += residuals[i] * residuals[i]
		var row [4]float64
		row = [4]float64{-geo.X / r, -geo.Y / r, -geo.Z / r, 1}
		for j := 0; j < 4; j++ {
			for k := 0; k < 4; k++ {
				ata[j][k] += row[j] * row[k]
			}
		}
		res.prns = append(res.prns, term.prn)
		res.maxResidual = math.Max(res.maxResidual, math.Abs(residuals[i]))
	}
	inv, ok := invert4(ata)
	if !ok {
		return res
	}
	res.rankOK = true
	sigma2 := sumSq / float64(n-4)
	sigmaUERE := cfg.SigmaUERE
	if sigmaUERE <= 0 {
		sigmaUERE = 1.0
	}
	scale := sigma2
	if scale <= 0 || math.IsNaN(scale) {
		scale = sigmaUERE * sigmaUERE
	}
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			res.cov[i][j] = inv[i][j] * scale
		}
	}
	res.rmsResidual = math.Sqrt(sumSq / float64(n))
	res.dop = dopFromGeometry(inv, pos, n)
	res.dop.RMSResidual = res.rmsResidual
	res.dop.MaxResidual = res.maxResidual
	res.dop.Covariance = res.cov
	res.dop.SigmaUERE = sigmaUERE
	res.ok = res.converged
	res.residuals = residuals
	return res
}

func dopFromGeometry(inv [4][4]float64, pos Vec3, n int) Quality {
	lla := ECEFToLLA(pos)
	sinLat, cosLat := math.Sin(lla.Lat), math.Cos(lla.Lat)
	sinLon, cosLon := math.Sin(lla.Lon), math.Cos(lla.Lon)
	r := [3][3]float64{
		{-sinLon, cosLon, 0},
		{-sinLat * cosLon, -sinLat * sinLon, cosLat},
		{cosLat * cosLon, cosLat * sinLon, sinLat},
	}
	rt := transpose3(r)
	q := [3][3]float64{
		{inv[0][0], inv[0][1], inv[0][2]},
		{inv[1][0], inv[1][1], inv[1][2]},
		{inv[2][0], inv[2][1], inv[2][2]},
	}
	qenu := multiply3(multiply3(r, q), rt)
	hDOP := math.Sqrt(qenu[0][0] + qenu[1][1])
	vDOP := math.Sqrt(qenu[2][2])
	tDOP := math.Sqrt(inv[3][3])
	pDOP := math.Sqrt(qenu[0][0] + qenu[1][1] + qenu[2][2])
	gDOP := math.Sqrt(inv[0][0] + inv[1][1] + inv[2][2] + inv[3][3])
	return Quality{
		N:    n,
		GDOP: gDOP,
		PDOP: pDOP,
		HDOP: hDOP,
		VDOP: vDOP,
		TDOP: tDOP,
	}
}

func transpose3(a [3][3]float64) [3][3]float64 {
	var out [3][3]float64
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			out[i][j] = a[j][i]
		}
	}
	return out
}

func multiply3(a, b [3][3]float64) [3][3]float64 {
	var out [3][3]float64
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			for k := 0; k < 3; k++ {
				out[i][j] += a[i][k] * b[k][j]
			}
		}
	}
	return out
}
