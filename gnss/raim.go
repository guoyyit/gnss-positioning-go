package gnss

import "math"

// RAIMConfig configures fault detection, exclusion and protection levels.
type RAIMConfig struct {
	Enabled           bool
	MaxFaults         int
	GlobalThreshold   float64
	SlipThreshold     float64
	DriftWindowEpochs int
	DriftRateLimit    float64
	HorizontalAlert   float64
	VerticalAlert     float64
	SigmaUERE         float64
}

type raimInput struct {
	terms      []satTerm
	baseline   lsResult
	history    map[int][]float64
	excluded   map[int]bool
	excludePRN func(prn int)
}

// runRAIM performs fault detection, exclusion and protection-level evaluation.
func runRAIM(in raimInput, cfg RAIMConfig, solver SolverConfig) (RAIMResult, lsResult) {
	res := RAIMResult{}
	baseline := in.baseline
	if !cfg.Enabled {
		return res, baseline
	}
	sigma := cfg.SigmaUERE
	if sigma <= 0 {
		sigma = solver.SigmaUERE
	}
	if sigma <= 0 {
		sigma = 1
	}
	threshold := cfg.GlobalThreshold
	if threshold <= 0 {
		threshold = chiSquareInvCDF(float64(len(in.terms)-4), 1e-5) * sigma * sigma
	}

	current := in.terms
	ls := baseline
	res.Available = len(current) >= 5
	res.HPL, res.VPL = protectionLevels(current, ls, sigma, cfg)

	sse := sumSquaredResiduals(ls)
	if sse <= threshold {
		driftPRNs := detectDrift(in.history, baseline, cfg)
		if len(driftPRNs) == 0 {
			res.GlobalStat = math.Sqrt(sse) / sigma
			res.GlobalThreshold = math.Sqrt(threshold) / sigma
			return res, ls
		}
		res.Detected = true
		for _, prn := range driftPRNs {
			current = excludeTerm(current, prn)
		}
		if len(current) >= 4 {
			trial := leastSquares(current, solver)
			if trial.rankOK && trial.converged && sumSquaredResiduals(trial) <= threshold {
				ls = trial
				sse = sumSquaredResiduals(ls)
				res.FaultPRNs = driftPRNs
				res.ExcludedPRNs = append([]int(nil), driftPRNs...)
				res.GlobalStat = math.Sqrt(sse) / sigma
				res.GlobalThreshold = math.Sqrt(threshold) / sigma
				res.HPL, res.VPL = protectionLevels(current, ls, sigma, cfg)
				res.Recovered = true
				if in.excludePRN != nil {
					for _, prn := range driftPRNs {
						in.excludePRN(prn)
					}
				}
				return res, ls
			}
		}
		res.GlobalStat = math.Sqrt(sse) / sigma
		res.GlobalThreshold = math.Sqrt(threshold) / sigma
		return res, ls
	}
	res.Detected = true

	maxFaults := cfg.MaxFaults
	if maxFaults <= 0 {
		maxFaults = 2
	}
	active := make(map[int]bool)
	for attempt := 0; attempt < maxFaults && len(current) >= 5; attempt++ {
		bestPRN := -1
		bestSSE := math.Inf(1)
		var bestLS lsResult
		for _, term := range current {
			if active[term.prn] {
				continue
			}
			trial := excludeTerm(current, term.prn)
			if len(trial) < 4 {
				continue
			}
			trialLS := leastSquares(trial, solver)
			if !trialLS.rankOK || !trialLS.converged {
				continue
			}
			trialSSE := sumSquaredResiduals(trialLS)
			if trialSSE < bestSSE {
				bestPRN = term.prn
				bestLS = trialLS
				bestSSE = trialSSE
			}
		}
		if bestPRN < 0 {
			break
		}
		if bestSSE <= threshold {
			active[bestPRN] = true
			current = excludeTerm(current, bestPRN)
			ls = bestLS
			sse = bestSSE
			break
		}
		// Single-satellite exclusion is ambiguous for multiple opposing
		// biases; search all pairs before accepting a wrong greedy pick.
		if maxFaults >= 2 && len(current) >= 6 {
			pairSSE := bestSSE
			var pairLS lsResult
			pairA, pairB := bestPRN, -1
			for i := 0; i < len(current); i++ {
				for j := i + 1; j < len(current); j++ {
					trial := excludeTerm(excludeTerm(current, current[i].prn), current[j].prn)
					if len(trial) < 4 {
						continue
					}
					trialLS := leastSquares(trial, solver)
					if !trialLS.rankOK || !trialLS.converged {
						continue
					}
					if s := sumSquaredResiduals(trialLS); s < pairSSE {
						pairSSE = s
						pairLS = trialLS
						pairA, pairB = current[i].prn, current[j].prn
					}
				}
			}
			if pairB >= 0 && pairSSE <= threshold {
				active[pairA] = true
				active[pairB] = true
				current = excludeTerm(excludeTerm(current, pairA), pairB)
				ls = pairLS
				sse = pairSSE
				break
			}
		}
		active[bestPRN] = true
		current = excludeTerm(current, bestPRN)
		ls = bestLS
		sse = bestSSE
	}

	if sse <= threshold {
		for prn := range active {
			res.FaultPRNs = append(res.FaultPRNs, prn)
			if in.excludePRN != nil {
				in.excludePRN(prn)
			}
		}
		sortInts(res.FaultPRNs)
		res.ExcludedPRNs = append([]int(nil), res.FaultPRNs...)
		res.Recovered = true
	}
	res.GlobalStat = math.Sqrt(sse) / sigma
	res.GlobalThreshold = math.Sqrt(threshold) / sigma
	res.HPL, res.VPL = protectionLevels(current, ls, sigma, cfg)
	return res, ls
}

func excludeTerm(terms []satTerm, prn int) []satTerm {
	out := make([]satTerm, 0, len(terms)-1)
	for _, term := range terms {
		if term.prn != prn {
			out = append(out, term)
		}
	}
	return out
}

func sumSquaredResiduals(r lsResult) float64 {
	sum := 0.0
	for _, v := range r.residuals {
		sum += v * v
	}
	return sum
}

func detectDrift(history map[int][]float64, ls lsResult, cfg RAIMConfig) []int {
	window := cfg.DriftWindowEpochs
	if window < 2 {
		window = 5
	}
	if cfg.DriftRateLimit <= 0 {
		return nil
	}
	var drift []int
	for i, prn := range ls.prns {
		series := history[prn]
		if len(series) < window-1 {
			continue
		}
		series = append(append([]float64(nil), series[len(series)-(window-1):]...), ls.residuals[i])
		n := float64(len(series))
		var sumX, sumY, sumXY, sumXX float64
		for j, y := range series {
			x := float64(j)
			sumX += x
			sumY += y
			sumXY += x * y
			sumXX += x * x
		}
		denom := n*sumXX - sumX*sumX
		if denom == 0 {
			continue
		}
		slope := (n*sumXY - sumX*sumY) / denom
		if math.Abs(slope) >= cfg.DriftRateLimit && math.Abs(ls.residuals[i]) >= cfg.SlipThreshold {
			drift = append(drift, prn)
		}
	}
	sortInts(drift)
	return drift
}

func protectionLevels(terms []satTerm, ls lsResult, sigma float64, cfg RAIMConfig) (hpl, vpl float64) {
	if !ls.rankOK || len(terms) < 5 {
		return 0, 0
	}
	pbias := cfg.HorizontalAlert
	if pbias <= 0 {
		pbias = 500
	}
	n := len(terms)
	h := make([][4]float64, n)
	for i, term := range terms {
		geo := vecSub(term.satPos, ls.pos)
		r := vecNorm(geo)
		h[i] = [4]float64{-geo.X / r, -geo.Y / r, -geo.Z / r, 1}
	}
	var ata [4][4]float64
	for i := 0; i < n; i++ {
		for j := 0; j < 4; j++ {
			for k := 0; k < 4; k++ {
				ata[j][k] += h[i][j] * h[i][k]
			}
		}
	}
	inv, ok := invert4(ata)
	if !ok {
		return 0, 0
	}
	maxHS := 0.0
	maxVS := 0.0
	for i := 0; i < n; i++ {
		var col [4]float64
		for j := 0; j < 4; j++ {
			for k := 0; k < 4; k++ {
				col[j] += inv[j][k] * h[i][k]
			}
		}
		denom := math.Sqrt(max(1e-12, 1-(h[i][0]*col[0]+h[i][1]*col[1]+h[i][2]*col[2]+h[i][3]*col[3])))
		hSlope := math.Sqrt(col[0]*col[0]+col[1]*col[1]) / denom
		vSlope := math.Abs(col[2]) / denom
		if hSlope > maxHS {
			maxHS = hSlope
		}
		if vSlope > maxVS {
			maxVS = vSlope
		}
	}
	hpl = sigma*ls.dop.HDOP*2.0 + maxHS*pbias
	vpl = sigma*ls.dop.VDOP*2.0 + maxVS*pbias
	return hpl, vpl
}

func sortInts(a []int) {
	for i := 1; i < len(a); i++ {
		for j := i; j > 0 && a[j-1] > a[j]; j-- {
			a[j-1], a[j] = a[j], a[j-1]
		}
	}
}
