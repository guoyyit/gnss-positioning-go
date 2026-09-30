package raim

import (
	"math"
	"time"

	"github.com/guoyyit/gnss-positioning-go/model"
	"github.com/guoyyit/gnss-positioning-go/position"
)

// Config 为 RAIM 完好性监测配置。
type Config struct {
	ProbabilityFalseAlarm float64 // 全局检验虚警概率
	SigmaRange            float64 // 标称伪距标准差，米
	HAL                   float64 // 水平告警限，米
	VAL                   float64 // 垂直告警限，米
	MaxFaults             int     // 最多尝试剔除的卫星数
	MinSatellites         int     // 完好性可用所需最少卫星数
	DriftWindow           int     // 慢漂移检测窗口（历元数）
	DriftThreshold        float64 // 慢漂移归一化统计量阈值
	FixedThreshold        float64 // >0 时使用固定全局检验阈值（无量纲卡方）而非卡方分位数
	SingleSatZThreshold   float64 // 单星识别的标准化残差门限；<=0 时按虚警概率自动计算
}

// DefaultConfig 返回默认 RAIM 配置。
func DefaultConfig() Config {
	return Config{
		ProbabilityFalseAlarm: 1e-6,
		SigmaRange:            3.0,
		HAL:                   35.0,
		VAL:                   50.0,
		MaxFaults:             2,
		MinSatellites:         5,
		DriftWindow:           8,
		DriftThreshold:        4.0,
		SingleSatZThreshold:   4.5,
	}
}

// DriftState 为单通道有界的慢漂移检测历史状态。
type DriftState struct {
	window    int
	threshold float64
	resid     []map[model.SatID]float64
}

// NewDriftState 构造慢漂移状态。
func NewDriftState(cfg Config) *DriftState {
	w := cfg.DriftWindow
	if w < 2 {
		w = 2
	}
	threshold := cfg.DriftThreshold
	if threshold <= 0 {
		threshold = DefaultConfig().DriftThreshold
	}
	return &DriftState{window: w, threshold: threshold, resid: make([]map[model.SatID]float64, 0, w)}
}

// Reset 清空历史窗口（故障确认或重配置后调用，避免旧残差污染）。
func (d *DriftState) Reset() {
	d.resid = d.resid[:0]
}

func (d *DriftState) push(residuals map[model.SatID]float64, sigma float64) (model.SatID, bool) {
	normalized := make(map[model.SatID]float64, len(residuals))
	for sat, v := range residuals {
		s := sigma
		if s <= 0 {
			s = 1
		}
		normalized[sat] = v / s
	}
	d.resid = append(d.resid, normalized)
	if len(d.resid) > d.window {
		d.resid = d.resid[len(d.resid)-d.window:]
	}
	if len(d.resid) < d.window {
		return "", false
	}

	// 对每颗卫星，对窗口内归一化残差做线性回归，斜率显著持续非零判为漂移。
	type acc struct {
		sumX, sumY, sumXY, sumXX float64
	}
	m := map[model.SatID]*acc{}
	n := float64(len(d.resid))
	for e, row := range d.resid {
		x := float64(e) - (n-1)/2
		for sat, y := range row {
			a := m[sat]
			if a == nil {
				a = &acc{}
				m[sat] = a
			}
			a.sumX += x
			a.sumY += y
			a.sumXY += x * y
			a.sumXX += x * x
		}
	}
	var best model.SatID
	bestScore := 0.0
	for sat, a := range m {
		if a.sumXX <= 0 {
			continue
		}
		slope := a.sumXY / a.sumXX
		score := math.Abs(slope) * math.Sqrt(n)
		if score > bestScore {
			bestScore = score
			best = sat
		}
	}
	if best != "" && bestScore > d.threshold {
		return best, true
	}
	return "", false
}

// Result 为 RAIM 评估输出。
type Result struct {
	Status          model.RAIMStatus
	FaultSatellites []model.SatID
	ExcludedSats    []model.SatID
	UsedSats        []model.SatID
	Statistic       float64
	Threshold       float64
	DegreesFreedom  int
	HPL             float64
	VPL             float64
	DriftDetected   bool

	Position   model.Vec3
	ClockBiasM float64
	Covariance [4][4]float64
	Sigma0     float64
	Residuals  map[model.SatID]float64
	OK         bool
	Reason     model.FailureReason
}

// Evaluate 执行单历元完好性检测、故障识别与剔除后重解。
func Evaluate(obs []model.Observation, epoch time.Time, pcfg position.Config, cfg Config, drift *DriftState) Result {
	if cfg.MaxFaults <= 0 {
		cfg.MaxFaults = DefaultConfig().MaxFaults
	}
	if cfg.MinSatellites <= 0 {
		cfg.MinSatellites = DefaultConfig().MinSatellites
	}

	all := solve(obs, epoch, pcfg)
	nAll := len(all.Used)

	threshold := cfg.FixedThreshold
	if threshold <= 0 {
		if nAll >= 5 {
			threshold = chiSquareQuantile(1-cfg.ProbabilityFalseAlarm, nAll-4)
		}
	}

	if !all.OK || nAll < cfg.MinSatellites {
		res := Result{
			Status:    model.RAIMUnavailable,
			Threshold: threshold,
			OK:        all.OK,
			Reason:    all.Reason,
		}
		if all.OK {
			res.Position = all.Position
			res.ClockBiasM = all.ClockBiasM
			res.Covariance = all.Covariance
			res.Sigma0 = all.Sigma0
			res.Residuals = all.Residuals
			res.UsedSats = all.Used
			res.Statistic = globalStatistic(all, cfg.SigmaRange)
			res.DegreesFreedom = nAll - 4
		}
		return res
	}

	stat := globalStatistic(all, cfg.SigmaRange)
	if stat <= threshold {
		driftSat, drifted := drift.push(all.Residuals, all.Sigma0)
		if drifted {
			if nAll-1 >= cfg.MinSatellites {
				remaining := removeSat(obs, driftSat)
				retry := solve(remaining, epoch, pcfg)
				retryThreshold := threshold
				if retry.OK && len(retry.Used) >= 5 {
					retryThreshold = chiSquareQuantile(1-cfg.ProbabilityFalseAlarm, len(retry.Used)-4)
				}
				if retry.OK && globalStatistic(retry, cfg.SigmaRange) <= retryThreshold {
					drift.Reset()
					res := Result{
						Status:          model.RAIMFaultExcluded,
						FaultSatellites: []model.SatID{driftSat},
						ExcludedSats:    []model.SatID{driftSat},
						UsedSats:        retry.Used,
						Statistic:       globalStatistic(retry, cfg.SigmaRange),
						Threshold:       retryThreshold,
						DegreesFreedom:  len(retry.Used) - 4,
						Position:        retry.Position,
						ClockBiasM:      retry.ClockBiasM,
						Covariance:      retry.Covariance,
						Sigma0:          retry.Sigma0,
						Residuals:       retry.Residuals,
						DriftDetected:   true,
						OK:              true,
						Reason:          model.FailureNone,
					}
					attachProtectionLevels(&res, cfg)
					return res
				}
			}
			drift.Reset()
			return Result{
				Status:          model.RAIMFaultDetected,
				FaultSatellites: []model.SatID{driftSat},
				DriftDetected:   true,
				UsedSats:        all.Used,
				Statistic:       stat,
				Threshold:       threshold,
				DegreesFreedom:  nAll - 4,
				Position:        all.Position,
				ClockBiasM:      all.ClockBiasM,
				Covariance:      all.Covariance,
				Sigma0:          all.Sigma0,
				Residuals:       all.Residuals,
				OK:              true,
				Reason:          model.FailureNone,
			}
		}

		res := Result{
			Status:         model.RAIMOK,
			UsedSats:       all.Used,
			Statistic:      stat,
			Threshold:      threshold,
			DegreesFreedom: nAll - 4,
			Position:       all.Position,
			ClockBiasM:     all.ClockBiasM,
			Covariance:     all.Covariance,
			Sigma0:         all.Sigma0,
			Residuals:      all.Residuals,
			OK:             true,
			Reason:         model.FailureNone,
		}
		attachProtectionLevels(&res, cfg)
		return res
	}

	// 前向搜索：逐颗加入嫌疑卫星并在剔除后重解。
	current := all
	excluded := []model.SatID{}
	remaining := append([]model.Observation(nil), obs...)
	faults := []model.SatID{}

	for depth := 0; depth < cfg.MaxFaults; depth++ {
		candidate, bestStat := pickCandidate(current, cfg)
		zThreshold := cfg.SingleSatZThreshold
		if zThreshold <= 0 {
			zThreshold = DefaultConfig().SingleSatZThreshold
		}
		if candidate == "" || bestStat < zThreshold {
			break
		}
		faults = append(faults, candidate)
		excluded = append(excluded, candidate)
		remaining = removeSat(remaining, candidate)

		if len(remaining) < pcfg.MinSatellites {
			break
		}
		retry := solve(remaining, epoch, pcfg)
		if !retry.OK {
			break
		}
		current = retry
		retryStat := globalStatistic(retry, cfg.SigmaRange)
		retryThreshold := cfg.FixedThreshold
		if retryThreshold <= 0 && len(retry.Used) >= 5 {
			retryThreshold = chiSquareQuantile(1-cfg.ProbabilityFalseAlarm, len(retry.Used)-4)
		}
		_ = bestStat
		if retryStat <= retryThreshold {
			res := Result{
				Status:          model.RAIMFaultExcluded,
				FaultSatellites: faults,
				ExcludedSats:    excluded,
				UsedSats:        retry.Used,
				Statistic:       retryStat,
				Threshold:       retryThreshold,
				DegreesFreedom:  len(retry.Used) - 4,
				Position:        retry.Position,
				ClockBiasM:      retry.ClockBiasM,
				Covariance:      retry.Covariance,
				Sigma0:          retry.Sigma0,
				Residuals:       retry.Residuals,
				OK:              true,
				Reason:          model.FailureNone,
			}
			drift.Reset()
			attachProtectionLevels(&res, cfg)
			return res
		}
		stat = retryStat
	}

	// 能检测到但无法在保持足够几何的前提下完成识别/剔除。
	return Result{
		Status:          model.RAIMFaultDetected,
		FaultSatellites: faults,
		ExcludedSats:    excluded,
		UsedSats:        current.Used,
		Statistic:       stat,
		Threshold:       threshold,
		DegreesFreedom:  nAll - 4,
		Position:        current.Position,
		ClockBiasM:      current.ClockBiasM,
		Covariance:      current.Covariance,
		Sigma0:          current.Sigma0,
		Residuals:       current.Residuals,
		OK:              current.OK,
		Reason:          model.FailureNoConvergence,
	}
}

type solveResult struct {
	OK         bool
	Reason     model.FailureReason
	Position   model.Vec3
	ClockBiasM float64
	Used       []model.SatID
	Residuals  map[model.SatID]float64
	Covariance [4][4]float64
	Sigma0     float64
}

func solve(obs []model.Observation, epoch time.Time, cfg position.Config) solveResult {
	r := position.Solve(obs, epoch, cfg)
	return solveResult{
		OK:         r.OK,
		Reason:     r.Reason,
		Position:   r.Position,
		ClockBiasM: r.ClockBiasM,
		Used:       r.Used,
		Residuals:  r.Residuals,
		Covariance: r.Covariance,
		Sigma0:     r.Sigma0,
	}
}

func globalStatistic(r solveResult, sigma float64) float64 {
	if sigma <= 0 {
		sigma = DefaultConfig().SigmaRange
	}
	s := 0.0
	for _, sat := range r.Used {
		z := r.Residuals[sat] / sigma
		s += z * z
	}
	return s
}

// pickCandidate 返回归一化残差最大的卫星及其统计量。
func pickCandidate(r solveResult, cfg Config) (model.SatID, float64) {
	sigma := cfg.SigmaRange
	if sigma <= 0 {
		sigma = DefaultConfig().SigmaRange
	}
	var best model.SatID
	bestStat := 0.0
	for _, sat := range r.Used {
		v := math.Abs(r.Residuals[sat]) / sigma
		if v > bestStat {
			bestStat = v
			best = sat
		}
	}
	return best, bestStat
}

func removeSat(obs []model.Observation, target model.SatID) []model.Observation {
	out := make([]model.Observation, 0, len(obs))
	for _, o := range obs {
		if o.Sat != target {
			out = append(out, o)
		}
	}
	return out
}

// attachProtectionLevels 基于剔除后协方差计算保护水平（加权 LS 协方差，单位米²）。
func attachProtectionLevels(res *Result, cfg Config) {
	if res.DegreesFreedom <= 0 {
		return
	}
	cov := res.Covariance

	g := position.ECEFToGeo(res.Position)
	r := position.ENUMatrix(g)
	var enu [3][3]float64
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			v := 0.0
			for a := 0; a < 3; a++ {
				for b := 0; b < 3; b++ {
					v += r[i][a] * cov[a][b] * r[j][b]
				}
			}
			enu[i][j] = v
		}
	}

	sigmaH := math.Sqrt(math.Max(0, enu[0][0]+enu[1][1]))
	sigmaV := math.Sqrt(math.Max(0, enu[2][2]))

	k := normalQuantile(1 - cfg.ProbabilityFalseAlarm/2)
	if math.IsInf(k, 0) || k <= 0 {
		k = 3.72
	}
	res.HPL = k * sigmaH
	res.VPL = k * sigmaV
}
