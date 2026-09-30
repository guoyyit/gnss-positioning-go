package position

import (
	"math"

	"github.com/guoyyit/gnss-positioning-go/model"
)

// DOP 为几何精度因子集合。
type DOP struct {
	G float64
	P float64
	H float64
	V float64
	T float64
}

// ComputeDOP 根据协方差与站心方向计算各类 DOP。
func ComputeDOP(cov [4][4]float64, latDeg, lonDeg float64) DOP {
	r := ENUMatrix(model.GeoPoint{LatitudeDeg: latDeg, LongitudeDeg: lonDeg})

	// C_ENU = R C_ECEF Rᵀ（仅位置三维）。
	var enu [3][3]float64
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			s := 0.0
			for a := 0; a < 3; a++ {
				for b := 0; b < 3; b++ {
					s += r[i][a] * cov[a][b] * r[j][b]
				}
			}
			enu[i][j] = s
		}
	}

	tracePos := cov[0][0] + cov[1][1] + cov[2][2]
	traceAll := tracePos + cov[3][3]

	p := math.Sqrt(math.Max(0, tracePos))
	h := math.Sqrt(math.Max(0, enu[0][0]+enu[1][1]))
	v := math.Sqrt(math.Max(0, enu[2][2]))
	t := math.Sqrt(math.Max(0, cov[3][3]))
	g := math.Sqrt(math.Max(0, traceAll))

	return DOP{G: g, P: p, H: h, V: v, T: t}
}

// BuildQualityReport 汇总精度、残差与协方差报告。
func BuildQualityReport(used []model.SatID, residuals map[model.SatID]float64, cov [4][4]float64, sigma0 float64, geo model.GeoPoint) model.QualityReport {
	dop := ComputeDOP(cov, geo.LatitudeDeg, geo.LongitudeDeg)

	sumSq := 0.0
	maxAbs := 0.0
	for _, sat := range used {
		v, ok := residuals[sat]
		if !ok {
			continue
		}
		sumSq += v * v
		if math.Abs(v) > maxAbs {
			maxAbs = math.Abs(v)
		}
	}
	rms := 0.0
	if len(used) > 0 {
		rms = math.Sqrt(sumSq / float64(len(used)))
	}

	return model.QualityReport{
		NumSatellites: len(used),
		GDOP:          dop.G,
		PDOP:          dop.P,
		HDOP:          dop.H,
		VDOP:          dop.V,
		TDOP:          dop.T,
		RMSResidual:   rms,
		MaxResidual:   maxAbs,
		Sigma0:        sigma0,
		Covariance:    cov,
	}
}
