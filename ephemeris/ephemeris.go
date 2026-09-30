package ephemeris

import (
	"math"
	"time"

	"github.com/guoyyit/gnss-positioning-go/model"
)

// WGS84 / 广播星历传播常数。
const (
	EarthGM       = 3.986004418e14  // m^3/s^2
	EarthRotation = 7.2921151467e-5 // rad/s
	SpeedOfLight  = 299792458.0     // m/s
	RelativisticF = -4.442807633e-10
)

// PositionAt 按广播星历计算卫星在 t 时刻的 ECEF 位置。
func PositionAt(eph *model.Ephemeris, t time.Time) (model.Vec3, error) {
	p, _, err := propagate(eph, t)
	return p, err
}

// ClockBias 返回 t 时刻卫星钟差（折算为米）。
func ClockBias(eph *model.Ephemeris, t time.Time) float64 {
	if eph == nil || eph.SemiMajorAxis <= 0 {
		return 0
	}
	_, ek, err := propagate(eph, t)
	if err != nil {
		return 0
	}
	dtc := t.Sub(eph.Toc).Seconds()
	dt := eph.Af0 + eph.Af1*dtc + eph.Af2*dtc*dtc
	// 相对论改正。
	dt += RelativisticF * eph.Eccentricity * math.Sqrt(eph.SemiMajorAxis) * math.Sin(ek)
	return dt * SpeedOfLight
}

// RotateEarth 对发射时刻卫星位置施加信号传播期间的地球自转改正（Sagnac）。
// travelTime 为信号从卫星到接收机的传播时间（秒）。
func RotateEarth(pos model.Vec3, travelTime float64) model.Vec3 {
	theta := EarthRotation * travelTime
	c, s := math.Cos(theta), math.Sin(theta)
	return model.Vec3{
		X: c*pos.X + s*pos.Y,
		Y: -s*pos.X + c*pos.Y,
		Z: pos.Z,
	}
}

func propagate(eph *model.Ephemeris, t time.Time) (model.Vec3, float64, error) {
	if eph == nil || eph.SemiMajorAxis <= 0 {
		return model.Vec3{}, 0, errInvalidEphemeris
	}

	// 相对参考历元时间，限制在一个 GPS 周内。
	tk := t.Sub(eph.Toe).Seconds()
	tk = wrapWeek(tk)

	a := eph.SemiMajorAxis
	n0 := math.Sqrt(EarthGM / (a * a * a))
	n := n0 + eph.DeltaN

	mk := eph.MeanAnomaly0 + n*tk
	ek := solveKepler(mk, eph.Eccentricity)

	e := eph.Eccentricity
	sinE, cosE := math.Sin(ek), math.Cos(ek)
	sqrt1e2 := math.Sqrt(1 - e*e)

	// 真近点角。
	nuk := math.Atan2(sqrt1e2*sinE, cosE-e)
	phi := nuk + eph.ArgPerigee

	twoPhi := 2 * phi
	du := eph.Cus*math.Sin(twoPhi) + eph.Cuc*math.Cos(twoPhi)
	dr := eph.Crs*math.Sin(twoPhi) + eph.Crc*math.Cos(twoPhi)
	di := eph.Cis*math.Sin(twoPhi) + eph.Cic*math.Cos(twoPhi)

	u := phi + du
	r := a*(1-e*cosE) + dr
	inc := eph.Inclination + di + eph.InclinationRate*tk

	xp := r * math.Cos(u)
	yp := r * math.Sin(u)

	omega := eph.RAAN + (eph.RAANRate-EarthRotation)*tk - EarthRotation*gpsWeekSeconds(eph.Toe)

	sinO, cosO := math.Sin(omega), math.Cos(omega)
	sinI, cosI := math.Sin(inc), math.Cos(inc)

	pos := model.Vec3{
		X: xp*cosO - yp*cosI*sinO,
		Y: xp*sinO + yp*cosI*cosO,
		Z: yp * sinI,
	}
	return pos, ek, nil
}

func solveKepler(m, e float64) float64 {
	ek := m
	for i := 0; i < 32; i++ {
		f := ek - e*math.Sin(ek) - m
		fp := 1 - e*math.Cos(ek)
		next := ek - f/fp
		if math.Abs(next-ek) < 1e-13 {
			return next
		}
		ek = next
	}
	return ek
}

func wrapWeek(tk float64) float64 {
	const halfWeek = 302400.0
	for tk > halfWeek {
		tk -= 2 * halfWeek
	}
	for tk < -halfWeek {
		tk += 2 * halfWeek
	}
	return tk
}

var gpsEpoch = time.Date(1980, time.January, 6, 0, 0, 0, 0, time.UTC)

func gpsWeekSeconds(t time.Time) float64 {
	s := t.Sub(gpsEpoch).Seconds()
	const week = 604800.0
	s = math.Mod(s, week)
	if s < 0 {
		s += week
	}
	return s
}
