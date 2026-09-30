package gnss

import "math"

const speedOfLight = 2.99792458e8

// SatellitePosition computes the ECEF position of a satellite at time t.
func SatellitePosition(e Ephemeris, t GNSSTime) Vec3 {
	tk := t.Seconds - e.Toe
	a := e.SqrtA * e.SqrtA
	n0 := math.Sqrt(muEarth / (a * a * a))
	n := n0 + e.DeltaN
	mk := e.M0 + n*tk

	ek := mk
	for i := 0; i < 32; i++ {
		next := mk + e.Ecc*math.Sin(ek)
		if math.Abs(next-ek) < 1e-13 {
			ek = next
			break
		}
		ek = next
	}

	sinE, cosE := math.Sin(ek), math.Cos(ek)
	sqrtOneE2 := math.Sqrt(1.0 - e.Ecc*e.Ecc)
	nuk := math.Atan2(sqrtOneE2*sinE, cosE-e.Ecc)
	phi := nuk + e.ArgPeri
	sin2Phi, cos2Phi := math.Sin(2.0*phi), math.Cos(2.0*phi)

	uk := phi + e.Cus*sin2Phi + e.Cuc*cos2Phi
	rk := a*(1.0-e.Ecc*cosE) + e.Crs*sin2Phi + e.Crc*cos2Phi
	ik := e.I0 + e.Cis*sin2Phi + e.Cic*cos2Phi + e.IDot*tk

	xp := rk * math.Cos(uk)
	yp := rk * math.Sin(uk)

	omegaK := e.Omega0 + (e.OmegaDot-omegaEarth)*tk - omegaEarth*e.Toe
	sinO, cosO := math.Sin(omegaK), math.Cos(omegaK)
	sinI, cosI := math.Sin(ik), math.Cos(ik)
	return Vec3{
		X: xp*cosO - yp*cosI*sinO,
		Y: xp*sinO + yp*cosI*cosO,
		Z: yp * sinI,
	}
}

// SatelliteClockBias returns the satellite clock correction in meters at time t.
func SatelliteClockBias(e Ephemeris, t GNSSTime) float64 {
	dt := t.Seconds - e.Toc
	a := e.SqrtA * e.SqrtA
	tk := t.Seconds - e.Toe
	n0 := math.Sqrt(muEarth / (a * a * a))
	ek := e.M0 + (n0+e.DeltaN)*tk
	for i := 0; i < 16; i++ {
		next := e.M0 + (n0+e.DeltaN)*tk + e.Ecc*math.Sin(ek)
		if math.Abs(next-ek) < 1e-13 {
			ek = next
			break
		}
		ek = next
	}
	relSeconds := -4.442807633e-10 * e.Ecc * e.SqrtA * math.Sin(ek)
	clockSeconds := e.Af0 + e.Af1*dt + e.Af2*dt*dt + relSeconds - e.TGD
	return speedOfLight * clockSeconds
}
