package gnss

import "math"

// WGS-84 ellipsoid constants.
const (
	wgs84A     = 6378137.0
	wgs84F     = 1.0 / 298.257223563
	wgs84E2    = wgs84F * (2.0 - wgs84F)
	omegaEarth = 7.2921151467e-5
	muEarth    = 3.986005e14
)

// ECEFToLLA converts an ECEF position (meters) to geodetic coordinates.
func ECEFToLLA(v Vec3) LLA {
	lon := math.Atan2(v.Y, v.X)
	p := math.Hypot(v.X, v.Y)
	lat := math.Atan2(v.Z, p*(1.0-wgs84E2))
	var alt float64
	for i := 0; i < 8; i++ {
		sinLat := math.Sin(lat)
		n := wgs84A / math.Sqrt(1.0-wgs84E2*sinLat*sinLat)
		alt = p/math.Cos(lat) - n
		next := math.Atan2(v.Z, p*(1.0-wgs84E2*n/(n+alt)))
		if math.Abs(next-lat) < 1e-12 {
			lat = next
			break
		}
		lat = next
	}
	n := wgs84A / math.Sqrt(1.0-wgs84E2*math.Sin(lat)*math.Sin(lat))
	alt = p/math.Cos(lat) - n
	return LLA{Lat: lat, Lon: lon, Alt: alt}
}

// LLAToECEF converts geodetic coordinates to an ECEF position (meters).
func LLAToECEF(l LLA) Vec3 {
	sinLat, cosLat := math.Sin(l.Lat), math.Cos(l.Lat)
	sinLon, cosLon := math.Sin(l.Lon), math.Cos(l.Lon)
	n := wgs84A / math.Sqrt(1.0-wgs84E2*sinLat*sinLat)
	return Vec3{
		X: (n + l.Alt) * cosLat * cosLon,
		Y: (n + l.Alt) * cosLat * sinLon,
		Z: (n*(1.0-wgs84E2) + l.Alt) * sinLat,
	}
}

func vecSub(a, b Vec3) Vec3 {
	return Vec3{X: a.X - b.X, Y: a.Y - b.Y, Z: a.Z - b.Z}
}

func vecAdd(a, b Vec3) Vec3 {
	return Vec3{X: a.X + b.X, Y: a.Y + b.Y, Z: a.Z + b.Z}
}

func vecScale(a Vec3, s float64) Vec3 {
	return Vec3{X: a.X * s, Y: a.Y * s, Z: a.Z * s}
}

func vecDot(a, b Vec3) float64 {
	return a.X*b.X + a.Y*b.Y + a.Z*b.Z
}

func vecNorm(a Vec3) float64 {
	return math.Sqrt(vecDot(a, a))
}
