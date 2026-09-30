package position

import (
	"math"

	"github.com/guoyyit/gnss-positioning-go/model"
)

// WGS84 椭球参数。
const (
	WGS84A  = 6378137.0
	WGS84F  = 1.0 / 298.257223563
	WGS84E2 = WGS84F * (2 - WGS84F)
)

// ECEFToGeo 将 ECEF 坐标转换为 WGS84 大地坐标（纬度、经度为度）。
func ECEFToGeo(v model.Vec3) model.GeoPoint {
	lon := math.Atan2(v.Y, v.X)
	p := math.Hypot(v.X, v.Y)

	// 初始纬度（忽略扁率）。
	lat := math.Atan2(v.Z, p*(1-WGS84E2))
	var h float64
	for i := 0; i < 8; i++ {
		sinLat := math.Sin(lat)
		n := WGS84A / math.Sqrt(1-WGS84E2*sinLat*sinLat)
		h = p/math.Cos(lat) - n
		next := math.Atan2(v.Z, p*(1-WGS84E2*n/(n+h)))
		if math.Abs(next-lat) < 1e-12 {
			lat = next
			break
		}
		lat = next
	}
	n := WGS84A / math.Sqrt(1-WGS84E2*math.Sin(lat)*math.Sin(lat))
	h = p/math.Cos(lat) - n

	const rad2deg = 180.0 / math.Pi
	return model.GeoPoint{
		LatitudeDeg:  lat * rad2deg,
		LongitudeDeg: lon * rad2deg,
		HeightM:      h,
	}
}

// ENUMatrix 返回 ECEF→站心 ENU 的旋转矩阵（3x3，行优先）。
func ENUMatrix(g model.GeoPoint) [3][3]float64 {
	const deg2rad = math.Pi / 180.0
	lat := g.LatitudeDeg * deg2rad
	lon := g.LongitudeDeg * deg2rad
	sinLat, cosLat := math.Sin(lat), math.Cos(lat)
	sinLon, cosLon := math.Sin(lon), math.Cos(lon)

	// 行：E, N, U；列：X, Y, Z。
	return [3][3]float64{
		{-sinLon, cosLon, 0},
		{-sinLat * cosLon, -sinLat * sinLon, cosLat},
		{cosLat * cosLon, cosLat * sinLon, sinLat},
	}
}

// ElevationRad 返回卫星相对接收机位置的仰角（弧度）。
// sat 与 recv 均为 ECEF（米）。
func ElevationRad(sat, recv model.Vec3) float64 {
	if recv.X == 0 && recv.Y == 0 && recv.Z == 0 {
		// 迭代初值位于地心时无法定义站心系，返回天顶方向以获得天顶标称延迟。
		return math.Pi / 2
	}
	geo := ECEFToGeo(recv)
	r := ENUMatrix(geo)
	dx := sat.X - recv.X
	dy := sat.Y - recv.Y
	dz := sat.Z - recv.Z
	east := r[0][0]*dx + r[0][1]*dy + r[0][2]*dz
	north := r[1][0]*dx + r[1][1]*dy + r[1][2]*dz
	up := r[2][0]*dx + r[2][1]*dy + r[2][2]*dz
	horizontal := math.Hypot(east, north)
	return math.Atan2(up, horizontal)
}
