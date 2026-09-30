package position

import (
	"math"

	"github.com/guoyyit/gnss-positioning-go/model"
)

// DelayModel 为对流层/电离层标称传播延迟模型（仅用于本地模拟，单位米）。
// 延迟随仰角呈 1/sin(el) 放大，天顶方向取标称值。
type DelayModel struct {
	TropoZenith float64 // 天顶方向对流层延迟，米
	IonoZenith  float64 // 天顶方向电离层延迟，米
}

// NominalDelay 返回给定仰角下的总标称延迟（米）。仰角按 5° 截止截断。
func (m DelayModel) NominalDelay(elevationRad float64) float64 {
	const minEl = 5.0 * math.Pi / 180.0
	el := elevationRad
	if el < minEl {
		el = minEl
	}
	scale := 1.0 / math.Sin(el)
	return (m.TropoZenith + m.IonoZenith) * scale
}

// DelayAtSat 由卫星与接收机 ECEF 计算标称延迟。
func (m DelayModel) DelayAtSat(sat, recv model.Vec3) float64 {
	return m.NominalDelay(ElevationRad(sat, recv))
}
