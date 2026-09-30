package model

import "time"

// SatID 标识一颗卫星。
type SatID string

// Vec3 为地心地固(ECEF)坐标系下的三维向量，单位米。
type Vec3 struct {
	X float64
	Y float64
	Z float64
}

// GeoPoint 为 WGS84 大地坐标，角度单位为度，高程单位为米。
type GeoPoint struct {
	LatitudeDeg  float64
	LongitudeDeg float64
	HeightM      float64
}

// Ephemeris 为某颗卫星在某参考历元的广播星历（开普勒参数，ICD-GPS-200 形式）。
type Ephemeris struct {
	Sat SatID

	// 参考历元（星历与时钟参数）。
	Toe time.Time
	Toc time.Time

	// 轨道尺度与形状。
	SemiMajorAxis float64 // 长半轴 a，米
	Eccentricity  float64
	Inclination   float64 // 弧度
	RAAN          float64 // 升交点赤经，弧度
	ArgPerigee    float64 // 近地点幅角，弧度
	MeanAnomaly0  float64 // Toe 时刻平近点角，弧度
	DeltaN        float64 // 平均运动修正，弧度/秒

	// 长期变化率。
	InclinationRate float64 // 弧度/秒
	RAANRate        float64 // 弧度/秒

	// 时钟多项式（Toc 基准）：bias 秒、drift 秒/秒、漂移率。
	Af0 float64
	Af1 float64
	Af2 float64

	// 调和摄动修正项。
	Cuc float64
	Cus float64
	Crc float64
	Crs float64
	Cic float64
	Cis float64

	// 广播健康标志。
	Healthy bool
}

// Observation 为一颗卫星在一个接收历元的伪距观测及其星历。
type Observation struct {
	Sat         SatID
	Pseudorange float64 // 米
	Ephemeris   Ephemeris
}

// EpochObs 为一个接收历元的全部观测。
type EpochObs struct {
	Index int
	Epoch time.Time // 接收机加盖的时间戳
	Obs   []Observation
}

// SolutionStatus 为定位解总体状态。
type SolutionStatus int

const (
	StatusOK SolutionStatus = iota
	StatusDegraded
	StatusFailed
)

// FailureReason 为定位失败的可区分原因。
type FailureReason int

const (
	FailureNone FailureReason = iota
	FailureInsufficientGeometry
	FailureNoConvergence
	FailureNoUsableSatellites
)

// QualityFlag 为观测链路与解算过程的可区分质量标记。
type QualityFlag int

const (
	FlagNone QualityFlag = iota
	FlagObservationMissing
	FlagEpochMissing
	FlagDuplicateSuppressed
	FlagOutOfOrder
	FlagClockJump
	FlagFaultExcluded
	FlagRAIMUnavailable
)

// RAIMStatus 为完好性监测结论。
type RAIMStatus int

const (
	RAIMOK RAIMStatus = iota
	RAIMFaultDetected
	RAIMFaultExcluded
	RAIMUnavailable
)

// QualityReport 为精度与残差统计。
type QualityReport struct {
	NumSatellites int
	GDOP          float64
	PDOP          float64
	HDOP          float64
	VDOP          float64
	TDOP          float64
	RMSResidual   float64
	MaxResidual   float64
	Sigma0        float64
	Covariance    [4][4]float64 // ECEF X,Y,Z 与接收机钟差(米)
}

// IntegrityReport 为完好性输出。
type IntegrityReport struct {
	Status          RAIMStatus
	FaultSatellites []SatID
	Statistic       float64
	Threshold       float64
	DegreesFreedom  int
	HPL             float64 // 水平保护级，米
	VPL             float64 // 垂直保护级，米
	HAL             float64 // 水平告警限，米
	VAL             float64 // 垂直告警限，米
	DriftDetected   bool
}

// Solution 为一个历元的完整解算结果。
type Solution struct {
	Channel       string
	Index         int
	Epoch         time.Time
	Status        SolutionStatus
	FailureReason FailureReason
	Flags         []QualityFlag

	PositionECEF Vec3
	ClockBiasM   float64
	Geo          GeoPoint

	UsedSats     []SatID
	ExcludedSats []SatID

	Quality   QualityReport
	Integrity IntegrityReport
}
