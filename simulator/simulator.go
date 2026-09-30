package simulator

import (
	"math"
	"math/rand"
	"time"

	"github.com/guoyyit/gnss-positioning-go/ephemeris"
	"github.com/guoyyit/gnss-positioning-go/model"
	"github.com/guoyyit/gnss-positioning-go/position"
)

// FaultKind 为故障注入类型。
type FaultKind int

const (
	FaultNone  FaultKind = iota
	FaultStep            // 单星阶跃故障
	FaultMulti           // 多星同时阶跃故障
	FaultDrift           // 缓慢漂移故障
)

// Fault 描述一段历元区间内的故障注入计划。
type Fault struct {
	Kind       FaultKind
	Satellites []model.SatID
	StartEpoch int
	EndEpoch   int
	Magnitude  float64 // 阶跃幅度，米（Step/Multi）
	Rate       float64 // 漂移速率，米/历元（Drift）
}

// Config 为模拟器配置。
type Config struct {
	Seed           int64
	StartTime      time.Time
	Interval       time.Duration
	NumEpochs      int
	NoiseSigma     float64 // 接收机热噪声标准差，米
	MultipathSigma float64 // 多径零均值偏差标准差，米
	TropoDelay     float64 // 对流层延迟标称值，米
	IonoDelay      float64 // 电离层延迟标称值，米
	Faults         []Fault
}

// Simulator 为确定性星座与伪距模拟器。
type Simulator struct {
	cfg         Config
	receiver    model.Vec3
	ephemerides []model.Ephemeris
}

// GenerateEphemerides 生成一个 Walker 样式星座的广播星历集合。
func GenerateEphemerides(t0 time.Time, satellitesPerPlane, planes int, radiusM, inclinationRad float64) []model.Ephemeris {
	total := satellitesPerPlane * planes
	ephs := make([]model.Ephemeris, 0, total)
	for plane := 0; plane < planes; plane++ {
		raan := 2 * math.Pi * float64(plane) / float64(planes)
		for slot := 0; slot < satellitesPerPlane; slot++ {
			phase := 2 * math.Pi * (float64(slot)/float64(satellitesPerPlane) +
				float64(plane)/float64(total))
			phase = math.Mod(phase, 2*math.Pi)

			id := model.SatID(satName(plane, slot))
			ephs = append(ephs, model.Ephemeris{
				Sat:             id,
				Toe:             t0,
				Toc:             t0,
				SemiMajorAxis:   radiusM,
				Eccentricity:    0,
				Inclination:     inclinationRad,
				RAAN:            raan,
				ArgPerigee:      0,
				MeanAnomaly0:    phase,
				DeltaN:          0,
				InclinationRate: 0,
				RAANRate:        0,
				Healthy:         true,
			})
		}
	}
	return ephs
}

func satName(plane, slot int) string {
	return "G" + string(rune('0'+plane)) + string(rune('0'+slot))
}

// New 构造模拟器。
func New(cfg Config, receiver model.Vec3, ephs []model.Ephemeris) *Simulator {
	if cfg.Interval <= 0 {
		cfg.Interval = time.Second
	}
	return &Simulator{cfg: cfg, receiver: receiver, ephemerides: ephs}
}

// Epoch 生成指定序号历元的观测。
func (s *Simulator) Epoch(index int) *model.EpochObs {
	epoch := s.cfg.StartTime.Add(time.Duration(index) * s.cfg.Interval)
	obs := make([]model.Observation, 0, len(s.ephemerides))

	for si := range s.ephemerides {
		eph := s.ephemerides[si]

		initialPos, err := ephemeris.PositionAt(&eph, epoch)
		if err != nil || !eph.Healthy {
			continue
		}
		travel := distance(s.receiver, initialPos) / ephemeris.SpeedOfLight
		tx := epoch.Add(-time.Duration(travel * float64(time.Second)))
		satPos, err := ephemeris.PositionAt(&eph, tx)
		if err != nil {
			continue
		}
		satPos = ephemeris.RotateEarth(satPos, travel)
		geomRange := distance(s.receiver, satPos)

		rng := s.stream(index, si)
		noise := 0.0
		if s.cfg.NoiseSigma > 0 {
			noise = rng.NormFloat64() * s.cfg.NoiseSigma
		}
		multipath := 0.0
		if s.cfg.MultipathSigma > 0 {
			multipath = rng.NormFloat64() * s.cfg.MultipathSigma
		}

		delayModel := position.DelayModel{TropoZenith: s.cfg.TropoDelay, IonoZenith: s.cfg.IonoDelay}
		pr := geomRange - ephemeris.ClockBias(&eph, tx) +
			delayModel.DelayAtSat(satPos, s.receiver) + noise + multipath
		pr += s.faultOffset(eph.Sat, index)

		obs = append(obs, model.Observation{
			Sat:         eph.Sat,
			Pseudorange: pr,
			Ephemeris:   eph,
		})
	}

	return &model.EpochObs{
		Index: index,
		Epoch: epoch,
		Obs:   obs,
	}
}

// Ephemerides 返回模拟器使用的星历副本。
func (s *Simulator) Ephemerides() []model.Ephemeris {
	return s.ephemerides
}

func (s *Simulator) stream(epoch, satIndex int) *rand.Rand {
	seed := s.cfg.Seed ^ int64(epoch)*1000003 ^ int64(satIndex+1)*1000033
	return rand.New(rand.NewSource(seed))
}

func (s *Simulator) faultOffset(sat model.SatID, epoch int) float64 {
	for _, f := range s.cfg.Faults {
		if epoch < f.StartEpoch || epoch > f.EndEpoch {
			continue
		}
		listed := false
		for _, id := range f.Satellites {
			if id == sat {
				listed = true
				break
			}
		}
		if !listed {
			continue
		}
		switch f.Kind {
		case FaultStep, FaultMulti:
			return f.Magnitude
		case FaultDrift:
			return f.Rate * float64(epoch-f.StartEpoch)
		default:
			return 0
		}
	}
	return 0
}

func distance(a, b model.Vec3) float64 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	dz := a.Z - b.Z
	return math.Sqrt(dx*dx + dy*dy + dz*dz)
}
