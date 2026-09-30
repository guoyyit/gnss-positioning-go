package gnss

import (
	"math"
	"math/rand"
	"sort"
)

// SimConfig configures the deterministic constellation simulator.
type SimConfig struct {
	Seed           int64
	StartTime      GNSSTime
	EpochStep      float64
	NumSatellites  int
	ReceiverLLA    LLA
	RxClockBias0   float64
	RxClockDrift   float64
	NoiseSigma     float64
	MultipathSigma float64
	DelayBias      map[int]float64
	Faults         []FaultSpec
	MaskElevation  float64
	ClockJumps     map[int]float64
}

// Simulator generates local, deterministic constellation observations.
type Simulator struct {
	cfg       SimConfig
	epoch     int
	rxPos     Vec3
	ephemeris []Ephemeris
	planes    int
}

// NewSimulator creates a simulator from cfg.
func NewSimulator(cfg SimConfig) *Simulator {
	if cfg.NumSatellites < 4 {
		cfg.NumSatellites = 4
	}
	if cfg.EpochStep <= 0 {
		cfg.EpochStep = 30
	}
	if cfg.MaskElevation <= 0 {
		cfg.MaskElevation = 5.0 * math.Pi / 180.0
	}
	s := &Simulator{
		cfg:    cfg,
		rxPos:  LLAToECEF(cfg.ReceiverLLA),
		planes: 6,
	}
	s.ephemeris = s.buildConstellation()
	return s
}

// NextEpoch produces the next simulated epoch.
func (s *Simulator) NextEpoch() EpochData {
	epoch := s.epoch
	s.epoch++
	t := GNSSTime{Seconds: s.cfg.StartTime.Seconds + s.cfg.EpochStep*float64(epoch)}

	rng := rand.New(rand.NewSource(s.cfg.Seed + int64(epoch)*1000003))
	rxClock := s.cfg.RxClockBias0 + s.cfg.RxClockDrift*float64(epoch)*s.cfg.EpochStep
	if jump, ok := s.cfg.ClockJumps[epoch]; ok {
		rxClock += jump
	}

	obs := make([]Observation, 0, len(s.ephemeris))
	prns := make([]int, 0, len(s.ephemeris))
	for i := range s.ephemeris {
		prns = append(prns, s.ephemeris[i].PRN)
	}
	sort.Ints(prns)

	for _, prn := range prns {
		e := s.ephemeris[prn-1]
		satPos := SatellitePosition(e, t)
		if satelliteElevation(satPos, s.cfg.ReceiverLLA, s.rxPos) < s.cfg.MaskElevation {
			continue
		}
		geom := vecNorm(vecSub(satPos, s.rxPos))
		pr := geom + rxClock - SatelliteClockBias(e, t)
		if s.cfg.NoiseSigma > 0 {
			pr += rng.NormFloat64() * s.cfg.NoiseSigma
		}
		if s.cfg.MultipathSigma > 0 {
			pr += rng.NormFloat64() * s.cfg.MultipathSigma
		}
		if bias, ok := s.cfg.DelayBias[prn]; ok {
			pr += bias
		}
		pr += s.faultOffset(prn, epoch)
		obs = append(obs, Observation{PRN: prn, Time: t, Pseudorange: pr})
	}

	ephCopy := make([]Ephemeris, len(s.ephemeris))
	copy(ephCopy, s.ephemeris)
	return EpochData{Epoch: epoch, Time: t, Obs: obs, Eph: ephCopy}
}

// EpochIndex returns the index of the next epoch to generate.
func (s *Simulator) EpochIndex() int {
	return s.epoch
}

// TruePosition returns the simulated receiver ECEF position.
func (s *Simulator) TruePosition() Vec3 {
	return s.rxPos
}

// TrueClockBias returns the simulated receiver clock bias in meters at epoch.
func (s *Simulator) TrueClockBias(epoch int) float64 {
	return s.cfg.RxClockBias0 + s.cfg.RxClockDrift*float64(epoch)*s.cfg.EpochStep
}

func (s *Simulator) buildConstellation() []Ephemeris {
	const orbitRadius = 26560000.0
	const inclination = 55.0 * math.Pi / 180.0
	toe := s.cfg.StartTime.Seconds
	ephem := make([]Ephemeris, 0, s.cfg.NumSatellites)
	for i := 0; i < s.cfg.NumSatellites; i++ {
		plane := i % s.planes
		slot := i / s.planes
		perPlane := (s.cfg.NumSatellites + s.planes - 1) / s.planes
		omega0 := 2.0 * math.Pi * float64(plane) / float64(s.planes)
		m0 := 2.0 * math.Pi * float64(slot) / float64(perPlane)
		ephem = append(ephem, Ephemeris{
			PRN:     i + 1,
			Toe:     toe,
			Toc:     toe,
			SqrtA:   math.Sqrt(orbitRadius),
			I0:      inclination,
			Omega0:  omega0,
			M0:      m0,
			Healthy: true,
		})
	}
	return ephem
}

func (s *Simulator) faultOffset(prn, epoch int) float64 {
	for _, f := range s.cfg.Faults {
		if f.PRN != prn || epoch < f.StartEpoch || epoch > f.EndEpoch {
			continue
		}
		elapsed := float64(epoch-f.StartEpoch) * s.cfg.EpochStep
		return f.Bias + f.Rate*elapsed
	}
	return 0
}

func satelliteElevation(satPos Vec3, ref LLA, refPos Vec3) float64 {
	d := vecSub(satPos, refPos)
	sinLat, cosLat := math.Sin(ref.Lat), math.Cos(ref.Lat)
	sinLon, cosLon := math.Sin(ref.Lon), math.Cos(ref.Lon)
	up := cosLat*cosLon*d.X + cosLat*sinLon*d.Y + sinLat*d.Z
	return math.Asin(up / vecNorm(d))
}
