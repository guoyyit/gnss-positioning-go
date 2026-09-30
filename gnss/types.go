package gnss

// Vec3 is a 3-D vector in the Earth-Centered Earth-Fixed frame, in meters.
type Vec3 struct {
	X, Y, Z float64
}

// LLA is geodetic latitude, longitude (radians) and ellipsoidal height (meters).
type LLA struct {
	Lat, Lon, Alt float64
}

// GNSSTime is a monotonic simulation clock: seconds since 1980-01-06T00:00:00Z.
type GNSSTime struct {
	Seconds float64
}

// Ephemeris holds GPS-like broadcast Keplerian elements for one satellite.
type Ephemeris struct {
	PRN      int
	Toe      float64
	Toc      float64
	SqrtA    float64
	Ecc      float64
	I0       float64
	Omega0   float64
	OmegaDot float64
	ArgPeri  float64
	M0       float64
	DeltaN   float64
	IDot     float64
	Cuc      float64
	Cus      float64
	Crc      float64
	Crs      float64
	Cic      float64
	Cis      float64
	Af0      float64
	Af1      float64
	Af2      float64
	TGD      float64
	Healthy  bool
}

// Observation is one pseudorange measurement at a given time.
type Observation struct {
	PRN         int
	Time        GNSSTime
	Pseudorange float64
}

// EpochData bundles observations and broadcast ephemerides for one epoch.
type EpochData struct {
	Epoch int
	Time  GNSSTime
	Obs   []Observation
	Eph   []Ephemeris
}

// FaultSpec describes an injected satellite fault for the simulator.
type FaultSpec struct {
	PRN        int
	StartEpoch int
	EndEpoch   int
	Bias       float64
	Rate       float64
}

// Status is the high-level result state of a positioning epoch.
type Status int

const (
	StatusOK Status = iota
	StatusInsufficientGeometry
	StatusNoConvergence
	StatusMissingEphemeris
	StatusInvalidData
	StatusRAIMDetectionFailure
)

// String returns a human-readable status name.
func (s Status) String() string {
	switch s {
	case StatusOK:
		return "ok"
	case StatusInsufficientGeometry:
		return "insufficient_geometry"
	case StatusNoConvergence:
		return "no_convergence"
	case StatusMissingEphemeris:
		return "missing_ephemeris"
	case StatusInvalidData:
		return "invalid_data"
	case StatusRAIMDetectionFailure:
		return "raim_detection_failure"
	default:
		return "unknown"
	}
}

// QualityFlag marks observation-link or clock anomalies attached to a solution.
type QualityFlag uint32

const (
	FlagNone       QualityFlag = 0
	FlagMissingObs QualityFlag = 1 << iota
	FlagDuplicate
	FlagOutOfOrder
	FlagClockJump
	FlagRAIMExclusion
	FlagRAIMUnavailable
	FlagDegradedGeometry
)

// Quality holds precision and residual statistics for one solution.
type Quality struct {
	N           int
	GDOP        float64
	PDOP        float64
	HDOP        float64
	VDOP        float64
	TDOP        float64
	RMSResidual float64
	MaxResidual float64
	Covariance  [4][4]float64
	SigmaUERE   float64
}

// RAIMResult is the receiver autonomous integrity monitoring outcome.
type RAIMResult struct {
	Available       bool
	Detected        bool
	FaultPRNs       []int
	ExcludedPRNs    []int
	HPL             float64
	VPL             float64
	GlobalStat      float64
	GlobalThreshold float64
	Recovered       bool
}

// Solution is the full output of one positioning epoch.
type Solution struct {
	ChannelID    string
	Epoch        int
	Time         GNSSTime
	Status       Status
	Reason       string
	Position     Vec3
	LLA          LLA
	ClockBias    float64
	UsedPRNs     []int
	ExcludedPRNs []int
	Quality      Quality
	RAIM         RAIMResult
	Flags        QualityFlag
}
