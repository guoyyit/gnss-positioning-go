package gnss

import (
	"errors"
	"sort"
)

var errInvalidEpoch = errors.New("epoch contains no observations")

// ChannelConfig configures one independent receiver channel.
type ChannelConfig struct {
	ID                 string
	Solver             SolverConfig
	ClockJumpThreshold float64
	HistoryEpochs      int
}

// Channel is an independent receiver processing stream.
type Channel struct {
	cfg          ChannelConfig
	lastEpoch    int
	lastTime     float64
	lastClock    float64
	haveClock    bool
	recent       map[int]Solution
	history      map[int][]float64
	excluded     map[int]exclusionEntry
	committed    map[int]bool
	lastDt       float64
	clockRate    float64
	clockRateSet bool
	clockRefTime float64
	clockRefBias float64
}

type exclusionEntry struct {
	prn      int
	expireAt int
}

// NewChannel creates a receiver channel.
func NewChannel(cfg ChannelConfig) *Channel {
	if cfg.ClockJumpThreshold <= 0 {
		cfg.ClockJumpThreshold = 1000
	}
	if cfg.HistoryEpochs <= 0 {
		cfg.HistoryEpochs = 10
	}
	if cfg.Solver.MinSatellites <= 0 {
		cfg.Solver.MinSatellites = 4
	}
	return &Channel{
		cfg:       cfg,
		recent:    make(map[int]Solution, cfg.HistoryEpochs*2),
		history:   make(map[int][]float64),
		excluded:  make(map[int]exclusionEntry),
		committed: make(map[int]bool),
		lastEpoch: -1,
	}
}

// Handle ingests one epoch and returns its committed solution.
func (ch *Channel) Handle(data EpochData) (Solution, error) {
	if len(data.Obs) == 0 {
		return Solution{Epoch: data.Epoch, Time: data.Time, Status: StatusInvalidData, Reason: errInvalidEpoch.Error()}, errInvalidEpoch
	}
	var flags QualityFlag
	if data.Epoch < ch.lastEpoch || data.Time.Seconds < ch.lastTime {
		flags |= FlagOutOfOrder
	}
	if flags&FlagOutOfOrder == 0 {
		if sol, ok := ch.recent[data.Epoch]; ok {
			return sol, nil
		}
	}

	terms, prepFlags := prepareTerms(data)
	flags |= prepFlags

	sol := Solution{
		ChannelID: ch.cfg.ID,
		Epoch:     data.Epoch,
		Time:      data.Time,
		Flags:     flags,
	}

	ch.expireExclusions(data.Epoch)

	minSats := ch.cfg.Solver.MinSatellites
	healthy := terms
	var preExcluded []int
	if ch.cfg.Solver.RAIM.Enabled && len(ch.excluded) > 0 {
		healthy = make([]satTerm, 0, len(terms))
		exSet := make(map[int]bool)
		for _, term := range terms {
			if _, bad := ch.excluded[term.prn]; bad {
				exSet[term.prn] = true
				continue
			}
			healthy = append(healthy, term)
		}
		for prn := range exSet {
			preExcluded = append(preExcluded, prn)
		}
		sort.Ints(preExcluded)
	}

	if len(healthy) < minSats {
		sol.Status = StatusInsufficientGeometry
		sol.Reason = "fewer than four usable observations after integrity filtering"
		ch.advanceSeen(data)
		return sol, nil
	}

	ls := leastSquares(healthy, ch.cfg.Solver)
	if !ls.rankOK {
		sol.Status = StatusInsufficientGeometry
		sol.Reason = "observation geometry matrix is rank deficient"
		ch.advanceSeen(data)
		return sol, nil
	}
	if !ls.converged {
		sol.Status = StatusNoConvergence
		sol.Reason = "least-squares iteration did not converge"
		ch.advanceSeen(data)
		return sol, nil
	}

	var raim RAIMResult
	if ch.cfg.Solver.RAIM.Enabled {
		in := raimInput{
			terms:    healthy,
			baseline: ls,
			history:  ch.history,
			excluded: map[int]bool{},
			excludePRN: func(prn int) {
				ch.excluded[prn] = exclusionEntry{prn: prn, expireAt: data.Epoch + ch.cfg.HistoryEpochs}
			},
		}
		raim, ls = runRAIM(in, ch.cfg.Solver.RAIM, ch.cfg.Solver)
		if !raim.Available {
			sol.Flags |= FlagRAIMUnavailable
		}
		if len(raim.ExcludedPRNs) > 0 {
			sol.Flags |= FlagRAIMExclusion
		}
		if ls.dop.PDOP > 6 {
			sol.Flags |= FlagDegradedGeometry
		}
	}

	if raim.Detected && !raim.Recovered {
		sol.Status = StatusRAIMDetectionFailure
		sol.Reason = "fault detected but could not be isolated with enough satellites"
		sol.RAIM = raim
		ch.advanceSeen(data)
		return sol, nil
	}

	if ch.haveClock && ch.clockRateSet {
		dt := data.Time.Seconds - ch.lastTime
		if dt <= 0 {
			dt = ch.lastDt
		}
		expected := ch.lastClock
		if ch.clockRateSet {
			expected = ch.clockRefBias + ch.clockRate*(data.Time.Seconds-ch.clockRefTime)
		}
		if abs(ls.clockBias-expected) > ch.cfg.ClockJumpThreshold {
			sol.Flags |= FlagClockJump
		}
	}

	sol.Status = StatusOK
	sol.Position = ls.pos
	sol.LLA = ECEFToLLA(ls.pos)
	sol.ClockBias = ls.clockBias
	sol.UsedPRNs = append([]int(nil), ls.prns...)
	sol.ExcludedPRNs = append(append([]int(nil), preExcluded...), raim.ExcludedPRNs...)
	sort.Ints(sol.ExcludedPRNs)
	sol.Quality = ls.dop
	sol.Quality.RMSResidual = ls.rmsResidual
	sol.Quality.MaxResidual = ls.maxResidual
	sol.Quality.Covariance = ls.cov
	sol.RAIM = raim

	ch.updateHistory(ls)
	ch.lastClock = ls.clockBias
	if dt := data.Time.Seconds - ch.lastTime; dt > 0 {
		ch.lastDt = dt
	}
	if !ch.haveClock {
		ch.clockRefTime = data.Time.Seconds
		ch.clockRefBias = ls.clockBias
	}
	if ch.haveClock && !ch.clockRateSet && ch.lastDt > 0 {
		ch.clockRate = (ls.clockBias - ch.clockRefBias) / (data.Time.Seconds - ch.clockRefTime)
		ch.clockRateSet = true
	}
	ch.haveClock = true
	ch.advanceSeen(data)
	ch.remember(data.Epoch, sol)
	return sol, nil
}

// Close releases bounded resources held by the channel.
func (ch *Channel) Close() error {
	ch.recent = nil
	ch.history = nil
	ch.excluded = nil
	ch.committed = nil
	return nil
}

// MarkCommit records that an epoch result has been durably committed.
func (ch *Channel) MarkCommit(epoch int) {
	ch.committed[epoch] = true
}

// IsCommitted reports whether an epoch result was already committed.
func (ch *Channel) IsCommitted(epoch int) bool {
	return ch.committed[epoch]
}

func (ch *Channel) advanceSeen(data EpochData) {
	if data.Epoch > ch.lastEpoch {
		ch.lastEpoch = data.Epoch
	}
	if data.Time.Seconds > ch.lastTime {
		ch.lastTime = data.Time.Seconds
	}
}

func (ch *Channel) remember(epoch int, sol Solution) {
	if len(ch.recent) >= ch.cfg.HistoryEpochs*4 {
		oldest := epoch
		for e := range ch.recent {
			if e < oldest {
				oldest = e
			}
		}
		delete(ch.recent, oldest)
	}
	ch.recent[epoch] = sol
}

func (ch *Channel) updateHistory(ls lsResult) {
	window := ch.cfg.Solver.RAIM.DriftWindowEpochs
	if window < 2 {
		window = 5
	}
	capacity := window + 2
	for i, prn := range ls.prns {
		series := append(ch.history[prn], ls.residuals[i])
		if len(series) > capacity {
			series = series[len(series)-capacity:]
		}
		ch.history[prn] = series
	}
}

func (ch *Channel) expireExclusions(epoch int) {
	for prn, entry := range ch.excluded {
		if entry.expireAt <= epoch {
			delete(ch.excluded, prn)
		}
	}
}
