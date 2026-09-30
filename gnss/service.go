package gnss

import "sync"

// Service runs independent receiver channels with persistence.
type Service struct {
	mu       sync.Mutex
	store    *Store
	channels map[string]*managedChannel
}

type managedChannel struct {
	mu      sync.Mutex
	channel *Channel
}

// NewService creates a positioning service backed by path.
func NewService(persistPath string) (*Service, error) {
	store, err := OpenStore(persistPath)
	if err != nil {
		return nil, err
	}
	return &Service{store: store, channels: make(map[string]*managedChannel)}, nil
}

// GetChannel returns an existing channel or creates a new one, replaying
// previously committed epochs so restarts do not skip or duplicate results.
func (s *Service) GetChannel(cfg ChannelConfig) (*Channel, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.getOrCreateChannelLocked(cfg)
}

func (s *Service) getOrCreateChannelLocked(cfg ChannelConfig) (*Channel, error) {
	if mc, ok := s.channels[cfg.ID]; ok {
		return mc.channel, nil
	}
	history, err := s.store.LoadChannel(cfg.ID)
	if err != nil {
		return nil, err
	}
	ch := NewChannel(cfg)
	for epoch := range history {
		ch.MarkCommit(epoch)
	}
	s.channels[cfg.ID] = &managedChannel{channel: ch}
	return ch, nil
}

// Submit processes an epoch and durably commits its solution exactly once.
// Repeated submissions for the same epoch return the previously committed result.
func (s *Service) Submit(cfg ChannelConfig, data EpochData) (Solution, error) {
	s.mu.Lock()
	mc, ok := s.channels[cfg.ID]
	if !ok {
		ch, err := s.getOrCreateChannelLocked(cfg)
		if err != nil {
			s.mu.Unlock()
			return Solution{}, err
		}
		mc = s.channels[cfg.ID]
		_ = ch
	}
	if s.store.IsCommitted(cfg.ID, data.Epoch) {
		history, err := s.store.LoadChannel(cfg.ID)
		s.mu.Unlock()
		if err != nil {
			return Solution{}, err
		}
		return history[data.Epoch], nil
	}
	ch := mc.channel
	s.mu.Unlock()

	mc.mu.Lock()
	defer mc.mu.Unlock()
	sol, err := ch.Handle(data)
	if err != nil {
		return sol, err
	}

	s.mu.Lock()
	if err := s.store.Commit(cfg.ID, sol); err != nil {
		s.mu.Unlock()
		return sol, err
	}
	ch.MarkCommit(data.Epoch)
	s.mu.Unlock()
	return sol, nil
}

// History returns the committed solutions for a channel keyed by epoch.
func (s *Service) History(id string) (map[int]Solution, error) {
	return s.store.LoadChannel(id)
}

// Close stops all channels and flushes the store.
func (s *Service) Close() error {
	s.mu.Lock()
	defer s.mu.Unlock()
	for _, mc := range s.channels {
		_ = mc.channel.Close()
	}
	s.channels = nil
	return s.store.Close()
}
