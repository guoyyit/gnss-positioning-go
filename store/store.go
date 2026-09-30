package store

import (
	"encoding/gob"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"sync"

	"github.com/guoyyit/gnss-positioning-go/model"
)

// Store 为按通道持久化已提交历元结果的存储。
type Store struct {
	mu     sync.Mutex
	path   string
	state  map[string]*channelState
	closed bool
}

type channelState struct {
	LastIndex int
	Solutions map[int]*model.Solution
}

type snapshot struct {
	Channels map[string]*channelState
}

// New 打开（不存在则创建）一个持久化存储。
func New(path string) (*Store, error) {
	if path == "" {
		return nil, errors.New("store path must not be empty")
	}
	if dir := filepath.Dir(path); dir != "." {
		if err := os.MkdirAll(dir, 0o755); err != nil {
			return nil, fmt.Errorf("create store directory: %w", err)
		}
	}
	s := &Store{path: path, state: map[string]*channelState{}}
	if err := s.Load(); err != nil {
		return nil, err
	}
	return s, nil
}

// Load 从磁盘恢复状态（文件不存在时视为空存储）。
func (s *Store) Load() error {
	s.mu.Lock()
	defer s.mu.Unlock()

	f, err := os.Open(s.path)
	if errors.Is(err, os.ErrNotExist) {
		s.state = map[string]*channelState{}
		return nil
	}
	if err != nil {
		return err
	}
	defer f.Close()

	var snap snapshot
	if err := gob.NewDecoder(f).Decode(&snap); err != nil {
		return fmt.Errorf("decode store: %w", err)
	}
	if snap.Channels == nil {
		snap.Channels = map[string]*channelState{}
	}
	s.state = snap.Channels
	return nil
}

// Commit 幂等提交一个历元结果：已提交的历元不会重复写入。
// 返回 true 表示本次为新提交，false 表示此前已存在。
func (s *Store) Commit(channel string, sol *model.Solution) (bool, error) {
	if sol == nil {
		return false, errors.New("nil solution")
	}
	s.mu.Lock()
	defer s.mu.Unlock()

	st := s.state[channel]
	if st == nil {
		st = &channelState{LastIndex: -1, Solutions: map[int]*model.Solution{}}
		s.state[channel] = st
	}
	if _, exists := st.Solutions[sol.Index]; exists {
		return false, nil
	}

	sol.Channel = channel
	st.Solutions[sol.Index] = sol
	if sol.Index > st.LastIndex {
		st.LastIndex = sol.Index
	}
	return true, s.flushLocked()
}

// LastIndex 返回某通道最近已提交历元序号；无记录返回 -1。
func (s *Store) LastIndex(channel string) int {
	s.mu.Lock()
	defer s.mu.Unlock()
	st := s.state[channel]
	if st == nil {
		return -1
	}
	return st.LastIndex
}

// Solution 返回某通道指定历元的已提交结果。
func (s *Store) Solution(channel string, index int) *model.Solution {
	s.mu.Lock()
	defer s.mu.Unlock()
	if st := s.state[channel]; st != nil {
		return st.Solutions[index]
	}
	return nil
}

// All 返回某通道全部已提交结果（序号升序）。
func (s *Store) All(channel string) []*model.Solution {
	s.mu.Lock()
	defer s.mu.Unlock()
	st := s.state[channel]
	if st == nil {
		return nil
	}
	indices := make([]int, 0, len(st.Solutions))
	for idx := range st.Solutions {
		indices = append(indices, idx)
	}
	sort.Ints(indices)
	out := make([]*model.Solution, 0, len(indices))
	for _, idx := range indices {
		out = append(out, st.Solutions[idx])
	}
	return out
}

// Close 刷盘并关闭。
func (s *Store) Close() error {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed {
		return nil
	}
	s.closed = true
	return s.flushLocked()
}

// flushLocked 原子写入（先写临时文件再改名）。
func (s *Store) flushLocked() error {
	if s.path == "" {
		return nil
	}
	tmp := s.path + ".tmp"
	f, err := os.Create(tmp)
	if err != nil {
		return err
	}
	enc := gob.NewEncoder(f)
	if err := enc.Encode(snapshot{Channels: s.state}); err != nil {
		f.Close()
		return err
	}
	if err := f.Sync(); err != nil {
		f.Close()
		return err
	}
	if err := f.Close(); err != nil {
		return err
	}
	return os.Rename(tmp, s.path)
}
