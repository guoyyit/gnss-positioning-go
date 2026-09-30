package gnss

import (
	"bufio"
	"encoding/json"
	"os"
	"path/filepath"
)

// Store persists committed epoch solutions for restart recovery.
type Store struct {
	path    string
	files   map[string]*os.File
	commits map[string]map[int]bool
}

// OpenStore opens or creates the persistence store at path.
func OpenStore(path string) (*Store, error) {
	if err := os.MkdirAll(path, 0o755); err != nil {
		return nil, err
	}
	return &Store{
		path:    path,
		files:   make(map[string]*os.File),
		commits: make(map[string]map[int]bool),
	}, nil
}

// LoadChannel returns previously committed solutions keyed by epoch index.
func (st *Store) LoadChannel(id string) (map[int]Solution, error) {
	committed := make(map[int]bool)
	st.commits[id] = committed
	out := make(map[int]Solution)
	f, err := os.Open(st.channelPath(id))
	if os.IsNotExist(err) {
		return out, nil
	}
	if err != nil {
		return nil, err
	}
	defer f.Close()
	scanner := bufio.NewScanner(f)
	scanner.Buffer(make([]byte, 64*1024), 4*1024*1024)
	for scanner.Scan() {
		var sol Solution
		if err := json.Unmarshal(scanner.Bytes(), &sol); err != nil {
			return nil, err
		}
		out[sol.Epoch] = sol
		committed[sol.Epoch] = true
	}
	return out, scanner.Err()
}

// Commit durably stores one committed solution.
func (st *Store) Commit(id string, sol Solution) error {
	if set, ok := st.commits[id]; ok && set[sol.Epoch] {
		return nil
	}
	f, ok := st.files[id]
	if !ok {
		var err error
		f, err = os.OpenFile(st.channelPath(id), os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0o644)
		if err != nil {
			return err
		}
		st.files[id] = f
	}
	raw, err := json.Marshal(sol)
	if err != nil {
		return err
	}
	if _, err := f.Write(append(raw, '\n')); err != nil {
		return err
	}
	if err := f.Sync(); err != nil {
		return err
	}
	if st.commits[id] == nil {
		st.commits[id] = make(map[int]bool)
	}
	st.commits[id][sol.Epoch] = true
	return nil
}

// Close flushes and closes the store.
func (st *Store) Close() error {
	var firstErr error
	for id, f := range st.files {
		if err := f.Close(); err != nil && firstErr == nil {
			firstErr = err
		}
		delete(st.files, id)
	}
	st.path = ""
	return firstErr
}

// IsCommitted reports whether an epoch for a channel was already committed.
func (st *Store) IsCommitted(id string, epoch int) bool {
	return st.commits[id][epoch]
}

func (st *Store) channelPath(id string) string {
	safe := filepath.Base(id)
	if safe == "." || safe == string(filepath.Separator) || safe == "" {
		safe = "channel"
	}
	return filepath.Join(st.path, "channel-"+safe+".wal")
}
