package pipeline

import (
	"sync"

	"github.com/guoyyit/gnss-positioning-go/model"

	"github.com/guoyyit/gnss-positioning-go/store"
)

// Engine 管理多个并发接收通道。
type Engine struct {
	mu       sync.RWMutex
	channels map[string]*Channel
	store    *store.Store
}

// NewEngine 构造引擎。
func NewEngine(st *store.Store) *Engine {
	return &Engine{channels: map[string]*Channel{}, store: st}
}

// AddChannel 注册一个新通道（若存储中存在历史结果则自动恢复游标）。
func (e *Engine) AddChannel(cfg Config) *Channel {
	ch := NewChannel(cfg, e.store)
	e.mu.Lock()
	e.channels[cfg.ID] = ch
	e.mu.Unlock()
	return ch
}

// GetChannel 返回指定通道。
func (e *Engine) GetChannel(id string) *Channel {
	e.mu.RLock()
	defer e.mu.RUnlock()
	return e.channels[id]
}

// RunPlayback 对一个通道执行确定性回放：投递全部数据包并返回提交结果。
// 重复包不会被重复提交，仅在其投递时被抑制。
func (e *Engine) RunPlayback(id string, deliveries []Delivery) []*model.Solution {
	e.mu.RLock()
	ch := e.channels[id]
	e.mu.RUnlock()
	if ch == nil {
		return nil
	}

	var committed []*model.Solution
	for _, d := range deliveries {
		if !ch.Ingest(d) {
			continue
		}
		committed = append(committed, ch.Drain()...)
	}
	committed = append(committed, ch.Drain()...)
	return committed
}

// Resume 重放某通道尚未提交的历元数据包，返回新提交结果；已提交历元幂等跳过。
func (e *Engine) Resume(id string, deliveries []Delivery) []*model.Solution {
	return e.RunPlayback(id, deliveries)
}

// Channels 返回通道标识列表。
func (e *Engine) Channels() []string {
	e.mu.RLock()
	defer e.mu.RUnlock()
	ids := make([]string, 0, len(e.channels))
	for id := range e.channels {
		ids = append(ids, id)
	}
	return ids
}
