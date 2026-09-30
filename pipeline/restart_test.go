package pipeline

import (
	"path/filepath"
	"testing"

	"github.com/guoyyit/gnss-positioning-go/store"
)

// 模拟首次运行只处理到历元 9 就中断，重启后重放完整序列：
// 0..9 幂等跳过，10..19 按既定策略重算并补提交，无重复无跳过。
func TestRestartRecomputesPending(t *testing.T) {
	dbPath := filepath.Join(t.TempDir(), "restart2.gob")

	st1, err := store.New(dbPath)
	if err != nil {
		t.Fatal(err)
	}
	eng1 := NewEngine(st1)
	ch1 := eng1.AddChannel(baseConfig("ch"))
	sim1, _ := testSimulator(t, 55, nil)
	for i := 0; i < 10; i++ {
		ch1.Ingest(Delivery{Epoch: sim1.Epoch(i)})
	}
	first := ch1.Drain()
	if len(first) != 10 {
		t.Fatalf("expected 10 first commits, got %d", len(first))
	}
	if err := st1.Close(); err != nil {
		t.Fatal(err)
	}

	// 重启，重放完整 0..19。
	sim2, _ := testSimulator(t, 55, nil)
	st2, err := store.New(dbPath)
	if err != nil {
		t.Fatal(err)
	}
	defer st2.Close()
	eng2 := NewEngine(st2)
	ch2 := eng2.AddChannel(baseConfig("ch"))
	var deliveries []Delivery
	for i := 0; i < 20; i++ {
		deliveries = append(deliveries, Delivery{Epoch: sim2.Epoch(i)})
	}
	added := eng2.RunPlayback("ch", deliveries)
	if len(added) != 10 {
		t.Fatalf("expected 10 recomputed commits, got %d", len(added))
	}
	for i, s := range added {
		if s.Index != 10+i {
			t.Fatalf("unexpected recomputed index %d at %d", s.Index, i)
		}
	}
	if got := len(st2.All("ch")); got != 20 {
		t.Fatalf("expected 20 total solutions, got %d", got)
	}
	if ch2.LastCommitted() != 19 {
		t.Fatalf("expected last committed 19, got %d", ch2.LastCommitted())
	}
}
