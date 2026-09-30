package pipeline

import (
	"path/filepath"
	"reflect"
	"testing"

	"github.com/guoyyit/gnss-positioning-go/model"
	"github.com/guoyyit/gnss-positioning-go/store"
)

func TestBoundedState(t *testing.T) {
	sim, _ := testSimulator(t, 3, nil)
	st, err := store.New(filepath.Join(t.TempDir(), "bounded.gob"))
	if err != nil {
		t.Fatal(err)
	}
	defer st.Close()

	eng := NewEngine(st)
	cfg := baseConfig("ch")
	cfg.ReorderTolerance = 3
	ch := eng.AddChannel(cfg)

	for i := 0; i < 500; i++ {
		ep := sim.Epoch(i % 40)
		ep.Index = i
		ch.Ingest(Delivery{Epoch: ep})
		if i%4 == 0 {
			ch.Drain()
		}
	}
	ch.Drain()

	rv := reflect.ValueOf(ch).Elem()
	pending := rv.FieldByName("pending")
	seen := rv.FieldByName("seen")
	driftResid := rv.FieldByName("drift").Elem().FieldByName("resid")

	// pending/seen 应受 ReorderTolerance 约束；drift 受窗口约束。
	if pending.Len() > 16 {
		t.Fatalf("pending map grew unbounded: %d", pending.Len())
	}
	if seen.Len() > 16 {
		t.Fatalf("seen map grew unbounded: %d", seen.Len())
	}
	if driftResid.Len() > cfg.RAIM.DriftWindow {
		t.Fatalf("drift history unbounded: %d", driftResid.Len())
	}

	if ch.LastCommitted() != 499 {
		t.Fatalf("expected all 500 epochs committed, last=%d", ch.LastCommitted())
	}
}

var _ = model.StatusOK
