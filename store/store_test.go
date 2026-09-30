package store

import (
	"path/filepath"
	"testing"

	"github.com/guoyyit/gnss-positioning-go/model"
)

func TestCommitIdempotentAndRecovery(t *testing.T) {
	path := filepath.Join(t.TempDir(), "state.gob")

	st, err := New(path)
	if err != nil {
		t.Fatal(err)
	}
	sol := func(i int) *model.Solution {
		return &model.Solution{Index: i, Status: model.StatusOK}
	}
	if added, err := st.Commit("a", sol(0)); err != nil || !added {
		t.Fatalf("first commit added=%v err=%v", added, err)
	}
	if added, _ := st.Commit("a", sol(0)); added {
		t.Fatal("duplicate commit must not add")
	}
	if added, _ := st.Commit("a", sol(2)); !added {
		t.Fatal("epoch 2 should add")
	}
	if added, _ := st.Commit("b", sol(0)); !added {
		t.Fatal("separate channel should add")
	}
	if err := st.Close(); err != nil {
		t.Fatal(err)
	}

	// 重启恢复。
	st2, err := New(path)
	if err != nil {
		t.Fatal(err)
	}
	defer st2.Close()
	if last := st2.LastIndex("a"); last != 2 {
		t.Fatalf("expected recovered last index 2, got %d", last)
	}
	if s := st2.Solution("a", 0); s == nil {
		t.Fatal("recovered epoch 0 missing")
	}
	all := st2.All("a")
	if len(all) != 2 || all[0].Index != 0 || all[1].Index != 2 {
		t.Fatalf("recovered solutions out of order: %v", all)
	}
	if last := st2.LastIndex("b"); last != 0 {
		t.Fatalf("channel b isolation broken: last=%d", last)
	}
	// 重启后再次提交已存在历元仍幂等。
	if added, _ := st2.Commit("a", sol(0)); added {
		t.Fatal("commit after restart must remain idempotent")
	}
}
