package r3_test

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func vecEqual(t *testing.T, want, got r3.Vec) {
	t.Helper()
	const tol = 1e-12
	require.InDelta(t, want.X, got.X, tol)
	require.InDelta(t, want.Y, got.Y, tol)
	require.InDelta(t, want.Z, got.Z, tol)
}

func TestVecOps(t *testing.T) {
	a := r3.NewVec(1, 2, 3)
	b := r3.NewVec(4, 5, 6)
	vecEqual(t, r3.NewVec(5, 7, 9), a.Add(b))
	vecEqual(t, r3.NewVec(-3, -3, -3), a.Sub(b))
	vecEqual(t, r3.NewVec(2, 4, 6), a.Scale(2))
	require.InDelta(t, 32, a.Dot(b), 1e-12)
	// x × y = z (right-handed).
	vecEqual(t, r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0).Cross(r3.NewVec(0, 1, 0)))

	zero := r3.Vec{}
	if _, ok := zero.Normalize(); ok {
		t.Fatal("normalizing the zero vector must report false")
	}
	u, ok := r3.NewVec(0, 3, 0).Normalize()
	require.True(t, ok)
	vecEqual(t, r3.NewVec(0, 1, 0), u)
}
