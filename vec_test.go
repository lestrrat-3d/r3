package r3_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func vecEqual(t *testing.T, want, got r3.Vec) {
	t.Helper()
	// These operations are exact; there is no accumulated drift to absorb, so
	// hold them to a tighter tolerance than composed geometry gets.
	const exact = 1e-12
	require.Truef(t, want.Equal(got, exact), "want %+v, got %+v", want, got)
}

func TestVecEqual(t *testing.T) {
	t.Parallel()

	v := r3.NewVec(1, 2, 3)
	require.True(t, v.Equal(v, 0), "a vector equals itself exactly")
	require.True(t, v.Equal(r3.NewVec(1.0000001, 2, 3), 1e-6), "within tolerance")
	require.False(t, v.Equal(r3.NewVec(1.0001, 2, 3), 1e-6), "outside tolerance")
	// The tolerance is applied per component, not to the distance between them.
	require.False(t, v.Equal(r3.NewVec(1, 2, 3.5), 1e-6), "a single axis is enough to differ")
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

func TestVecNormalizeNaN(t *testing.T) {
	t.Parallel()

	// A NaN length compares false against any threshold, so the guard has to be
	// an accept test: anything not provably long enough is rejected. Otherwise
	// Normalize would hand back a NaN vector labelled as a unit direction.
	for name, v := range map[string]r3.Vec{
		"nan x": r3.NewVec(math.NaN(), 0, 0),
		"nan y": r3.NewVec(1, math.NaN(), 0),
		"nan z": r3.NewVec(0, 0, math.NaN()),
	} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()

			n, ok := v.Normalize()
			require.False(t, ok)
			require.Equal(t, r3.Vec{}, n)
		})
	}
}
