package r3_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func mkFrame(t *testing.T, origin, u, v r3.Vec) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(origin, u, v)
	require.NoError(t, err)
	return f
}

func TestNewFrameOrthonormalizes(t *testing.T) {
	// Skewed, non-unit axes must come back orthonormal and right-handed.
	f, err := r3.NewFrame(r3.NewVec(1, 1, 1), r3.NewVec(0, 2, 0), r3.NewVec(3, 3, 0))
	require.NoError(t, err)
	require.True(t, f.IsValid())
	require.InDelta(t, 1, f.U().Len(), 1e-12)
	require.InDelta(t, 1, f.V().Len(), 1e-12)
	require.InDelta(t, 0, f.U().Dot(f.V()), 1e-12)
	// N == U × V and is unit.
	vecEqual(t, f.U().Cross(f.V()), f.N())
	require.InDelta(t, 1, f.N().Len(), 1e-12)
}

func TestNewFrameDegenerate(t *testing.T) {
	_, err := r3.NewFrame(r3.Vec{}, r3.Vec{}, r3.NewVec(0, 1, 0))
	require.ErrorIs(t, err, r3.ErrDegenerateFrame)
	// Collinear axes: v parallel to u leaves no perpendicular component.
	_, err = r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(2, 0, 0))
	require.ErrorIs(t, err, r3.ErrDegenerateFrame)
}

func TestNewFrameNonFinite(t *testing.T) {
	// Every comparison against NaN is false, so a guard phrased as a rejection
	// would admit these; the frame that came back would not be orthonormal.
	nan := math.NaN()

	_, err := r3.NewFrame(r3.Vec{}, r3.NewVec(nan, 0, 0), r3.NewVec(0, 1, 0))
	require.ErrorIs(t, err, r3.ErrDegenerateFrame)

	_, err = r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, nan, 0))
	require.ErrorIs(t, err, r3.ErrDegenerateFrame)

	_, err = r3.NewFrame(r3.Vec{}, r3.NewVec(math.Inf(1), 0, 0), r3.NewVec(0, 1, 0))
	require.ErrorIs(t, err, r3.ErrDegenerateFrame, "an infinite axis normalizes to NaN")
}

func TestZeroFrameInvalid(t *testing.T) {
	require.False(t, r3.Frame{}.IsValid())
}

func TestFrameEqual(t *testing.T) {
	t.Parallel()

	f := mkFrame(t, r3.NewVec(1, 2, 3), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.True(t, f.Equal(f, 0))

	// A frame that differs only in origin, and one only in orientation.
	moved := mkFrame(t, r3.NewVec(1, 2, 4), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	turned := mkFrame(t, r3.NewVec(1, 2, 3), r3.NewVec(0, 1, 0), r3.NewVec(-1, 0, 0))
	require.False(t, f.Equal(moved, 1e-9))
	require.False(t, f.Equal(turned, 1e-9))
}

func TestFrameRoundTrip(t *testing.T) {
	f := mkFrame(t, r3.NewVec(10, -5, 2), r3.NewVec(1, 1, 0), r3.NewVec(-1, 1, 0))
	for _, w := range []r3.Vec{
		r3.NewVec(0, 0, 0),
		r3.NewVec(3, 4, 5),
		r3.NewVec(-7, 2, 9),
	} {
		vecEqual(t, w, f.ToWorld(f.ToLocal(w)))
	}
}

func TestKnownMapsXZ(t *testing.T) {
	// The XZ datum: U = +X, V = +Z, N = −Y.
	xz := mkFrame(t, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1))
	vecEqual(t, r3.NewVec(1, 0, 0), xz.ToWorldUV(1, 0))
	vecEqual(t, r3.NewVec(0, 0, 1), xz.ToWorldUV(0, 1))
	vecEqual(t, r3.NewVec(0, -1, 0), xz.N())
}

func TestOffsetAlongNormal(t *testing.T) {
	// Shifting the XY origin along +N (=+Z) moves world z, leaving x, y.
	xy := mkFrame(t, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	d := 7.5
	shifted := mkFrame(t, xy.Origin().Add(xy.N().Scale(d)), xy.U(), xy.V())
	w := shifted.ToWorldUV(3, 4)
	vecEqual(t, r3.NewVec(3, 4, d), w)
	require.InDelta(t, math.Abs(d), math.Abs(w.Z), 1e-12)
}
