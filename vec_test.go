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

func TestVecNormalizeNonFinite(t *testing.T) {
	t.Parallel()

	// A NaN length compares false against any threshold, so the guard has to be
	// an accept test: anything not provably usable is rejected. Otherwise
	// Normalize would hand back a NaN vector labelled as a unit direction.
	//
	// An infinite length is the mirror image: it sails through a lower-bound
	// check, and then 1/Inf == 0 scales the vector flat, so Normalize would
	// report the zero vector as a valid unit direction.
	for name, v := range map[string]r3.Vec{
		"nan x":  r3.NewVec(math.NaN(), 0, 0),
		"nan y":  r3.NewVec(1, math.NaN(), 0),
		"nan z":  r3.NewVec(0, 0, math.NaN()),
		"+inf x": r3.NewVec(math.Inf(1), 0, 0),
		"-inf y": r3.NewVec(1, math.Inf(-1), 0),
		"+inf z": r3.NewVec(0, 0, math.Inf(1)),
	} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()

			n, ok := v.Normalize()
			require.False(t, ok)
			require.Equal(t, r3.Vec{}, n)
		})
	}
}

func TestVecNormalizeHugeFinite(t *testing.T) {
	t.Parallel()

	// The components are finite and perfectly representable, but their squares
	// are not: 1e200² overflows to +Inf, so a sum-of-squares length would report
	// +Inf and scaling by 1/Inf would flatten the vector to (0, 0, 0) — a zero
	// vector announced as a unit direction. Len avoids squaring, so this
	// normalizes correctly rather than being rejected.
	v := r3.NewVec(1e200, 1e200, 1e200)
	require.InDelta(t, math.Sqrt(3)*1e200, v.Len(), 1e188, "Len must not overflow")

	n, ok := v.Normalize()
	require.True(t, ok, "a huge but finite vector has a direction")
	c := 1 / math.Sqrt(3)
	vecEqual(t, r3.NewVec(c, c, c), n)
	require.InDelta(t, 1, n.Len(), 1e-12, "the result is a unit vector")
}

func TestVecNormalizeOverflowingLength(t *testing.T) {
	t.Parallel()

	// The last step past TestVecNormalizeHugeFinite: here even math.Hypot cannot
	// save the LENGTH, because the true length (√2·MaxFloat64) is not a float64.
	// The DIRECTION is, though — and the direction is all Normalize was asked for.
	// Rejecting this vector blamed the direction for a defect of the magnitude,
	// and NewFrame then reported "degenerate (zero or collinear) axes" about axes
	// that were neither.
	v := r3.NewVec(math.MaxFloat64, math.MaxFloat64, 0)
	require.True(t, math.IsInf(v.Len(), 1), "the length genuinely is not representable")

	n, ok := v.Normalize()
	require.True(t, ok, "a finite vector has a direction even when its length overflows")
	c := 1 / math.Sqrt(2)
	vecEqual(t, r3.NewVec(c, c, 0), n)
	require.InDelta(t, 1, n.Len(), 1e-12, "the result is a unit vector")

	// The recovery is a division by the largest component, so a lopsided vector
	// comes out right too, not just a diagonal one.
	n, ok = r3.NewVec(math.MaxFloat64, 0, -math.MaxFloat64/2).Normalize()
	require.True(t, ok)
	want, ok := r3.NewVec(1, 0, -0.5).Normalize()
	require.True(t, ok)
	vecEqual(t, want, n)

	// And a frame built on such an axis now succeeds: the axes were never at
	// fault, and ErrDegenerateFrame stays reserved for axes that really are zero
	// or collinear.
	f, err := r3.NewFrame(r3.Vec{}, v, r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	require.True(t, f.IsValid())
	vecEqual(t, r3.NewVec(c, c, 0), f.U())
}
