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

func TestNewFrameHugeAxes(t *testing.T) {
	t.Parallel()

	// Two finite axes, plainly NOT collinear: (1, 1, 1) and (1, 1, −1) in
	// direction, merely written at the top of the float64 range. The Gram–Schmidt
	// projection v·û used to overflow on the way to a result that cancels back
	// down — ⅔·Max + ⅔·Max is +Inf before the −⅓·Max that would have brought it
	// home — so the perpendicular came out NaN, Normalize refused it, and NewFrame
	// reported "zero or collinear axes" about axes that are neither. Magnitude is
	// not degeneracy.
	const max = math.MaxFloat64
	f, err := r3.NewFrame(r3.Vec{}, r3.NewVec(max, max, max), r3.NewVec(max, max, -max))
	require.NoError(t, err)
	require.True(t, f.IsValid())
	require.InDelta(t, 1, f.U().Len(), 1e-12)
	require.InDelta(t, 1, f.V().Len(), 1e-12)
	require.InDelta(t, 0, f.U().Dot(f.V()), 1e-12, "the axes must come back orthogonal")
	require.InDelta(t, 1, f.N().Len(), 1e-12)

	// Right-handed, as every Frame is: its transform is proper, not a reflection.
	tr, err := r3.FromFrame(f)
	require.NoError(t, err)
	require.False(t, tr.IsReflection())

	// And it is the very frame the same directions give at ordinary magnitudes:
	// only the size differed.
	small := mkFrame(t, r3.Vec{}, r3.NewVec(1, 1, 1), r3.NewVec(1, 1, -1))
	require.True(t, f.Equal(small, 1e-12), "magnitude must not change the frame")
}

func TestNewFrameDegenerate(t *testing.T) {
	_, err := r3.NewFrame(r3.Vec{}, r3.Vec{}, r3.NewVec(0, 1, 0))
	require.ErrorIs(t, err, r3.ErrDegenerateFrame)
	// Collinear axes: v parallel to u leaves no perpendicular component.
	_, err = r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(2, 0, 0))
	require.ErrorIs(t, err, r3.ErrDegenerateFrame)

	// A zero v is degenerate however large u is.
	_, err = r3.NewFrame(r3.Vec{}, r3.NewVec(math.MaxFloat64, math.MaxFloat64, 0), r3.Vec{})
	require.ErrorIs(t, err, r3.ErrDegenerateFrame)

	// Collinear axes stay degenerate at the top of the range too: making the
	// projection overflow-safe must not launder a genuinely vanishing
	// perpendicular into a direction.
	_, err = r3.NewFrame(
		r3.Vec{},
		r3.NewVec(math.MaxFloat64, math.MaxFloat64, math.MaxFloat64),
		r3.NewVec(math.MaxFloat64/2, math.MaxFloat64/2, math.MaxFloat64/2),
	)
	require.ErrorIs(t, err, r3.ErrDegenerateFrame, "collinear is collinear at any magnitude")
}

func TestNewFrameExtremeDynamicRangeAxis(t *testing.T) {
	t.Parallel()

	// u = X, v = (Max, 1e-20, 0): finite, and NOT collinear with u — its
	// perpendicular component is the 1e-20 along Y. Scaling v by its own largest
	// component (Max) to keep the projection from overflowing annihilated that
	// 1e-20, the perpendicular came out zero, and NewFrame reported "zero or
	// collinear axes" about axes that are neither. Dynamic range is not degeneracy
	// any more than magnitude is.
	f, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(math.MaxFloat64, 1e-20, 0))
	require.NoError(t, err)
	require.True(t, f.IsValid())
	require.True(t, f.U().Equal(r3.NewVec(1, 0, 0), 1e-12))
	require.True(t, f.V().Equal(r3.NewVec(0, 1, 0), 1e-12), "the perpendicular is the tiny Y component")
	require.True(t, f.N().Equal(r3.NewVec(0, 0, 1), 1e-12))
	require.InDelta(t, 0, f.U().Dot(f.V()), 1e-12)
}

func TestNewFrameDenormalPerpendicular(t *testing.T) {
	t.Parallel()

	// The far end of the dynamic range, and the same class of bug as the 1e-20 case
	// above: v = (Max, SmallestNonzeroFloat64, 0) against u = X is finite and plainly
	// NOT collinear — its perpendicular component is that denormal along Y. But v was
	// quartered UNCONDITIONALLY before the projection was subtracted off, to buy
	// overflow headroom, and quartering the smallest denormal there is underflows it
	// to zero. The perpendicular vanished and NewFrame reported "zero or collinear
	// axes". The headroom is now bought only when the unscaled subtraction actually
	// overflows.
	f, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(math.MaxFloat64, math.SmallestNonzeroFloat64, 0))
	require.NoError(t, err, "a denormal perpendicular is still a perpendicular")
	require.True(t, f.IsValid())
	require.True(t, f.U().Equal(r3.NewVec(1, 0, 0), 1e-12))
	require.True(t, f.V().Equal(r3.NewVec(0, 1, 0), 1e-12), "the perpendicular is the denormal Y component")
	require.True(t, f.N().Equal(r3.NewVec(0, 0, 1), 1e-12))

	// And the case the quartering was introduced FOR must still work: the true
	// perpendicular of these two wants a component past MaxFloat64, so the unscaled
	// subtraction overflows and the quartered path takes over.
	const max = math.MaxFloat64
	g, err := r3.NewFrame(r3.Vec{}, r3.NewVec(max, max, max), r3.NewVec(max, max, -max))
	require.NoError(t, err, "the overflow path must still be there when it is needed")
	require.True(t, g.IsValid())
	require.InDelta(t, 0, g.U().Dot(g.V()), 1e-12)

	// Both extremes, not one at the cost of the other.
	small := mkFrame(t, r3.Vec{}, r3.NewVec(1, 1, 1), r3.NewVec(1, 1, -1))
	require.True(t, g.Equal(small, 1e-12), "magnitude must not change the frame")
}

func TestNewFrameNonFinite(t *testing.T) {
	// Every comparison against NaN is false, so a guard phrased as a rejection
	// would admit these; the frame that came back would not be orthonormal.
	nan := math.NaN()

	_, err := r3.NewFrame(r3.Vec{}, r3.NewVec(nan, 0, 0), r3.NewVec(0, 1, 0))
	require.ErrorIs(t, err, r3.ErrDegenerateFrame)

	_, err = r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, nan, 0))
	require.ErrorIs(t, err, r3.ErrDegenerateFrame)

	// An infinite axis has infinite length, which is not a usable direction:
	// Normalize rejects it outright rather than dividing through by +Inf.
	_, err = r3.NewFrame(r3.Vec{}, r3.NewVec(math.Inf(1), 0, 0), r3.NewVec(0, 1, 0))
	require.ErrorIs(t, err, r3.ErrDegenerateFrame, "an infinite u axis has no direction")

	_, err = r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, math.Inf(-1), 0))
	require.ErrorIs(t, err, r3.ErrDegenerateFrame, "an infinite v axis has no direction")
}

func TestNewFrameNonFiniteOrigin(t *testing.T) {
	// The axes are validated by Normalize; the origin never was — it is stored
	// verbatim. NewFrame therefore used to hand back a frame whose IsValid said
	// true and whose ToWorld mapped every local coordinate to NaN.
	//
	// The error is ErrNonFinite and NOT ErrDegenerateFrame: the axes here are
	// perfectly good. ErrDegenerateFrame means what it says — zero or collinear
	// AXES — and keeping the two apart is the point.
	for name, origin := range map[string]r3.Vec{
		"nan":  r3.NewVec(math.NaN(), 0, 0),
		"+inf": r3.NewVec(0, math.Inf(1), 0),
		"-inf": r3.NewVec(0, 0, math.Inf(-1)),
	} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()

			f, err := r3.NewFrame(origin, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
			require.ErrorIs(t, err, r3.ErrNonFinite)
			require.Equal(t, r3.Frame{}, f)
		})
	}

	// A huge but finite origin is a position like any other, and is accepted.
	f, err := r3.NewFrame(r3.NewVec(1e300, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	require.True(t, f.IsValid())
}

func TestZeroFrameInvalid(t *testing.T) {
	require.False(t, r3.Frame{}.IsValid())
}

func TestFrameMappingOverflowsAtMaxFloat64(t *testing.T) {
	t.Parallel()

	// The accepted per-point limit, which Frame shares with Transform.ApplyDir —
	// the docs must not pretend it is unique to Transform. ToWorld and ToLocal sum
	// their terms in fixed order and are infallible, so a coordinate out at
	// MaxFloat64 can drive an intermediate sum to ±Inf and be returned as-is: a
	// wrong answer, not an error. Pinned so the claim and the code cannot drift.
	const max = math.MaxFloat64
	f := mkFrame(t, r3.Vec{}, r3.NewVec(2, 2, -1).Scale(1.0/3.0), r3.NewVec(2, -1, 2).Scale(1.0/3.0))
	require.True(t, f.IsValid())

	w := f.ToWorld(r3.NewVec(max, max, max))
	require.True(t, math.IsInf(w.X, 1), "the documented overflow, not a silent finite lie")

	l := f.ToLocal(r3.NewVec(max, max, max))
	require.True(t, math.IsInf(l.X, 1) || math.IsInf(l.Z, -1), "same artefact, through the dot products")

	// And the contrast that makes the limit tolerable: at any magnitude a real
	// model contains, the mapping is exact and the round-trip holds.
	p := r3.NewVec(3, -4, 5)
	require.True(t, f.ToWorld(f.ToLocal(p)).Equal(p, 1e-12))
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
