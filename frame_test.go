package r3_test

import (
	"math"
	"math/rand/v2"
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

func TestNewFrameUnrepresentableProjection(t *testing.T) {
	t.Parallel()

	// The case no rescaling of a Gram–Schmidt PROJECTION could ever have saved.
	// u = (1, 1, 0) and v = (Max, Max, SmallestNonzeroFloat64) are finite and plainly
	// not collinear — the perpendicular is (0, 0, 1), carried entirely by that
	// denormal Z. But the projection scalar v·û is √2·MaxFloat64, a number float64
	// DOES NOT HAVE: it overflows to +Inf, and v − û·(+Inf) is garbage. Scaling v down
	// to keep the projection finite only flushes the denormal — the whole of the
	// perpendicular — to zero. The perpendicular is now taken by a double cross
	// product (Lagrange), which never forms that scalar: the huge components cancel
	// inside the cross, where they cancel against each other.
	const max = math.MaxFloat64
	f, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 1, 0), r3.NewVec(max, max, math.SmallestNonzeroFloat64))
	require.NoError(t, err, "finite, non-collinear axes are not degenerate")
	require.True(t, f.IsValid())
	require.True(t, f.U().Equal(r3.NewVec(1, 1, 0).Scale(1/math.Sqrt2), 1e-12))
	require.True(t, f.V().Equal(r3.NewVec(0, 0, 1), 1e-12), "the perpendicular is the denormal Z component")

	// N is the third axis of that same frame, up to nothing at all: U × V.
	require.True(t, f.N().Equal(r3.NewVec(1, -1, 0).Scale(1/math.Sqrt2), 1e-12))
	require.InDelta(t, 0, f.U().Dot(f.V()), 1e-12)

	// Same axes at ordinary magnitudes give the same frame: only the size differed.
	small := mkFrame(t, r3.Vec{}, r3.NewVec(1, 1, 0), r3.NewVec(1, 1, 1))
	require.True(t, f.Equal(small, 1e-12), "magnitude and dynamic range must not change the frame")
}

func TestNewFrameCollinearUnderflowingAxis(t *testing.T) {
	t.Parallel()

	// The silent-garbage case. u = (Max/2, 1e-20, 0) and v = 2·u are LITERALLY
	// collinear — v is u scaled — so they span no plane and there is no frame to
	// build. But collinearity used to be judged against a NORMALIZED u, and
	// normalizing rounds: u.Normalize() is (0.9999999999999999, 0, 0), the 1e-20
	// having underflowed away. Against THAT vector v has a perpendicular, so
	// NewFrame invented a plane out of degenerate input and reported no error:
	// U = X, V = Y, N = Z, IsValid() true. Collinearity is now decided by u × v on
	// the axes AS GIVEN, where it is exactly zero.
	u := r3.NewVec(math.MaxFloat64/2, 1e-20, 0)
	_, err := r3.NewFrame(r3.Vec{}, u, u.Scale(2))
	require.ErrorIs(t, err, r3.ErrDegenerateFrame, "v = 2·u spans no plane, whatever normalizing u rounds away")

	// The same shape at other multiples, other scales, and in the other order. Every
	// u here has a component that underflows to nothing when the axis is normalized,
	// and every multiple is a power of two — exact, so each v is LITERALLY u scaled
	// and not merely nearly so. There is nothing here for a frame to be built from.
	for _, tc := range []struct {
		u  r3.Vec
		ks []float64
	}{
		{r3.NewVec(math.MaxFloat64/2, 1e-20, 0), []float64{1, 2, 0.5, -1, -2, -0.5}},
		{r3.NewVec(1e-20, math.MaxFloat64/2, 0), []float64{1, 2, 0.5, -1, -2}},
		{r3.NewVec(math.MaxFloat64/8, 1e-30, 1e-25), []float64{1, 2, 4, 0.5, -1, -2}},
		{r3.NewVec(0, math.MaxFloat64/4, 1e-25), []float64{1, 2, 0.5, -1, -2}},
		// Denormal scale: the whole axis down where the exponent has run out.
		{r3.NewVec(8*math.SmallestNonzeroFloat64, math.SmallestNonzeroFloat64, 0), []float64{1, 2, 4, -1, -2}},
	} {
		for _, k := range tc.ks {
			v := tc.u.Scale(k)
			_, err := r3.NewFrame(r3.Vec{}, tc.u, v)
			require.ErrorIs(t, err, r3.ErrDegenerateFrame, "u=%v, v=%v·u", tc.u, k)
			// And collinearity does not care which axis is which.
			_, err = r3.NewFrame(r3.Vec{}, v, tc.u)
			require.ErrorIs(t, err, r3.ErrDegenerateFrame, "u=%v·u, v=%v", k, tc.u)
		}
	}

	// The line between the two must stay where it belongs: a REAL angle, however
	// small, is not collinearity, and is still accepted.
	f, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(1, 1e-13, 0))
	require.NoError(t, err, "a tiny angle is an angle")
	require.True(t, f.IsValid())
	require.True(t, f.V().Equal(r3.NewVec(0, 1, 0), 1e-12))
}

func TestNewFrameCollinearAtEveryScale(t *testing.T) {
	t.Parallel()

	// Collinear is collinear: at every ratio, sign, and magnitude float64 has. The
	// ratios that are not powers of two round v off the line by a last bit, which is
	// as collinear as float64 gets and must be refused all the same — crossFiltered's
	// rounding band is what turns that residue back into the zero it stands for.
	// (The ratios are kept where the scaled axis stays finite: an overflowing v is a
	// different rejection, tested elsewhere.)
	const max = math.MaxFloat64
	for _, tc := range []struct {
		u  r3.Vec
		ks []float64
	}{
		{r3.NewVec(1, 0, 0), []float64{1, 2, 1.1, math.Pi, -1, -3.5, 0.25}},
		{r3.NewVec(1, 2, 3), []float64{1, 2, 1.1, math.Pi, -1, -3.5, 0.25}},
		{r3.NewVec(1e-300, -1e-300, 1e-300), []float64{1, 2, 1.1, math.Pi, -1, -3.5, 0.25}},
		{r3.NewVec(max, max, max), []float64{1, 0.5, 0.25, -1, -0.5}},
		{r3.NewVec(max/2, max/4, 0), []float64{1, 1.1, -1.9, 0.5, -0.25}},
		{r3.NewVec(math.SmallestNonzeroFloat64, 2*math.SmallestNonzeroFloat64, 0), []float64{1, 2, 4, -1, -2}},
	} {
		for _, k := range tc.ks {
			_, err := r3.NewFrame(r3.Vec{}, tc.u, tc.u.Scale(k))
			require.ErrorIs(t, err, r3.ErrDegenerateFrame, "u=%v, ratio=%v", tc.u, k)
		}
	}
}

func TestNewFrameULPFloor(t *testing.T) {
	t.Parallel()

	// INTENTIONAL, owner-decided behavior — do not "fix" either direction.
	// The floor is on EVIDENCE, not angle: when the determinant is the
	// cancellation residue of two nearly-equal products, "a real razor-thin
	// plane" and "collinear input rounded by the caller's own arithmetic" are
	// bit-identical — v = 1.1*u stored leaves the same one-ulp residue as an axis
	// deliberately one ulp off — and NewFrame rejects BOTH rather than fabricate
	// a normal out of rounding noise. (An UNCANCELLED determinant builds at any
	// angle, however tiny — see TestNewFrameOverflowingCrossRealPlane's
	// ~1e-328 rad plane.) This test pins both sides of the cancellation boundary
	// so neither direction regresses silently.
	u := r3.NewVec(1, 1, 1)

	// One ULP off: an angle of ~1.3e-16 rad. Indistinguishable from noise — rejected.
	v := r3.NewVec(math.Nextafter(1, 2), 1, 1)
	_, err := r3.NewFrame(r3.Vec{}, u, v)
	require.ErrorIs(t, err, r3.ErrDegenerateFrame, "below the ULP floor is collinear by decree")

	// And the construction-noise twin it cannot be told apart from — also rejected.
	_, err = r3.NewFrame(r3.Vec{}, u, u.Scale(1.1))
	require.ErrorIs(t, err, r3.ErrDegenerateFrame)

	// Three orders above the floor: a REAL angle of 1e-13 rad builds.
	above, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(1, 1e-13, 0))
	require.NoError(t, err, "a certifiable angle must build")
	require.True(t, above.IsValid())
}

func TestNewFrameCollinearOverflowingCross(t *testing.T) {
	t.Parallel()

	// The third sibling of one root cause. u = (Max/2, Max/2, 1e-20) and v = 2·u
	// are LITERALLY collinear — v is u scaled by an exact power of two — so they
	// span no plane. But the raw cross product overflows (Max/2 · Max is past the
	// range), and every earlier rescue rescaled a VECTOR to get back under it:
	// scaling u by its own largest component flushed the 1e-20 — the one component
	// whose products decide collinearity, sitting ~328 decimal orders below the
	// top — before the decision was made, and NewFrame fabricated V = +Z, nil
	// error, IsValid() true out of degenerate input. No common scale can hold
	// components 600+ decimal orders apart, so the decision is now made in
	// (mantissa, exponent) form, where nothing overflows and nothing is flushed.
	const max = math.MaxFloat64
	u := r3.NewVec(max/2, max/2, 1e-20)
	_, err := r3.NewFrame(r3.Vec{}, u, u.Scale(2))
	require.ErrorIs(t, err, r3.ErrDegenerateFrame, "v = 2·u spans no plane, however its cross product overflows")

	// The same shape at other multiples (the non-power-of-two ones round v off the
	// line by a last bit, which is as collinear as float64 gets and must be folded
	// back to zero by the rounding band), with the tiny component in each of the
	// three positions, and with it all the way down at the smallest denormal.
	for _, tc := range []struct {
		u  r3.Vec
		ks []float64
	}{
		{r3.NewVec(max/2, max/2, 1e-20), []float64{2, -2, 1.5, 1e-3}},
		{r3.NewVec(max/2, 1e-20, max/2), []float64{2, -2, 1.5, 1e-3}},
		{r3.NewVec(1e-20, max/2, max/2), []float64{2, -2, 1.5, 1e-3}},
		// The smallest denormal takes only the exact power-of-two multiples: any
		// other multiple rounds SmallestNonzeroFloat64 by a different RELATIVE
		// amount than it rounds the huge components, so the scaled v lands
		// genuinely off u's line and the axes as given really do span a plane —
		// a different case, not this one.
		{r3.NewVec(max/2, max/2, math.SmallestNonzeroFloat64), []float64{2, -2}},
		{r3.NewVec(max/2, math.SmallestNonzeroFloat64, max/2), []float64{2, -2}},
		{r3.NewVec(math.SmallestNonzeroFloat64, max/2, max/2), []float64{2, -2}},
	} {
		for _, k := range tc.ks {
			v := tc.u.Scale(k)
			_, err := r3.NewFrame(r3.Vec{}, tc.u, v)
			require.ErrorIs(t, err, r3.ErrDegenerateFrame, "u=%v, v=%v·u", tc.u, k)
			// Collinearity does not care which operand is which.
			_, err = r3.NewFrame(r3.Vec{}, v, tc.u)
			require.ErrorIs(t, err, r3.ErrDegenerateFrame, "u=%v·u, v=%v", k, tc.u)
		}
	}
}

func TestNewFrameOverflowingCrossRealPlane(t *testing.T) {
	t.Parallel()

	// The cousin that must NOT be refused: flip the sign of the tiny component
	// and the two axes span a real plane — at an angle of some 1e-328 radians,
	// but a real one, carried entirely by components the collinear case above
	// proves are preserved. Refusing this while refusing that would just be the
	// old annihilation wearing the other sign.
	const max = math.MaxFloat64
	u := r3.NewVec(max/2, max/2, 1e-20)
	v := r3.NewVec(max/2, max/2, -1e-20)
	f, err := r3.NewFrame(r3.Vec{}, u, v)
	require.NoError(t, err, "a tiny angle is an angle, at any magnitude")
	require.True(t, f.IsValid())

	// The true normal: u × v = 2·(Max/2)·1e-20 · (−1, 1, 0), so N must come back
	// along (−1, 1, 0)/√2 — a sane direction, not an artifact of the overflow.
	nWant := r3.NewVec(-1, 1, 0).Scale(1 / math.Sqrt2)
	require.True(t, f.N().Equal(nWant, 1e-12), "N=%v", f.N())
	// Perpendicular to both axes' directions (the axes themselves are too large
	// for a meaningful raw dot product).
	d := r3.NewVec(1, 1, 0).Scale(1 / math.Sqrt2)
	require.InDelta(t, 0, f.N().Dot(d), 1e-12, "N must be perpendicular to the plane it claims to be normal to")
	require.True(t, f.U().Equal(d, 1e-12))
	// V carries the side v's tiny component chose: −Z, and V·v > 0.
	require.True(t, f.V().Equal(r3.NewVec(0, 0, -1), 1e-12))
	require.Positive(t, f.V().Dot(v), "V must lie on the same side of u as v did")
}

func TestNewFrameNormalMatchesPlainCross(t *testing.T) {
	t.Parallel()

	// The kernel must not diverge from the plain arithmetic where the plain
	// arithmetic is fine: for ordinary axes — cross product finite and
	// comfortably clear of its own rounding noise — the frame's N, built through
	// the exponent-tracked kernel, must be the direction of the plain u × v.
	rng := rand.New(rand.NewPCG(23, 42))
	for range 2000 {
		u, v := randVec(rng), randVec(rng)
		c := u.Cross(v)
		// Near-collinear pairs are excluded not as a concession but by the test's
		// own terms: there the plain cross is mostly rounding residue, and has no
		// trustworthy direction for the kernel to agree with.
		if c.Len() < 1e-3 {
			continue
		}
		want, ok := c.Normalize()
		require.True(t, ok)
		f, err := r3.NewFrame(r3.Vec{}, u, v)
		require.NoError(t, err, "u=%v, v=%v", u, v)
		require.True(t, f.IsValid(), "u=%v, v=%v", u, v)
		require.True(t, f.N().Equal(want, 1e-9), "u=%v, v=%v: N=%v, plain=%v", u, v, f.N(), want)
	}
}

func TestNewFrameHandedness(t *testing.T) {
	t.Parallel()

	// perp = (u × v) × u is the Gram–Schmidt perpendicular times |u|², a POSITIVE
	// multiple; the OTHER operand order at either cross gives that vector NEGATED.
	// Get it wrong and V flips, which flips N = U × V, and the frame is silently
	// left-handed. Nothing else in the package would notice. So pin the sign at both
	// ends.

	// The datum everyone knows: X, then Y, gives +Z. Not −Z.
	xy := mkFrame(t, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.True(t, xy.N().Equal(r3.NewVec(0, 0, 1), 1e-12), "X × Y is +Z")

	// And V must stay on the side of u that the caller's v was on: orthonormalizing
	// v may swing it round to perpendicular, but never through u to the far side.
	for _, tc := range []struct{ u, v r3.Vec }{
		{r3.NewVec(0, 2, 0), r3.NewVec(3, 3, 0)},
		{r3.NewVec(1, 1, 1), r3.NewVec(1, 0, 0)},
		{r3.NewVec(-2, 5, 1), r3.NewVec(0.5, 0.25, -3)},
		{r3.NewVec(1, 0, 0), r3.NewVec(math.MaxFloat64, 1e-20, 0)},
	} {
		f := mkFrame(t, r3.Vec{}, tc.u, tc.v)
		require.True(t, f.IsValid())
		// Positive, not merely non-zero: a flipped sign would make this negative.
		require.Positive(t, f.V().Dot(tc.v), "V must lie on the same side of u as v did")
		// Right-handed: the frame's own normal agrees with u × v, which is the
		// handedness the caller asked for.
		require.Positive(t, f.N().Dot(tc.u.Cross(tc.v)), "N must agree with u × v")
	}

	// The same two signs over ordinary random axes, which is where a flip would hide
	// — a sign error is invisible in any single frame and shows up only against the
	// input that fixed it. Every one of them must build, and be valid.
	rng := rand.New(rand.NewPCG(17, 18))
	for range 2000 {
		u, v := randVec(rng), randVec(rng)
		// Skip pairs that are near-collinear by chance: the frame is legitimate but
		// the sign of a dot product against a perpendicular that is mostly rounding
		// noise says nothing.
		if u.Cross(v).Len() < 1e-6 {
			continue
		}
		f, err := r3.NewFrame(randVec(rng), u, v)
		require.NoError(t, err, "u=%v, v=%v", u, v)
		require.True(t, f.IsValid(), "u=%v, v=%v", u, v)
		require.Positive(t, f.V().Dot(v), "V flipped: u=%v, v=%v", u, v)
		require.Positive(t, f.N().Dot(u.Cross(v)), "N flipped: u=%v, v=%v", u, v)
		require.InDelta(t, 0, f.U().Dot(f.V()), 1e-12)
		require.InDelta(t, 1, f.N().Len(), 1e-12)
	}
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
