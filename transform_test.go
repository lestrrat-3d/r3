package r3_test

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// tol is the slack allowed when comparing results of floating-point geometry.
const tol = 1e-9

var (
	axisX = r3.NewVec(1, 0, 0)
	axisY = r3.NewVec(0, 1, 0)
	axisZ = r3.NewVec(0, 0, 1)
)

// randVec returns a point in the cube [-10, 10)³.
func randVec(rng *rand.Rand) r3.Vec {
	c := func() float64 { return rng.Float64()*20 - 10 }
	return r3.NewVec(c(), c(), c())
}

// randTransform returns a valid transform drawn from every constructor in turn,
// so a property asserted over it is asserted over the whole type.
func randTransform(t *testing.T, rng *rand.Rand) r3.Transform {
	t.Helper()

	angle := units.Radians(rng.Float64() * 2 * math.Pi)
	switch rng.IntN(5) {
	case 0:
		return r3.Identity()
	case 1:
		tr, err := r3.Translation(randVec(rng))
		require.NoError(t, err)
		return tr
	case 2:
		tr, err := r3.Rotation(randVec(rng), angle)
		require.NoError(t, err)
		return tr
	case 3:
		tr, err := r3.RotationAround(randVec(rng), randVec(rng), angle)
		require.NoError(t, err)
		return tr
	default:
		f, err := r3.NewFrame(randVec(rng), randVec(rng), randVec(rng))
		require.NoError(t, err)
		tr, err := r3.Reflection(f)
		require.NoError(t, err)
		return tr
	}
}

func TestTransformApply(t *testing.T) {
	t.Parallel()

	t.Run("a translation moves a point but not a direction", func(t *testing.T) {
		t.Parallel()

		// The whole reason Apply and ApplyDir are separate methods: a point has
		// a position and a direction does not.
		tr, err := r3.Translation(r3.NewVec(1, 2, 3))
		require.NoError(t, err)
		d := r3.NewVec(0, 0, 1)

		require.True(t, tr.Apply(r3.NewVec(10, 10, 10)).Equal(r3.NewVec(11, 12, 13), tol))
		require.True(t, tr.ApplyDir(d).Equal(d, tol), "a direction must not translate")
	})

	t.Run("round-trip through the inverse", func(t *testing.T) {
		t.Parallel()

		rng := rand.New(rand.NewPCG(1, 2))
		for range 200 {
			tr := randTransform(t, rng)
			p := randVec(rng)
			inv, err := tr.Inverse()
			require.NoError(t, err)
			require.True(t, inv.Apply(tr.Apply(p)).Equal(p, tol))
		}
	})
}

func TestTransformThen(t *testing.T) {
	t.Parallel()

	t.Run("composes in application order", func(t *testing.T) {
		t.Parallel()

		// This is the spec for the ordering choice: a.Then(b) is "a first, then
		// b", the reverse of the matrix notation B·A.
		rng := rand.New(rand.NewPCG(3, 4))
		for range 200 {
			a, b, p := randTransform(t, rng), randTransform(t, rng), randVec(rng)
			ab, err := a.Then(b)
			require.NoError(t, err, "ordinary transforms compose without overflow")
			require.True(t, ab.Apply(p).Equal(b.Apply(a.Apply(p)), tol))
		}
	})

	t.Run("identity is neutral", func(t *testing.T) {
		t.Parallel()

		rng := rand.New(rand.NewPCG(5, 6))
		for range 50 {
			tr := randTransform(t, rng)

			after, err := tr.Then(r3.Identity())
			require.NoError(t, err)
			require.True(t, after.Equal(tr, tol))

			before, err := r3.Identity().Then(tr)
			require.NoError(t, err)
			require.True(t, before.Equal(tr, tol))
		}
	})

	t.Run("orthonormality survives long chains", func(t *testing.T) {
		t.Parallel()

		// Floating-point drift is the failure mode here: a transform that has
		// been composed a hundred times must still be a rigid motion.
		rng := rand.New(rand.NewPCG(7, 8))
		acc := r3.Identity()
		for range 100 {
			next, err := acc.Then(randTransform(t, rng))
			require.NoError(t, err)
			acc = next
			require.True(t, acc.IsValid())
		}
	})

	t.Run("rejects a composition that overflows", func(t *testing.T) {
		t.Parallel()

		// Both operands are valid — a displacement by MaxFloat64 is finite and a
		// perfectly good rigid motion — but their composition is not: the
		// translations add to +Inf. Then used to return that Transform with no way
		// to complain, which is the one hole a constructor-only invariant cannot
		// plug: two things that exist, composing into a thing that must not.
		far, err := r3.Translation(r3.NewVec(math.MaxFloat64, 0, 0))
		require.NoError(t, err)
		require.True(t, far.IsValid())

		tr, err := far.Then(far)
		require.ErrorIs(t, err, r3.ErrNonFinite)
		require.Equal(t, r3.Transform{}, tr)

		// The boundary is real, not a blanket ban on big numbers: half of
		// MaxFloat64 doubles without overflowing, so it still composes.
		half, err := r3.Translation(r3.NewVec(math.MaxFloat64/2, 0, 0))
		require.NoError(t, err)
		ok, err := half.Then(half)
		require.NoError(t, err)
		require.True(t, ok.IsValid())
		require.InDelta(t, math.MaxFloat64, ok.Translation().X, 1e292)
	})

	t.Run("rejects an inverse that overflows", func(t *testing.T) {
		t.Parallel()

		// The inverse translation is −Lᵀ·t: three dot products, each summing three
		// products. Under a rotated basis a huge-but-finite translation therefore
		// overflows a component even though the transform being inverted is itself
		// impeccable. Exactness of the transpose does not make it representable.
		spin, err := r3.Rotation(axisZ, units.Degrees(45))
		require.NoError(t, err)
		huge, err := r3.Translation(r3.NewVec(math.MaxFloat64, math.MaxFloat64, 0))
		require.NoError(t, err)
		tr, err := spin.Then(huge)
		require.NoError(t, err, "a rotation composed with a translation cannot overflow")
		require.True(t, tr.IsValid())

		// Inverting it must sum √2/2·Max + √2/2·Max = √2·Max, which is past
		// MaxFloat64.
		inv, err := tr.Inverse()
		require.ErrorIs(t, err, r3.ErrNonFinite)
		require.Equal(t, r3.Transform{}, inv)
	})

	t.Run("conservatively rejects an inverse whose intermediate sum overflows", func(t *testing.T) {
		t.Parallel()

		// The ACCEPTED limit, pinned so the documented contract is enforced and not
		// merely described. ApplyDir sums ex·x + ey·y + ez·z in a fixed order, so a
		// partial sum can overflow where the total would not: this basis has the row
		// (⅔, ⅔, −⅓), and against a translation of (Max, Max, Max) that is
		// ⅔·Max + ⅔·Max = +Inf before the −⅓·Max that would have brought it back to
		// exactly Max. The true inverse translation, (−Max, −Max, −Max), is
		// perfectly representable; r3 declines to compute it.
		//
		// This is deliberate — ApplyDir runs once per transformed point, and the
		// package will not tax every point transform to serve coordinates 1e289
		// light-years out (see its doc comment). What matters is that the failure is
		// CONSERVATIVE: an error, never a wrong transform. So assert the error.
		const max = math.MaxFloat64
		b := r3.Basis{
			EX: r3.NewVec(2, 2, -1).Scale(1.0 / 3.0),
			EY: r3.NewVec(2, -1, 2).Scale(1.0 / 3.0),
			EZ: r3.NewVec(-1, 2, 2).Scale(1.0 / 3.0),
		}
		tr, err := r3.FromBasis(b, r3.NewVec(max, max, max))
		require.NoError(t, err, "the basis is orthonormal and the translation finite")
		require.True(t, tr.IsValid())

		inv, err := tr.Inverse()
		require.ErrorIs(t, err, r3.ErrNonFinite, "the conservative rejection, not a wrong answer")
		require.Equal(t, r3.Transform{}, inv)
	})

	t.Run("Apply and ApplyDir return non-finite components at MaxFloat64", func(t *testing.T) {
		t.Parallel()

		// The other half of the accepted limit, and the uncomfortable half: Apply and
		// ApplyDir have no error to return, so where Then/Inverse conservatively
		// REJECT, these two hand back the ±Inf. For the (⅔, ⅔, −⅓) row the exact
		// answer is exactly Max — finite and representable — and we return +Inf. That
		// is a wrong answer, not an error, and the docs must not claim otherwise.
		// Pinned here so the claim and the code cannot drift apart.
		const max = math.MaxFloat64
		b := r3.Basis{
			EX: r3.NewVec(2, 2, -1).Scale(1.0 / 3.0),
			EY: r3.NewVec(2, -1, 2).Scale(1.0 / 3.0),
			EZ: r3.NewVec(-1, 2, 2).Scale(1.0 / 3.0),
		}
		tr, err := r3.FromBasis(b, r3.Vec{})
		require.NoError(t, err)

		d := r3.NewVec(max, max, max)
		require.True(t, math.IsInf(tr.ApplyDir(d).X, 1), "the documented overflow, not a silent finite lie")
		require.True(t, math.IsInf(tr.Apply(d).X, 1))

		// And the contrast that makes the limit tolerable: ordinary magnitudes are
		// exact, so nothing a real model contains is affected.
		ord := r3.NewVec(3, 4, 5)
		require.True(t, tr.ApplyDir(ord).Equal(
			b.EX.Scale(3).Add(b.EY.Scale(4)).Add(b.EZ.Scale(5)), 1e-12))
	})

	t.Run("rejects a composition whose linear part drifts out of tolerance", func(t *testing.T) {
		t.Parallel()

		// Orthonormality is judged with slack, and slack composes. A linear part
		// whose EX·EY is 7e-10 is INSIDE the 1e-9 tolerance and IsValid says yes;
		// composed with itself the error doubles to 1.4e-9, which is outside. Then
		// used to check only the translation and hand that back with a nil error: two
		// valid transforms composing into an invalid one.
		//
		// No exported constructor produces such a transform any more — FromBasis, the
		// one that used to, now orthonormalizes what it admits — so the skew is
		// installed directly. The guard still has to hold: rounding accumulates over a
		// long chain, and Then must judge the linear part it PRODUCES rather than
		// trust the ones it was given.
		b := r3.Basis{EX: r3.NewVec(1, 0, 0), EY: r3.NewVec(7e-10, 1, 0), EZ: r3.NewVec(0, 0, 1)}
		a := r3.TransformWithBasis(b, r3.Vec{})
		require.True(t, a.IsValid(), "the transform being composed really is valid")
		require.InDelta(t, 7e-10, a.Basis().EX.Dot(a.Basis().EY), 1e-20)

		c, err := a.Then(a)
		require.ErrorIs(t, err, r3.ErrNotOrthonormal, "the doubled drift is outside the tolerance")
		require.Equal(t, r3.Transform{}, c)
	})

	t.Run("rejects an invalid receiver or argument", func(t *testing.T) {
		t.Parallel()

		// The zero value is public and documented invalid, so a fallible method must
		// not launder it into a nil-error result. No separate receiver check is
		// needed: validating the RESULT catches it, because a zero basis composes —
		// and transposes — to a zero basis, which is not orthonormal.
		inv, err := r3.Transform{}.Inverse()
		require.ErrorIs(t, err, r3.ErrNotOrthonormal)
		require.Equal(t, r3.Transform{}, inv)

		after, err := r3.Transform{}.Then(r3.Identity())
		require.ErrorIs(t, err, r3.ErrNotOrthonormal, "an invalid receiver poisons the composition")
		require.Equal(t, r3.Transform{}, after)

		before, err := r3.Identity().Then(r3.Transform{})
		require.ErrorIs(t, err, r3.ErrNotOrthonormal, "an invalid argument poisons it just as much")
		require.Equal(t, r3.Transform{}, before)
	})

	t.Run("the ordinary inverse still round-trips", func(t *testing.T) {
		t.Parallel()

		// The fallible signature must not have cost the happy path anything.
		tr, err := r3.RotationAround(r3.NewVec(10, 20, 30), axisZ, units.Degrees(90))
		require.NoError(t, err)
		inv, err := tr.Inverse()
		require.NoError(t, err)

		p := r3.NewVec(1, 2, 3)
		require.True(t, inv.Apply(tr.Apply(p)).Equal(p, tol))

		back, err := tr.Then(inv)
		require.NoError(t, err)
		require.True(t, back.Equal(r3.Identity(), tol))
	})
}

func TestRotation(t *testing.T) {
	t.Parallel()

	t.Run("90 degrees about Z maps +X to +Y", func(t *testing.T) {
		t.Parallel()

		tr, err := r3.Rotation(axisZ, units.Degrees(90))
		require.NoError(t, err)
		require.True(t, tr.ApplyDir(axisX).Equal(axisY, tol))
		require.True(t, tr.ApplyDir(axisY).Equal(axisX.Scale(-1), tol))
		require.True(t, tr.ApplyDir(axisZ).Equal(axisZ, tol), "the axis is fixed")
	})

	t.Run("degrees and radians agree", func(t *testing.T) {
		t.Parallel()

		// The unit is carried by the value, not assumed by the callee.
		deg, err := r3.Rotation(axisZ, units.Degrees(90))
		require.NoError(t, err)
		rad, err := r3.Rotation(axisZ, units.Radians(math.Pi/2))
		require.NoError(t, err)
		require.True(t, deg.Equal(rad, tol))
	})

	t.Run("a full turn is the identity", func(t *testing.T) {
		t.Parallel()

		tr, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(360))
		require.NoError(t, err)
		require.True(t, tr.Equal(r3.Identity(), tol))
	})

	t.Run("preserves lengths and angles", func(t *testing.T) {
		t.Parallel()

		rng := rand.New(rand.NewPCG(9, 10))
		for range 100 {
			tr, err := r3.Rotation(randVec(rng), units.Radians(rng.Float64()*2*math.Pi))
			require.NoError(t, err)

			a, b := randVec(rng), randVec(rng)
			ra, rb := tr.ApplyDir(a), tr.ApplyDir(b)
			require.InDelta(t, a.Len(), ra.Len(), tol)
			require.InDelta(t, a.Dot(b), ra.Dot(rb), tol)
		}
	})

	t.Run("is proper, not a reflection", func(t *testing.T) {
		t.Parallel()

		tr, err := r3.Rotation(axisZ, units.Degrees(30))
		require.NoError(t, err)
		require.False(t, tr.IsReflection())
	})

	t.Run("RotationAround fixes its center", func(t *testing.T) {
		t.Parallel()

		center := r3.NewVec(10, 20, 30)
		tr, err := r3.RotationAround(center, axisZ, units.Degrees(90))
		require.NoError(t, err)
		require.True(t, tr.Apply(center).Equal(center, tol))

		// A point one unit along +X from the center swings to +Y of it.
		p := center.Add(axisX)
		require.True(t, tr.Apply(p).Equal(center.Add(axisY), tol))
	})
}

func TestReflection(t *testing.T) {
	t.Parallel()

	// The XY plane through the origin: reflecting across it negates Z.
	xy, err := r3.NewFrame(r3.Vec{}, axisX, axisY)
	require.NoError(t, err)

	t.Run("reverses orientation", func(t *testing.T) {
		t.Parallel()

		tr, err := r3.Reflection(xy)
		require.NoError(t, err)
		require.True(t, tr.IsReflection())
		require.True(t, tr.IsValid(), "a reflection is still an isometry")
		require.True(t, tr.Apply(r3.NewVec(1, 2, 3)).Equal(r3.NewVec(1, 2, -3), tol))
	})

	t.Run("is its own inverse", func(t *testing.T) {
		t.Parallel()

		tr, err := r3.Reflection(xy)
		require.NoError(t, err)
		twice, err := tr.Then(tr)
		require.NoError(t, err)
		require.True(t, twice.Equal(r3.Identity(), tol))
	})

	t.Run("an enormously distant plane whose offset is finite", func(t *testing.T) {
		t.Parallel()

		// The plane through (Max, Max, −Max) with normal (1, 1, 1)/√3. Its offset,
		// 2(origin·n)n, is about ⅔·Max in each component — an ordinary finite
		// number — but origin·n overflows on the way and the doubling overflows
		// again, so Reflection used to reject a perfectly good mirror plane with
		// ErrNonFinite. The dot is now taken in a scaled binade and only the result
		// is scaled back, so what gets rejected is an offset that really is
		// unrepresentable, not one that merely passed near the ceiling.
		const max = math.MaxFloat64
		// u ⊥ (1,1,1) and u × v ∝ (1,1,1), so N is (1, 1, 1)/√3.
		far, err := r3.NewFrame(r3.NewVec(max, max, -max), r3.NewVec(1, -1, 0), r3.NewVec(1, 1, -2))
		require.NoError(t, err)
		require.True(t, far.IsValid())
		require.True(t, far.N().Equal(r3.NewVec(1, 1, 1).Scale(1/math.Sqrt(3)), tol))

		tr, err := r3.Reflection(far)
		require.NoError(t, err)
		require.True(t, tr.IsValid())
		require.True(t, tr.IsReflection())

		want := 2.0 / 3.0 * max
		require.InDelta(t, want, tr.Translation().X, 1e295)
		require.InDelta(t, want, tr.Translation().Y, 1e295)
		require.InDelta(t, want, tr.Translation().Z, 1e295)
	})

	t.Run("a tiny offset survives an enormous origin component", func(t *testing.T) {
		t.Parallel()

		// The other end of the same range, and the one that bites: the plane's
		// origin is (Max, 0, 1e-20) and its normal is exactly +Z, so origin·n is
		// exactly 1e-20 and the offset is 2e-20. Scaling the ORIGIN by its largest
		// component — Max — flushed the 1e-20 to zero, and Reflection then mirrored
		// across the plane z = 0 instead: no error, no infinity, just the wrong
		// plane. The dot is now taken by scaling the PRODUCTS, which a huge X cannot
		// touch, because it is multiplied by n's zero X.
		f, err := r3.NewFrame(r3.NewVec(math.MaxFloat64, 0, 1e-20), axisX, axisY)
		require.NoError(t, err)
		require.True(t, f.N().Equal(r3.NewVec(0, 0, 1), tol))

		tr, err := r3.Reflection(f)
		require.NoError(t, err)
		require.True(t, tr.IsValid())
		require.Equal(t, r3.NewVec(0, 0, 2e-20), tr.Translation())

		// The defining property, which the translation alone does not prove: every
		// point ON the mirror plane is fixed by the reflection. The origin is such a
		// point, and the old code moved its z from 1e-20 to −1e-20.
		require.Equal(t, f.Origin(), tr.Apply(f.Origin()))
		require.Equal(t, f.ToWorldUV(2, 3), tr.Apply(f.ToWorldUV(2, 3)))
		// And a point 1e-20 above the plane lands 1e-20 below it — z = 3e-20 → −1e-20.
		require.InDelta(t, -1e-20, tr.Apply(r3.NewVec(0, 0, 3e-20)).Z, 1e-30)
	})

	t.Run("fixes points on the mirror plane", func(t *testing.T) {
		t.Parallel()

		// An off-origin plane, to catch a reflection that forgets the offset.
		off, err := r3.NewFrame(r3.NewVec(0, 0, 5), axisX, axisY)
		require.NoError(t, err)
		tr, err := r3.Reflection(off)
		require.NoError(t, err)

		onPlane := off.ToWorldUV(3, 4)
		require.True(t, tr.Apply(onPlane).Equal(onPlane, tol))
		// And a point 2 above the plane lands 2 below it.
		require.True(t, tr.Apply(r3.NewVec(0, 0, 7)).Equal(r3.NewVec(0, 0, 3), tol))
	})
}

func TestFromFrame(t *testing.T) {
	t.Parallel()

	t.Run("agrees with Frame.ToWorld", func(t *testing.T) {
		t.Parallel()

		// The two paths into world space must not drift apart.
		rng := rand.New(rand.NewPCG(11, 12))
		for range 100 {
			f, err := r3.NewFrame(randVec(rng), randVec(rng), randVec(rng))
			require.NoError(t, err)
			tr, err := r3.FromFrame(f)
			require.NoError(t, err)

			local := randVec(rng)
			require.True(t, tr.Apply(local).Equal(f.ToWorld(local), tol))
			// And the inverse must agree with ToLocal.
			inv, err := tr.Inverse()
			require.NoError(t, err)
			world := randVec(rng)
			require.True(t, inv.Apply(world).Equal(f.ToLocal(world), tol))
		}
	})

	t.Run("frame-to-frame placement", func(t *testing.T) {
		t.Parallel()

		// The recipe decad actually wants, asserted so nobody re-derives it
		// backwards: express the point in a's coordinates, plant it in b's.
		a, err := r3.NewFrame(r3.NewVec(1, 2, 3), axisX, axisY)
		require.NoError(t, err)
		b, err := r3.NewFrame(r3.NewVec(-4, 0, 6), axisY, axisZ)
		require.NoError(t, err)

		from, err := r3.FromFrame(a)
		require.NoError(t, err)
		to, err := r3.FromFrame(b)
		require.NoError(t, err)
		out, err := from.Inverse()
		require.NoError(t, err)
		place, err := out.Then(to)
		require.NoError(t, err)

		require.True(t, place.Apply(a.Origin()).Equal(b.Origin(), tol))
		require.True(t, place.ApplyDir(a.U()).Equal(b.U(), tol))
		require.True(t, place.ApplyDir(a.V()).Equal(b.V(), tol))
		require.True(t, place.ApplyDir(a.N()).Equal(b.N(), tol))
	})
}

func TestBasisRoundTrip(t *testing.T) {
	t.Parallel()

	// The persistence path: read the numbers out, feed them back in.
	rng := rand.New(rand.NewPCG(13, 14))
	for range 100 {
		tr := randTransform(t, rng)
		back, err := r3.FromBasis(tr.Basis(), tr.Translation())
		require.NoError(t, err)
		require.True(t, back.Equal(tr, tol))
		require.Equal(t, tr.IsReflection(), back.IsReflection(), "handedness must survive")
	}
}

func TestFromBasisOrthonormalizes(t *testing.T) {
	t.Parallel()

	t.Run("a tolerance-level skew is snapped straight", func(t *testing.T) {
		t.Parallel()

		// The defect this exists for. EX·EY is 7e-10 — inside the 1e-9 tolerance, so
		// the basis is ADMITTED — and FromBasis used to store it exactly as it
		// arrived. But the transpose is the exact inverse of a TRULY orthonormal basis
		// and of nothing else, so Inverse of that transform was not exact: the round
		// trip of (10, 10, 0) came back at (10.000000007, …), a drift of ~1e-8, while
		// doc.go and the README both promised "exact — the transpose, never a solve".
		// Now the basis is orthonormalized on the way in.
		b := r3.Basis{EX: r3.NewVec(1, 0, 0), EY: r3.NewVec(7e-10, 1, 0), EZ: r3.NewVec(0, 0, 1)}
		tr, err := r3.FromBasis(b, r3.Vec{})
		require.NoError(t, err, "7e-10 is inside the orthonormality tolerance: admitted, not refused")
		require.True(t, tr.IsValid())

		// Stored orthonormal to machine precision, not merely to the tolerance that
		// let it in. 1e-9 slack would pass on the skewed basis too, and prove nothing.
		got := tr.Basis()
		require.InDelta(t, 0, got.EX.Dot(got.EY), 1e-15)
		require.InDelta(t, 0, got.EY.Dot(got.EZ), 1e-15)
		require.InDelta(t, 0, got.EZ.Dot(got.EX), 1e-15)
		require.InDelta(t, 1, got.EX.Len(), 1e-15)
		require.InDelta(t, 1, got.EY.Len(), 1e-15)
		require.InDelta(t, 1, got.EZ.Len(), 1e-15)

		// And so the inverse round-trips to machine precision. This is the whole
		// point, so the bound is tight: at the old ~1e-8 drift it fails.
		inv, err := tr.Inverse()
		require.NoError(t, err)
		p := r3.NewVec(10, 10, 0)
		back := inv.Apply(tr.Apply(p))
		require.True(t, back.Equal(p, 1e-14), "the inverse must be exact, not merely within tolerance")

		// The corrected basis is still the basis the caller described, to within the
		// skew that was corrected.
		require.True(t, got.EX.Equal(b.EX, 1e-9))
		require.True(t, got.EY.Equal(b.EY, 1e-9))
		require.True(t, got.EZ.Equal(b.EZ, 1e-9))
	})

	t.Run("a genuinely non-orthonormal basis is still refused", func(t *testing.T) {
		t.Parallel()

		// Snapping a tolerance-level skew straight is not a licence to correct a basis
		// that is simply wrong: a scale, a shear or a collapse must come back as an
		// error, not as a silently different transform than the caller asked for.
		// Validation runs FIRST, and only what it admits is orthonormalized.
		for name, b := range map[string]r3.Basis{
			"scaled":     {EX: axisX.Scale(2), EY: axisY, EZ: axisZ},
			"sheared":    {EX: axisX, EY: r3.NewVec(1, 1, 0).Scale(1 / math.Sqrt2), EZ: axisZ},
			"45° skew":   {EX: axisX, EY: r3.NewVec(1, 1, 0), EZ: axisZ},
			"collapsed":  {EX: axisX, EY: axisX, EZ: axisZ},
			"just over":  {EX: axisX, EY: r3.NewVec(1e-8, 1, 0), EZ: axisZ},
			"zero value": {},
		} {
			t.Run(name, func(t *testing.T) {
				t.Parallel()

				tr, err := r3.FromBasis(b, r3.Vec{})
				require.ErrorIs(t, err, r3.ErrNotOrthonormal)
				require.Equal(t, r3.Transform{}, tr)
			})
		}
	})

	t.Run("handedness survives the orthonormalization", func(t *testing.T) {
		t.Parallel()

		// The easiest thing to get wrong. Gram–Schmidt on EX and EY alone, with
		// EZ := EX × EY, is always RIGHT-handed: it would take an improper basis and
		// hand back a proper one — a mirrored body silently un-mirrored — with no
		// error and no way for the caller to notice. All three vectors go through the
		// procedure instead, which preserves the sign of the determinant.
		refl := r3.Basis{EX: axisX, EY: axisY, EZ: axisZ.Scale(-1)}
		tr, err := r3.FromBasis(refl, r3.NewVec(1, 2, 3))
		require.NoError(t, err)
		require.True(t, tr.IsReflection(), "det = −1 in, det = −1 out")
		require.True(t, tr.Basis().EZ.Equal(r3.NewVec(0, 0, -1), 1e-15))

		// A reflection that is ALSO skewed within tolerance: it is corrected AND stays
		// improper, and its inverse (itself, being an involution) is exact.
		skewed := r3.Basis{EX: axisX, EY: r3.NewVec(7e-10, 1, 0), EZ: r3.NewVec(0, 0, -1)}
		sk, err := r3.FromBasis(skewed, r3.Vec{})
		require.NoError(t, err)
		require.True(t, sk.IsReflection())
		require.InDelta(t, 0, sk.Basis().EX.Dot(sk.Basis().EY), 1e-15)

		inv, err := sk.Inverse()
		require.NoError(t, err)
		p := r3.NewVec(10, 10, 0)
		require.True(t, inv.Apply(sk.Apply(p)).Equal(p, 1e-14))

		// And a reflection built by the package survives the persistence round trip,
		// which is what FromBasis is for.
		xy, err := r3.NewFrame(r3.NewVec(0, 0, 5), axisX, axisY)
		require.NoError(t, err)
		mirror, err := r3.Reflection(xy)
		require.NoError(t, err)
		require.True(t, mirror.IsReflection())

		back, err := r3.FromBasis(mirror.Basis(), mirror.Translation())
		require.NoError(t, err)
		require.True(t, back.IsReflection(), "handedness must survive persistence")
		require.True(t, back.Equal(mirror, 1e-15))
	})
}

func TestTransformValidity(t *testing.T) {
	t.Parallel()

	t.Run("the zero value is invalid", func(t *testing.T) {
		t.Parallel()

		require.False(t, r3.Transform{}.IsValid())
	})

	t.Run("every constructor produces a valid transform", func(t *testing.T) {
		t.Parallel()

		rng := rand.New(rand.NewPCG(15, 16))
		for range 100 {
			require.True(t, randTransform(t, rng).IsValid())
		}
	})

	t.Run("a non-finite translation makes a transform invalid", func(t *testing.T) {
		t.Parallel()

		// IsValid used to inspect the linear part alone, so an identity basis
		// paired with a NaN translation reported itself a rigid motion while
		// mapping every point to NaN.
		for name, v := range map[string]r3.Vec{
			"nan":  r3.NewVec(math.NaN(), 0, 0),
			"+inf": r3.NewVec(0, math.Inf(1), 0),
			"-inf": r3.NewVec(0, 0, math.Inf(-1)),
		} {
			t.Run(name, func(t *testing.T) {
				t.Parallel()

				require.False(t, r3.TransformWithTranslation(v).IsValid())
			})
		}

		require.True(t, r3.TransformWithTranslation(r3.NewVec(1, 2, 3)).IsValid())
	})
}

func TestTransformDegenerateInput(t *testing.T) {
	t.Parallel()

	t.Run("Rotation rejects a zero axis", func(t *testing.T) {
		t.Parallel()

		_, err := r3.Rotation(r3.Vec{}, units.Degrees(90))
		require.ErrorIs(t, err, r3.ErrDegenerateAxis)

		_, err = r3.RotationAround(r3.NewVec(1, 1, 1), r3.Vec{}, units.Degrees(90))
		require.ErrorIs(t, err, r3.ErrDegenerateAxis)
	})

	t.Run("Rotation rejects a NaN axis", func(t *testing.T) {
		t.Parallel()

		// A NaN axis has no direction to rotate about; normalizing it would
		// otherwise hand back a NaN "unit" vector and poison the whole transform.
		_, err := r3.Rotation(r3.NewVec(math.NaN(), 0, 0), units.Degrees(90))
		require.ErrorIs(t, err, r3.ErrDegenerateAxis)

		_, err = r3.RotationAround(r3.NewVec(1, 1, 1), r3.NewVec(0, math.NaN(), 0), units.Degrees(90))
		require.ErrorIs(t, err, r3.ErrDegenerateAxis)
	})

	t.Run("Rotation rejects an infinite axis", func(t *testing.T) {
		t.Parallel()

		// An infinite axis is degenerate for the same reason a zero one is: there
		// is no finite direction to rotate about. It used to slip past the length
		// guard, and scaling by 1/Inf then produced a NaN "unit" axis, so Rotation
		// returned a nil error alongside a transform that mapped every point to
		// NaN — a constructor handing back a non-isometry.
		rot, err := r3.Rotation(r3.NewVec(math.Inf(1), 0, 0), units.Degrees(90))
		require.ErrorIs(t, err, r3.ErrDegenerateAxis)
		require.Equal(t, r3.Transform{}, rot)

		_, err = r3.Rotation(r3.NewVec(0, 0, math.Inf(-1)), units.Degrees(90))
		require.ErrorIs(t, err, r3.ErrDegenerateAxis)

		_, err = r3.RotationAround(r3.NewVec(1, 1, 1), r3.NewVec(0, math.Inf(1), 0), units.Degrees(90))
		require.ErrorIs(t, err, r3.ErrDegenerateAxis)
	})

	t.Run("Rotation accepts a huge but finite axis", func(t *testing.T) {
		t.Parallel()

		// The axis is finite, so it has a direction: (1e200, 1e200, 1e200)
		// normalizes like (1, 1, 1) does. A sum-of-squares length would have
		// overflowed to +Inf and scaled the axis flat to (0, 0, 0), yielding a nil
		// error and a broken transform.
		huge, err := r3.Rotation(r3.NewVec(1e200, 1e200, 1e200), units.Degrees(120))
		require.NoError(t, err)
		require.True(t, huge.IsValid())

		unit, err := r3.Rotation(r3.NewVec(1, 1, 1), units.Degrees(120))
		require.NoError(t, err)
		require.True(t, huge.Equal(unit, 1e-12), "magnitude must not change the rotation")
	})

	t.Run("Rotation accepts a tiny but nonzero axis", func(t *testing.T) {
		t.Parallel()

		// (1e-20, 0, 0) IS the +X axis; smallness is not degeneracy. Normalize's
		// zeroLen floor — a divide-by-zero guard sized for vectors of order one —
		// would have called it "zero" and returned ErrDegenerateAxis for an axis
		// with a perfectly good direction. The axis is taken scale-free instead.
		tiny, err := r3.Rotation(r3.NewVec(1e-20, 0, 0), units.Degrees(90))
		require.NoError(t, err)
		require.True(t, tiny.IsValid())

		unit, err := r3.Rotation(r3.NewVec(1, 0, 0), units.Degrees(90))
		require.NoError(t, err)
		require.True(t, tiny.Equal(unit, 1e-12), "magnitude must not change the rotation")

		// All the way down: the smallest denormal still names +X.
		denorm, err := r3.Rotation(r3.NewVec(math.SmallestNonzeroFloat64, 0, 0), units.Degrees(90))
		require.NoError(t, err)
		require.True(t, denorm.Equal(unit, 1e-12))
	})

	t.Run("Rotation rejects a value that is not an angle", func(t *testing.T) {
		t.Parallel()

		_, err := r3.Rotation(axisZ, units.Millimeters(30))
		require.ErrorIs(t, err, units.ErrIncompatible, "a length is not an angle")

		_, err = r3.Rotation(axisZ, units.Scalar(1.5))
		require.ErrorIs(t, err, units.ErrIncompatible, "a bare number is not an angle")
	})

	t.Run("Rotation rejects the zero units.Value", func(t *testing.T) {
		t.Parallel()

		// This case is the whole argument for taking a typed angle: with a bare
		// float64, a forgotten field is a silent 0-radian identity rotation.
		// Here it is an error.
		_, err := r3.Rotation(axisZ, units.Value{})
		require.ErrorIs(t, err, units.ErrIncompatible)
	})

	t.Run("frame constructors reject an invalid frame", func(t *testing.T) {
		t.Parallel()

		_, err := r3.Reflection(r3.Frame{})
		require.ErrorIs(t, err, r3.ErrDegenerateFrame)

		_, err = r3.FromFrame(r3.Frame{})
		require.ErrorIs(t, err, r3.ErrDegenerateFrame)
	})

	t.Run("frame constructors reject a non-finite origin", func(t *testing.T) {
		t.Parallel()

		// NewFrame no longer builds such a frame, so this is the guard behind the
		// guard: FromFrame copies the origin into the translation verbatim, and
		// Reflection multiplies it. Both must refuse. The error names the actual
		// fault — a non-finite position — rather than blaming the axes.
		for name, origin := range map[string]r3.Vec{
			"nan":  r3.NewVec(math.NaN(), 0, 0),
			"+inf": r3.NewVec(0, math.Inf(1), 0),
		} {
			t.Run(name, func(t *testing.T) {
				t.Parallel()

				bad := r3.FrameWithOrigin(origin)
				require.False(t, bad.IsValid(), "a frame anchored nowhere is not a frame")

				tr, err := r3.FromFrame(bad)
				require.ErrorIs(t, err, r3.ErrNonFinite)
				require.Equal(t, r3.Transform{}, tr)

				_, err = r3.Reflection(bad)
				require.ErrorIs(t, err, r3.ErrNonFinite, "a mirror plane anchored nowhere has no reflection")
			})
		}
	})

	t.Run("Rotation rejects a non-finite angle", func(t *testing.T) {
		t.Parallel()

		// The unit KIND was checked, the MAGNITUDE never was: a NaN or infinite
		// number of radians is an angle as far as units is concerned, and it went
		// straight into math.Sincos, which answers NaN. Rotation returned that
		// NaN basis with a nil error beside it.
		for name, angle := range map[string]units.Value{
			"nan":  units.Radians(math.NaN()),
			"+inf": units.Radians(math.Inf(1)),
			"-inf": units.Radians(math.Inf(-1)),
			// Degrees is the same value in another dress — the conversion to
			// radians cannot rescue it.
			"nan degrees": units.Degrees(math.NaN()),
			"inf degrees": units.Degrees(math.Inf(1)),
		} {
			t.Run(name, func(t *testing.T) {
				t.Parallel()

				tr, err := r3.Rotation(axisZ, angle)
				require.ErrorIs(t, err, r3.ErrNonFinite)
				require.Equal(t, r3.Transform{}, tr)

				_, err = r3.RotationAround(r3.NewVec(1, 2, 3), axisZ, angle)
				require.ErrorIs(t, err, r3.ErrNonFinite)
			})
		}
	})

	t.Run("RotationAround rejects a non-finite center", func(t *testing.T) {
		t.Parallel()

		// The axis and the angle are both fine; it is the pivot that is nowhere.
		_, err := r3.RotationAround(r3.NewVec(math.NaN(), 0, 0), axisZ, units.Degrees(90))
		require.ErrorIs(t, err, r3.ErrNonFinite)

		_, err = r3.RotationAround(r3.NewVec(0, math.Inf(1), 0), axisZ, units.Degrees(90))
		require.ErrorIs(t, err, r3.ErrNonFinite)
	})

	t.Run("RotationAround rejects a translation that overflows", func(t *testing.T) {
		t.Parallel()

		// Same shape of bug as Reflection's: every input is finite and valid — a
		// pivot at MaxFloat64, a unit axis, a half turn — but the offset the
		// constructor COMPUTES, center − R·center, is 2·MaxFloat64 for a half turn
		// about Z, which is +Inf. RotationAround used to hand that back with a nil
		// error and an IsValid() of false: a Transform that was not a rigid motion.
		tr, err := r3.RotationAround(r3.NewVec(math.MaxFloat64, 0, 0), axisZ, units.Degrees(180))
		require.ErrorIs(t, err, r3.ErrNonFinite)
		require.Equal(t, r3.Transform{}, tr)

		// An ordinary pivot is untouched by the check.
		center := r3.NewVec(10, 20, 30)
		ok, err := r3.RotationAround(center, axisZ, units.Degrees(180))
		require.NoError(t, err)
		require.True(t, ok.IsValid())
		require.True(t, ok.Apply(center).Equal(center, tol), "the center is fixed")
	})

	t.Run("Translation rejects a non-finite displacement", func(t *testing.T) {
		t.Parallel()

		// Translation used to be infallible, and a NaN displacement produced a
		// Transform whose IsValid said true — the linear part was a pristine
		// identity — while Apply sent every point to NaN.
		for name, v := range map[string]r3.Vec{
			"nan":  r3.NewVec(math.NaN(), 0, 0),
			"+inf": r3.NewVec(0, math.Inf(1), 0),
			"-inf": r3.NewVec(0, 0, math.Inf(-1)),
		} {
			t.Run(name, func(t *testing.T) {
				t.Parallel()

				tr, err := r3.Translation(v)
				require.ErrorIs(t, err, r3.ErrNonFinite)
				require.Equal(t, r3.Transform{}, tr)
			})
		}
	})

	t.Run("Translation accepts an ordinary displacement", func(t *testing.T) {
		t.Parallel()

		// The fallible signature must not have cost the happy path anything.
		tr, err := r3.Translation(r3.NewVec(1, 2, 3))
		require.NoError(t, err)
		require.True(t, tr.IsValid())
		require.True(t, tr.Apply(r3.NewVec(10, 20, 30)).Equal(r3.NewVec(11, 22, 33), tol))
		require.True(t, tr.Translation().Equal(r3.NewVec(1, 2, 3), tol))
	})

	t.Run("FromBasis rejects a non-finite translation", func(t *testing.T) {
		t.Parallel()

		// The basis is impeccable; only the position is not a position. This is
		// the persistence back door: FromBasis validated the linear part alone.
		good := r3.Basis{EX: axisX, EY: axisY, EZ: axisZ}
		for name, v := range map[string]r3.Vec{
			"nan":  r3.NewVec(math.NaN(), 0, 0),
			"+inf": r3.NewVec(0, math.Inf(1), 0),
			"-inf": r3.NewVec(0, 0, math.Inf(-1)),
		} {
			t.Run(name, func(t *testing.T) {
				t.Parallel()

				tr, err := r3.FromBasis(good, v)
				require.ErrorIs(t, err, r3.ErrNonFinite)
				require.Equal(t, r3.Transform{}, tr)
			})
		}
	})

	t.Run("Reflection rejects a translation that overflows", func(t *testing.T) {
		t.Parallel()

		// Every input is individually valid and finite, yet the offset the
		// constructor COMPUTES — 2(origin·n)n — overflows: the plane sits 1e308
		// along its own normal, and 2e308 is past MaxFloat64 (~1.798e308). So the
		// result is what has to be validated, not just the input.
		far, err := r3.NewFrame(r3.NewVec(0, 0, 1e308), axisX, axisY)
		require.NoError(t, err)
		require.True(t, far.IsValid(), "the frame itself is perfectly finite")

		tr, err := r3.Reflection(far)
		require.ErrorIs(t, err, r3.ErrNonFinite)
		require.Equal(t, r3.Transform{}, tr)

		// And the boundary is real, not a blanket ban on big numbers: 1e300 is
		// just as huge and doubles without overflowing, so it still reflects.
		near, err := r3.NewFrame(r3.NewVec(0, 0, 1e300), axisX, axisY)
		require.NoError(t, err)
		ok, err := r3.Reflection(near)
		require.NoError(t, err)
		require.True(t, ok.IsValid())
		require.True(t, ok.Translation().Equal(r3.NewVec(0, 0, 2e300), 1e288))
	})

	t.Run("FromBasis rejects a non-orthonormal basis", func(t *testing.T) {
		t.Parallel()

		nan := math.NaN()
		inf := math.Inf(1)

		// The back door a scale or shear would sneak in through. The non-finite
		// cases matter because every comparison against NaN is false: a guard
		// phrased as a rejection (x > tol) would let them through.
		for name, b := range map[string]r3.Basis{
			"zero":                          {},
			"scaled":                        {EX: axisX.Scale(2), EY: axisY, EZ: axisZ},
			"sheared":                       {EX: axisX, EY: r3.NewVec(1, 1, 0), EZ: axisZ},
			"collapsed":                     {EX: axisX, EY: axisX, EZ: axisZ},
			"nan":                           {EX: axisX, EY: r3.NewVec(nan, 0, 0), EZ: axisZ},
			"inf":                           {EX: axisX, EY: r3.NewVec(inf, 0, 0), EZ: axisZ},
			"nan in an otherwise unit axis": {EX: r3.NewVec(1, nan, 0), EY: axisY, EZ: axisZ},
		} {
			t.Run(name, func(t *testing.T) {
				t.Parallel()

				_, err := r3.FromBasis(b, r3.Vec{})
				require.ErrorIs(t, err, r3.ErrNotOrthonormal)
			})
		}
	})
}
