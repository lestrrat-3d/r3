package r3_test

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

const screwTol = 1e-12

func mustT(t *testing.T) func(r3.Transform, error) r3.Transform {
	t.Helper()
	return func(tr r3.Transform, err error) r3.Transform {
		t.Helper()
		require.NoError(t, err)
		return tr
	}
}

func screwRadians(t *testing.T, s r3.Screw) float64 {
	t.Helper()
	rad, err := s.Angle.In(units.Radian)
	require.NoError(t, err)
	return rad
}

// randProper returns a random proper transform: a randTransform without the
// reflection case, optionally composed with a second one.
func randProper(t *testing.T, rng *rand.Rand) r3.Transform {
	t.Helper()
	for {
		tr := randTransform(t, rng)
		if rng.IntN(2) == 0 {
			tr = mustT(t)(tr.Then(randTransform(t, rng)))
		}
		if !tr.IsReflection() {
			return tr
		}
	}
}

func unit(v r3.Vec) r3.Vec {
	n, _ := v.Normalize()
	return n
}

func TestTransformScrew(t *testing.T) {
	t.Parallel()
	must := mustT(t)

	// rebuild checks that sc.At(1) reproduces tr within tolerance.
	rebuild := func(t *testing.T, tr r3.Transform, tolerance float64) r3.Screw {
		t.Helper()
		sc, err := tr.Screw()
		require.NoError(t, err)
		back, err := sc.At(1)
		require.NoError(t, err)
		require.True(t, back.Equal(tr, tolerance), "rebuild %v != %v", back, tr)
		return sc
	}

	t.Run("identity", func(t *testing.T) {
		t.Parallel()
		sc, err := r3.Identity().Screw()
		require.NoError(t, err)
		require.Equal(t, r3.NewVec(0, 0, 1), sc.Axis)
		require.Equal(t, r3.Vec{}, sc.Point)
		require.Equal(t, 0.0, screwRadians(t, sc))
		require.Equal(t, 0.0, sc.Slide)
	})

	t.Run("a pure translation", func(t *testing.T) {
		t.Parallel()
		tr := must(r3.Translation(r3.NewVec(1, 2, 3)))
		sc := rebuild(t, tr, screwTol)
		require.InDelta(t, 0, screwRadians(t, sc), 0)
		require.True(t, sc.Axis.Equal(r3.NewVec(1, 2, 3).Scale(1/math.Sqrt(14)), screwTol))
		require.InDelta(t, math.Sqrt(14), sc.Slide, screwTol)
		require.True(t, sc.Point.Equal(r3.Vec{}, screwTol))
	})

	t.Run("a rotation about an offset axis", func(t *testing.T) {
		t.Parallel()
		tr := must(r3.RotationAround(r3.NewVec(10, 0, 0), axisZ, units.Degrees(90)))
		sc := rebuild(t, tr, screwTol)
		require.InDelta(t, math.Pi/2, screwRadians(t, sc), screwTol)
		require.True(t, sc.Axis.Equal(axisZ, screwTol))
		require.True(t, sc.Point.Equal(r3.NewVec(10, 0, 0), screwTol))
		require.InDelta(t, 0, sc.Slide, screwTol)
	})

	t.Run("a screw", func(t *testing.T) {
		t.Parallel()
		tr := must(must(r3.Rotation(axisZ, units.Degrees(90))).Then(must(r3.Translation(r3.NewVec(0, 0, 5)))))
		sc := rebuild(t, tr, screwTol)
		require.InDelta(t, 5, sc.Slide, screwTol)
		require.True(t, sc.Point.Equal(r3.Vec{}, screwTol))
	})

	t.Run("random rebuild", func(t *testing.T) {
		t.Parallel()
		rng := rand.New(rand.NewPCG(1, 2))
		for range 200 {
			tr := randProper(t, rng)
			sc := rebuild(t, tr, screwTol)
			rad := screwRadians(t, sc)
			require.GreaterOrEqual(t, rad, 0.0)
			require.LessOrEqual(t, rad, math.Pi)
			require.LessOrEqual(t, math.Abs(sc.Point.Dot(sc.Axis)), 1e-9*(1+sc.Point.Len()))

			zero, err := sc.At(0)
			require.NoError(t, err)
			require.True(t, zero.Equal(r3.Identity(), 0))

			half, err := sc.At(0.5)
			require.NoError(t, err)
			twice, err := half.Then(half)
			require.NoError(t, err)
			require.True(t, twice.Equal(tr, screwTol))
		}
	})

	t.Run("angle and axis of a known rotation", func(t *testing.T) {
		t.Parallel()
		rng := rand.New(rand.NewPCG(3, 4))
		for range 20 {
			n := unit(randVec(rng))
			for _, theta := range []float64{1e-3, 0.5, math.Pi / 2, 2, math.Pi - 1e-3, math.Pi} {
				tr := must(r3.Rotation(n, units.Radians(theta)))
				sc := rebuild(t, tr, screwTol)
				require.InDelta(t, theta, screwRadians(t, sc), screwTol)
				if theta == math.Pi {
					require.True(t, sc.Axis.Equal(n, screwTol) || sc.Axis.Equal(n.Scale(-1), screwTol))
				} else {
					require.True(t, sc.Axis.Equal(n, screwTol), "theta %v: %v vs %v", theta, sc.Axis, n)
				}
			}
		}
	})

	t.Run("near pi uses the symmetric part", func(t *testing.T) {
		t.Parallel()
		tr := must(r3.Rotation(r3.NewVec(1, 1, 1), units.Radians(math.Pi-1e-8)))
		sc := rebuild(t, tr, screwTol)
		require.True(t, sc.Axis.Equal(r3.NewVec(1, 1, 1).Scale(1/math.Sqrt(3)), screwTol), "axis %v", sc.Axis)
	})

	t.Run("an exact half-turn with a symmetric linear part", func(t *testing.T) {
		t.Parallel()
		tr := must(r3.FromBasis(r3.Basis{EX: r3.NewVec(-1, 0, 0), EY: r3.NewVec(0, -1, 0), EZ: axisZ}, r3.Vec{}))
		sc, err := tr.Screw()
		require.NoError(t, err)
		require.Equal(t, math.Pi, screwRadians(t, sc))
		require.Equal(t, r3.NewVec(0, 0, 1), sc.Axis)
		for _, c := range []float64{sc.Axis.X, sc.Axis.Y, sc.Axis.Z, sc.Point.X, sc.Point.Y, sc.Point.Z, sc.Slide} {
			require.False(t, math.Signbit(c), "negative zero or negative component in %v", sc)
		}
	})

	t.Run("a symmetric half-turn's first nonzero axis component is positive", func(t *testing.T) {
		t.Parallel()
		// 2nnᵀ − I for n = (−0.6, 0.8, 0), symmetric to the bit. The longest
		// column of S is Y, whose normalization is +n (X negative), so the
		// canonical sign rule must flip it.
		tr := r3.TransformWithBasis(r3.Basis{
			EX: r3.NewVec(-0.28, -0.96, 0),
			EY: r3.NewVec(-0.96, 0.28, 0),
			EZ: r3.NewVec(0, 0, -1),
		}, r3.Vec{})
		sc, err := tr.Screw()
		require.NoError(t, err)
		require.Equal(t, math.Pi, screwRadians(t, sc))
		require.True(t, sc.Axis.Equal(r3.NewVec(0.6, -0.8, 0), screwTol), "axis %v", sc.Axis)
		require.False(t, math.Signbit(sc.Axis.Z), "negative zero in %v", sc.Axis)
	})

	t.Run("a half-turn's axis sign follows the stored residue", func(t *testing.T) {
		t.Parallel()
		for _, deg := range []float64{180, -180} {
			tr := must(r3.Rotation(axisZ, units.Degrees(deg)))
			sc := rebuild(t, tr, screwTol)
			require.InDelta(t, math.Pi, screwRadians(t, sc), screwTol)
			require.True(t, sc.Axis.Equal(axisZ, screwTol) || sc.Axis.Equal(axisZ.Scale(-1), screwTol))
		}
	})

	t.Run("a tiny rotation with a sideways translation", func(t *testing.T) {
		t.Parallel()
		tr := must(must(r3.Rotation(axisZ, units.Radians(1e-8))).Then(must(r3.Translation(axisX))))
		sc := rebuild(t, tr, screwTol)
		require.InDelta(t, 1e-8, screwRadians(t, sc), 1e-18)
		require.True(t, sc.Point.Equal(r3.NewVec(0.5, 1e8, 0), 1e-4), "point %v", sc.Point)
	})

	t.Run("a rotation below Normalize's floor", func(t *testing.T) {
		t.Parallel()
		tr := must(must(r3.Rotation(axisZ, units.Radians(1e-13))).Then(must(r3.Translation(axisX))))
		rebuild(t, tr, screwTol)
	})

	t.Run("a full turn is a near-zero rotation", func(t *testing.T) {
		t.Parallel()
		tr := must(r3.Rotation(axisZ, units.Radians(2*math.Pi)))
		sc := rebuild(t, tr, screwTol)
		require.Less(t, screwRadians(t, sc), 1e-15)
		require.Equal(t, 0.0, sc.Slide)
	})

	t.Run("a within-tolerance skew is admitted", func(t *testing.T) {
		t.Parallel()
		tr := r3.TransformWithBasis(r3.Basis{EX: axisX, EY: r3.NewVec(7e-10, 1, 0), EZ: axisZ}, r3.NewVec(1, 2, 3))
		rebuild(t, tr, 2e-9)
	})

	t.Run("a reflection is improper", func(t *testing.T) {
		t.Parallel()
		xy, err := r3.NewFrame(r3.Vec{}, axisX, axisY)
		require.NoError(t, err)
		sc, err := must(r3.Reflection(xy)).Screw()
		require.ErrorIs(t, err, r3.ErrImproper)
		require.Equal(t, r3.Screw{}, sc)
	})

	t.Run("an invalid transform is refused", func(t *testing.T) {
		t.Parallel()
		_, err := r3.Transform{}.Screw()
		require.ErrorIs(t, err, r3.ErrNotOrthonormal)
		_, err = r3.TransformWithTranslation(r3.NewVec(math.NaN(), 0, 0)).Screw()
		require.ErrorIs(t, err, r3.ErrNonFinite)
	})

	t.Run("an unrepresentable point is refused", func(t *testing.T) {
		t.Parallel()
		tr := must(must(r3.Rotation(axisZ, units.Radians(1e-300))).Then(must(r3.Translation(r3.NewVec(1e10, 0, 0)))))
		_, err := tr.Screw()
		require.ErrorIs(t, err, r3.ErrNonFinite)
	})
}

func TestScrewAt(t *testing.T) {
	t.Parallel()
	must := mustT(t)

	t.Run("accepts any point on the axis", func(t *testing.T) {
		t.Parallel()
		want := must(r3.RotationAround(r3.NewVec(10, 0, 0), axisZ, units.Degrees(90)))
		for _, axis := range []r3.Vec{axisZ, r3.NewVec(0, 0, 3)} {
			got, err := r3.Screw{Axis: axis, Point: r3.NewVec(10, 0, 7), Angle: units.Degrees(90)}.At(1)
			require.NoError(t, err)
			require.True(t, got.Equal(want, screwTol))
		}
	})

	t.Run("extrapolates", func(t *testing.T) {
		t.Parallel()
		sc := r3.Screw{Axis: axisZ, Point: r3.NewVec(10, 0, 0), Angle: units.Degrees(50), Slide: 3}
		one, err := sc.At(1)
		require.NoError(t, err)
		two, err := sc.At(2)
		require.NoError(t, err)
		oneTwice, err := one.Then(one)
		require.NoError(t, err)
		require.True(t, two.Equal(oneTwice, screwTol))
		minus, err := sc.At(-1)
		require.NoError(t, err)
		inv, err := one.Inverse()
		require.NoError(t, err)
		require.True(t, minus.Equal(inv, screwTol))
	})

	t.Run("rejects the zero value", func(t *testing.T) {
		t.Parallel()
		_, err := r3.Screw{}.At(0.5)
		require.ErrorIs(t, err, r3.ErrDegenerateAxis)
	})

	t.Run("rejects a non-finite fraction", func(t *testing.T) {
		t.Parallel()
		sc := r3.Screw{Axis: axisZ, Angle: units.Radians(1)}
		for _, f := range []float64{math.NaN(), math.Inf(1)} {
			_, err := sc.At(f)
			require.ErrorIs(t, err, r3.ErrNonFinite)
		}
	})

	t.Run("rejects an angle that is not an angle", func(t *testing.T) {
		t.Parallel()
		for _, a := range []units.Value{units.Millimeters(1), {}} {
			_, err := r3.Screw{Axis: axisZ, Angle: a}.At(1)
			require.ErrorIs(t, err, units.ErrIncompatible)
		}
		_, err := r3.Screw{Axis: axisZ, Angle: units.Radians(math.NaN())}.At(1)
		require.ErrorIs(t, err, r3.ErrNonFinite)
	})

	t.Run("rejects a non-finite point or slide", func(t *testing.T) {
		t.Parallel()
		_, err := r3.Screw{Axis: axisZ, Angle: units.Radians(1), Point: r3.NewVec(math.NaN(), 0, 0)}.At(1)
		require.ErrorIs(t, err, r3.ErrNonFinite)
		_, err = r3.Screw{Axis: axisZ, Angle: units.Radians(1), Slide: math.Inf(1)}.At(1)
		require.ErrorIs(t, err, r3.ErrNonFinite)
	})
}

func TestTransformInterpolate(t *testing.T) {
	t.Parallel()
	must := mustT(t)

	t.Run("endpoints", func(t *testing.T) {
		t.Parallel()
		rng := rand.New(rand.NewPCG(5, 6))
		for range 200 {
			from, to := randProper(t, rng), randProper(t, rng)
			at0, err := from.Interpolate(to, 0)
			require.NoError(t, err)
			require.True(t, at0.Equal(from, screwTol))
			at1, err := from.Interpolate(to, 1)
			require.NoError(t, err)
			require.True(t, at1.Equal(to, screwTol))
		}
	})

	t.Run("world and body frames trace the same path", func(t *testing.T) {
		t.Parallel()
		rng := rand.New(rand.NewPCG(7, 8))
		for range 50 {
			from, to := randProper(t, rng), randProper(t, rng)
			inv := must(from.Inverse())
			rel := must(inv.Then(to))
			relBody := must(to.Then(inv))
			relSc, err := rel.Screw()
			require.NoError(t, err)
			bodySc, err := relBody.Screw()
			require.NoError(t, err)
			for _, s := range []float64{0.25, 0.5, 0.75} {
				a, err := relSc.At(s)
				require.NoError(t, err)
				b, err := bodySc.At(s)
				require.NoError(t, err)
				require.True(t, must(from.Then(a)).Equal(must(b.Then(from)), screwTol))
			}
		}
	})

	t.Run("takes the shorter arc", func(t *testing.T) {
		t.Parallel()
		got, err := r3.Identity().Interpolate(must(r3.Rotation(axisZ, units.Degrees(270))), 0.5)
		require.NoError(t, err)
		require.True(t, got.Equal(must(r3.Rotation(axisZ, units.Degrees(-45))), screwTol))
	})

	t.Run("a half-turn picks one arc deterministically", func(t *testing.T) {
		t.Parallel()
		to := must(r3.Rotation(axisZ, units.Degrees(180)))
		got, err := r3.Identity().Interpolate(to, 0.5)
		require.NoError(t, err)
		sc, err := got.Screw()
		require.NoError(t, err)
		require.InDelta(t, math.Pi/2, screwRadians(t, sc), screwTol)
		require.True(t, sc.Axis.Equal(axisZ, screwTol) || sc.Axis.Equal(axisZ.Scale(-1), screwTol))
		require.True(t, must(got.Then(got)).Equal(to, screwTol))
		again, err := r3.Identity().Interpolate(to, 0.5)
		require.NoError(t, err)
		require.Equal(t, got, again)
	})

	t.Run("two reflections interpolate, one does not", func(t *testing.T) {
		t.Parallel()
		xy, err := r3.NewFrame(r3.Vec{}, axisX, axisY)
		require.NoError(t, err)
		yz, err := r3.NewFrame(r3.Vec{}, axisY, axisZ)
		require.NoError(t, err)
		rxy, ryz := must(r3.Reflection(xy)), must(r3.Reflection(yz))
		got, err := rxy.Interpolate(ryz, 0.5)
		require.NoError(t, err)
		require.True(t, got.IsReflection())
		_, err = r3.Identity().Interpolate(rxy, 0.5)
		require.ErrorIs(t, err, r3.ErrImproper)
	})

	t.Run("a pure translation moves linearly", func(t *testing.T) {
		t.Parallel()
		got, err := r3.Identity().Interpolate(must(r3.Translation(r3.NewVec(2, 4, 6))), 0.25)
		require.NoError(t, err)
		require.True(t, got.Equal(must(r3.Translation(r3.NewVec(0.5, 1, 1.5))), screwTol))
	})
}
