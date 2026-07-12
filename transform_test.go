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
			require.True(t, tr.Inverse().Apply(tr.Apply(p)).Equal(p, tol))
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
			require.True(t, a.Then(b).Apply(p).Equal(b.Apply(a.Apply(p)), tol))
		}
	})

	t.Run("identity is neutral", func(t *testing.T) {
		t.Parallel()

		rng := rand.New(rand.NewPCG(5, 6))
		for range 50 {
			tr := randTransform(t, rng)
			require.True(t, tr.Then(r3.Identity()).Equal(tr, tol))
			require.True(t, r3.Identity().Then(tr).Equal(tr, tol))
		}
	})

	t.Run("orthonormality survives long chains", func(t *testing.T) {
		t.Parallel()

		// Floating-point drift is the failure mode here: a transform that has
		// been composed a hundred times must still be a rigid motion.
		rng := rand.New(rand.NewPCG(7, 8))
		acc := r3.Identity()
		for range 100 {
			acc = acc.Then(randTransform(t, rng))
			require.True(t, acc.IsValid())
		}
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
		require.True(t, tr.Then(tr).Equal(r3.Identity(), tol))
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
			world := randVec(rng)
			require.True(t, tr.Inverse().Apply(world).Equal(f.ToLocal(world), tol))
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
		place := from.Inverse().Then(to)

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
