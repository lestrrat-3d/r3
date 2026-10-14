package r3_test

import (
	"math"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// refLen is Vec.Len as it was before the math.Abs calls were replaced by
// comparisons. The differential tests hold the current Len to its exact bits.
func refLen(v r3.Vec) float64 {
	ordinary := func(x float64) bool {
		return x == 0 || (x >= 0x1p-511 && x <= 0x1p511)
	}
	x, y, z := math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z)
	if ordinary(x) && ordinary(y) && ordinary(z) {
		xx, yy, zz := float64(x*x), float64(y*y), float64(z*z)
		return math.Sqrt(xx + yy + zz)
	}
	return math.Hypot(math.Hypot(v.X, v.Y), v.Z)
}

func refIsFinite(x float64) bool { return math.Abs(x) <= math.MaxFloat64 }

func adversarialFloats() []float64 {
	base := []float64{
		0, 1, 0.5, 2, 3, 0x1p-511, 0x1p511, 0x1p-1022, 0x1p-1074, 5e-324,
		math.MaxFloat64, math.SmallestNonzeroFloat64, math.Inf(1),
		math.Nextafter(0x1p-511, 0), math.Nextafter(0x1p-511, 1),
		math.Nextafter(0x1p511, 0), math.Nextafter(0x1p511, math.Inf(1)),
		math.Nextafter(math.MaxFloat64, 0), 1e-300, 1e300, 1e-20, 1e20,
	}
	var out []float64
	for _, f := range base {
		out = append(out, f, -f)
	}
	// NaNs with distinct payloads and both signs.
	for _, bits := range []uint64{
		0x7ff8000000000001, 0xfff8000000000001, 0x7ff0000000000001, 0xfff0000000000001,
		0x7fffffffffffffff, 0xffffffffffffffff, 0x7ff8dead0000beef,
	} {
		out = append(out, math.Float64frombits(bits))
	}
	return out
}

func TestLenMatchesAbsForm(t *testing.T) {
	t.Run("adversarial pairs", func(t *testing.T) {
		vals := adversarialFloats()
		for _, a := range vals {
			require.Equalf(t, refIsFinite(a), r3.IsFinite(a), "isFinite(%v)", a)
			for _, b := range vals {
				for _, c := range []float64{0, math.Copysign(0, -1), 1, -1, 0x1p511, math.Inf(-1), math.NaN()} {
					for _, v := range []r3.Vec{{X: a, Y: b, Z: c}, {X: c, Y: a, Z: b}, {X: b, Y: c, Z: a}} {
						want, got := refLen(v), v.Len()
						require.Equalf(t, math.Float64bits(want), math.Float64bits(got), "Len(%v)", v)
					}
				}
			}
		}
	})

	t.Run("random triples", func(t *testing.T) {
		rng := rand.New(rand.NewSource(1))
		rnd := func() float64 {
			switch rng.Intn(4) {
			case 0:
				return math.Float64frombits(rng.Uint64())
			case 1:
				return (rng.Float64() - 0.5) * 2e3
			case 2:
				return math.Ldexp(rng.Float64()-0.5, rng.Intn(1100)-550)
			default:
				return math.Ldexp(rng.NormFloat64(), rng.Intn(40)-20)
			}
		}
		for i := 0; i < 1_000_000; i++ {
			v := r3.Vec{X: rnd(), Y: rnd(), Z: rnd()}
			want, got := refLen(v), v.Len()
			require.Equalf(t, math.Float64bits(want), math.Float64bits(got), "Len(%v)", v)
			require.Equalf(t, refIsFinite(v.X), r3.IsFinite(v.X), "isFinite(%v)", v.X)
		}
	})
}
