package r3

import (
	"errors"
	"math"
)

// ErrNonFinite is returned by any constructor handed — or computing — a NaN or
// infinite number: an angle, a position, an origin, a translation. Such a value
// is not a point of ℝ³ and not an amount of rotation, so no rigid motion
// corresponds to it. Every constructor rejects it rather than propagate it,
// which is what keeps the [Transform] and [Frame] invariants true without an
// asterisk.
//
// It is distinct from [ErrDegenerateAxis], [ErrDegenerateFrame] and
// [ErrNotOrthonormal]: those name a direction that cannot be recovered from the
// input, whereas this one names a number that was never real to begin with.
var ErrNonFinite = errors.New("r3: non-finite value (NaN or Inf)")

// zeroLen is the threshold below which a vector is treated as having no
// direction. It is a divide-by-zero guard for Normalize, not a geometric
// tolerance.
const zeroLen = 1e-12

// isFinite reports whether x is a real number: neither NaN nor infinite.
//
// The predicate is phrased positively — |x| <= MaxFloat64, an ACCEPT test —
// rather than as a rejection such as math.IsNaN(x) || math.IsInf(x, 0). A NaN
// compares false against every bound whichever way the test is written, so it
// must be made to fail an accept test; a reject test it would sail straight
// through. The whole package leans on this convention.
func isFinite(x float64) bool { return math.Abs(x) <= math.MaxFloat64 }

// Vec is a vector (or point) in 3-space: pure transient coordinate math. It
// carries no document state.
type Vec struct {
	X, Y, Z float64
}

// NewVec returns the vector (x, y, z).
func NewVec(x, y, z float64) Vec { return Vec{X: x, Y: y, Z: z} }

// Add returns v + o.
func (v Vec) Add(o Vec) Vec { return Vec{v.X + o.X, v.Y + o.Y, v.Z + o.Z} }

// Sub returns v − o.
func (v Vec) Sub(o Vec) Vec { return Vec{v.X - o.X, v.Y - o.Y, v.Z - o.Z} }

// Scale returns v scaled by s.
func (v Vec) Scale(s float64) Vec { return Vec{v.X * s, v.Y * s, v.Z * s} }

// Dot returns the dot product v · o.
func (v Vec) Dot(o Vec) float64 { return v.X*o.X + v.Y*o.Y + v.Z*o.Z }

// Cross returns the cross product v × o.
func (v Vec) Cross(o Vec) Vec {
	return Vec{
		v.Y*o.Z - v.Z*o.Y,
		v.Z*o.X - v.X*o.Z,
		v.X*o.Y - v.Y*o.X,
	}
}

// Len returns the Euclidean length of v.
//
// It is computed by nested [math.Hypot] rather than the naive sqrt of the sum of
// squares: squaring a large-but-finite component overflows to +Inf (e.g. 1e200²)
// and a small one underflows to 0, so the naive form reports an infinite length
// for a perfectly representable vector. Hypot scales internally and so is exact
// over the whole finite range.
//
// Len is NaN only if some component is NaN. It is +Inf if some component is
// infinite — and also in the one honest overflow: when the true length is itself
// past [math.MaxFloat64], as for (MaxFloat64, MaxFloat64, 0), whose length is
// √2·MaxFloat64. No float64 can name that number, so no implementation could do
// better. The DIRECTION of such a vector is still recoverable, and
// [Vec.Normalize] recovers it without going through Len.
func (v Vec) Len() float64 { return math.Hypot(math.Hypot(v.X, v.Y), v.Z) }

// isFinite reports whether every component of v is finite, i.e. whether v names
// an actual point (or direction) of ℝ³. A vector with a NaN or infinite
// component does not, and no rigid motion can be built from it.
func (v Vec) isFinite() bool { return isFinite(v.X) && isFinite(v.Y) && isFinite(v.Z) }

// scaledDown returns v scaled by the exact power of two 2⁻ᵉ that brings v's
// largest absolute component into [0.5, 1), together with that exponent e. It
// returns false when there is no such scaling: v is the zero vector, or some
// component is NaN or infinite.
//
// It is how the package takes a dot product against a vector whose components
// are enormous without overflowing on the way there: scale down, compute in the
// scaled binade, scale the RESULT back with [Vec.ldexp]. Scaling by a power of
// two shifts the exponent of every component and touches no mantissa bit, so the
// arithmetic done on the scaled vector is bit for bit what it would have been on
// the original — minus the overflow. What is left is one honest failure mode: the
// scale-back itself can overflow, and it does so exactly when the true result is
// too large to name, which is a result worth rejecting.
//
// The guard is phrased positively (0 < m <= MaxFloat64, an ACCEPT test) so that a
// NaN component — which makes m NaN, and every comparison against NaN is false —
// fails it rather than sails through it.
func (v Vec) scaledDown() (Vec, int, bool) {
	m := math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z)))
	if !(m > 0 && m <= math.MaxFloat64) {
		return Vec{}, 0, false
	}
	_, e := math.Frexp(m)
	return Vec{
		X: math.Ldexp(v.X, -e),
		Y: math.Ldexp(v.Y, -e),
		Z: math.Ldexp(v.Z, -e),
	}, e, true
}

// ldexp returns v with every component multiplied by 2ᵉ. It undoes the scaling
// [Vec.scaledDown] applied, and like it, it is exact — except for the one
// overflow to ±Inf that is the caller's business to detect, since a component
// past MaxFloat64 here is a genuinely unrepresentable result and not an artefact
// of the order of operations.
func (v Vec) ldexp(e int) Vec {
	return Vec{
		X: math.Ldexp(v.X, e),
		Y: math.Ldexp(v.Y, e),
		Z: math.Ldexp(v.Z, e),
	}
}

// Equal reports whether v and o agree componentwise within tol. It is a
// tolerant comparison for floating-point results, not an exact one: two vectors
// that should be equal rarely are, bit for bit, once any arithmetic has touched
// them.
func (v Vec) Equal(o Vec, tol float64) bool {
	return math.Abs(v.X-o.X) <= tol &&
		math.Abs(v.Y-o.Y) <= tol &&
		math.Abs(v.Z-o.Z) <= tol
}

// Normalize returns the unit vector along v and true, or the zero vector and
// false when v has no usable direction. The boolean is deliberate: unlike a
// floor-against-zero helper, Normalize never fabricates a non-unit direction —
// callers must handle the false case.
//
// A direction is usable exactly when every component of v is finite and v is not
// (near-)zero. So:
//
//   - any component is NaN or infinite: false. Such a v is not a vector of ℝ³ at
//     all; there is no direction to report, and dividing through by +Inf would
//     silently flatten v to the zero vector.
//   - v is (near-)zero (length below zeroLen): false.
//   - v is huge but finite, e.g. (1e200, 1e200, 1e200): true, with the correct
//     unit vector.
//   - v is so huge that its LENGTH overflows, e.g. (MaxFloat64, MaxFloat64, 0):
//     also true, with the correct unit vector — here (1/√2, 1/√2, 0). The length
//     is not representable but the direction is, and the direction is all
//     Normalize was asked for. It is obtained by dividing v through by its
//     largest absolute component first, which cannot overflow, and normalizing
//     that. Rejecting this case would blame the direction for a defect of the
//     magnitude — and callers such as [NewFrame] would then report "degenerate
//     axes" about axes that were never degenerate.
//
// The guards are phrased positively (!(accept) rather than reject): a NaN
// compares false whichever way the test is written, so it must fail an accept
// test rather than pass a reject test.
func (v Vec) Normalize() (Vec, bool) {
	// Finiteness is checked on the COMPONENTS, not on the length: a finite v may
	// still have an unrepresentable length, and that is not a reason to refuse it
	// a direction.
	if !v.isFinite() {
		return Vec{}, false
	}
	l := v.Len()
	if !(l >= zeroLen) {
		return Vec{}, false
	}
	if l <= math.MaxFloat64 {
		return v.Scale(1 / l), true
	}
	// The length overflowed even though v did not. Divide by the largest absolute
	// component — an exact-magnitude scaling that maps the largest component to
	// ±1, so the scaled length lies in [1, √3] and cannot overflow — then
	// normalize that. The result is the direction of v, which is what was asked
	// for; only its (unrepresentable) length is lost.
	m := math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z)))
	s := Vec{X: v.X / m, Y: v.Y / m, Z: v.Z / m}
	return s.Scale(1 / s.Len()), true
}
