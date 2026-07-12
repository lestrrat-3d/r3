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
// over the whole finite range. Len is +Inf only if some component is infinite,
// and NaN only if some component is NaN.
func (v Vec) Len() float64 { return math.Hypot(math.Hypot(v.X, v.Y), v.Z) }

// isFinite reports whether every component of v is finite, i.e. whether v names
// an actual point (or direction) of ℝ³. A vector with a NaN or infinite
// component does not, and no rigid motion can be built from it.
func (v Vec) isFinite() bool { return isFinite(v.X) && isFinite(v.Y) && isFinite(v.Z) }

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
// A direction is usable only when v's length is a finite number of at least
// zeroLen. So:
//
//   - v is (near-)zero, or any component is NaN: false.
//   - any component is infinite: false. There is no finite direction to report,
//     and dividing through by +Inf would silently flatten v to the zero vector.
//   - v is huge but finite, e.g. (1e200, 1e200, 1e200): true, with the correct
//     unit vector. [Vec.Len] does not overflow, so such a vector normalizes like
//     any other rather than being rejected.
//
// The guard is phrased positively (!(accept) rather than reject): a NaN length
// compares false whichever way the test is written, so it must fail the accept
// test rather than pass a reject test. l <= [math.MaxFloat64] is the finiteness
// half — a length is never negative, so only +Inf (and NaN) can fail it.
func (v Vec) Normalize() (Vec, bool) {
	l := v.Len()
	if !(l >= zeroLen && l <= math.MaxFloat64) {
		return Vec{}, false
	}
	return v.Scale(1 / l), true
}
