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

// dotUnit returns v · u, where u MUST be a unit vector. That precondition is not
// a formality: it is the whole reason the result cannot be got wrong by
// overflowing on the way to it.
//
// Because every component of a unit vector obeys |uᵢ| <= 1, every product vᵢ·uᵢ
// obeys |vᵢ·uᵢ| <= |vᵢ| <= MaxFloat64 — so the PRODUCTS can never overflow, only
// their sum can (it can reach 3·MaxFloat64). The ordinary [Vec.Dot] sums them in
// a fixed order and so can hit ±Inf mid-way to a result that was perfectly
// representable, which the cold paths ([NewFrame], [Reflection]) must not accept:
// there, an overflow is indistinguishable from a vanishing perpendicular or a
// vanishing plane offset, and the caller is told "degenerate" about geometry that
// was nothing of the kind.
//
// So: sum directly, and only if that sum is not finite, redo it with each product
// scaled by the exact power of two ¼, which bounds the sum by ¾·MaxFloat64, and
// scale the sum back by 4. Scaling by a power of two moves no mantissa bit, so the
// detour is exact; and it scales the PRODUCTS, never v itself — scaling v by its
// own largest component would annihilate a small-but-decisive component (a plane
// at (MaxFloat64, 0, 1e-20) has offset 1e-20, and the offset is what is being
// asked for). Only the scale-back can overflow, and it does so exactly when the
// true dot product is past MaxFloat64 — a result worth reporting as ±Inf, which
// the callers' finiteness guards then reject.
//
// A NaN or infinite component of v propagates into the result, as it should.
func (v Vec) dotUnit(u Vec) float64 {
	px, py, pz := v.X*u.X, v.Y*u.Y, v.Z*u.Z
	// Phrased positively (an ACCEPT test) so that a NaN — false against every
	// bound whichever way the test is written — takes the same path as an
	// overflow rather than sailing through.
	if s := px + py + pz; isFinite(s) {
		return s
	}
	return ((px * 0.25) + (py * 0.25) + (pz * 0.25)) * 4
}

// direction returns the unit vector along v and true, or false when v has no
// direction at all: v is exactly zero, or some component is NaN or infinite. It
// is [Vec.Normalize] without the zeroLen floor — the magnitude is stripped by an
// exact power-of-two scaling first, so the direction of a vector is recovered
// wherever in the float64 range that vector lives, from MaxFloat64 down to the
// denormals.
//
// That is exactly what the floor is FOR, so this is not a drop-in replacement:
// Normalize's floor protects a CALLER who asks for the direction of what may be
// noise. direction is for the one place inside the package that has already
// established its vector is not noise — [NewFrame]'s in-plane perpendicular,
// whose scale is set by axes the caller chose and can legitimately be 1e-20 (from
// u = (1, 0, 0), v = (MaxFloat64, 1e-20, 0)) without being any less of a
// direction. NewFrame checks the result is genuinely orthogonal to u, which is
// what rules out the noise this helper would otherwise happily normalize.
//
// The guard is phrased positively (0 < m <= MaxFloat64, an ACCEPT test) so that a
// NaN component — which makes m NaN, and every comparison against NaN is false —
// fails it rather than sails through it.
func (v Vec) direction() (Vec, bool) {
	m := math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z)))
	if !(m > 0 && m <= math.MaxFloat64) {
		return Vec{}, false
	}
	// Scale the largest component into [0.5, 1) — exactly, by shifting exponents,
	// so no mantissa bit moves. The scaled length then lies in [0.5, √3]: it can
	// neither overflow nor underflow, whatever v was.
	_, e := math.Frexp(m)
	s := Vec{
		X: math.Ldexp(v.X, -e),
		Y: math.Ldexp(v.Y, -e),
		Z: math.Ldexp(v.Z, -e),
	}
	return s.Scale(1 / s.Len()), true
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
	// The length overflowed even though v did not. Strip the magnitude first — an
	// exact power-of-two scaling that leaves a vector whose length cannot overflow
	// — and normalize that. The result is the direction of v, which is what was
	// asked for; only its (unrepresentable) length is lost.
	return v.direction()
}
