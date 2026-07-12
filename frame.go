package r3

import (
	"errors"
	"math"
)

// ErrDegenerateFrame is returned by [NewFrame] when its axes are zero or
// collinear, so no orthonormal frame can be built from them.
var ErrDegenerateFrame = errors.New("r3: degenerate frame (zero or collinear axes)")

// orthoTol is how far the stored axes may drift from unit length / mutual
// orthogonality before [Frame.IsValid] rejects them.
const orthoTol = 1e-9

// Frame is a right-handed orthonormal coordinate frame in world space: an
// origin plus two in-plane unit axes U and V. The normal N is always U × V and
// is derived, never stored, so the frame cannot disagree with its own normal.
//
// The fields are unexported: a Frame can only be created through [NewFrame],
// which enforces orthonormality. The zero value Frame{} is invalid (see
// [Frame.IsValid]); a caller that receives a Frame from outside must validate it
// before trusting it.
type Frame struct {
	origin, u, v Vec
}

// NewFrame returns an orthonormal right-handed frame at origin whose first axis
// is along u and whose second axis lies in the u–v plane. The axes come back
// orthonormalized (u is kept; v is made perpendicular to u; both are normalized) —
// Gram–Schmidt in effect, though not by the projection that name suggests; see
// below. It returns [ErrDegenerateFrame] when u or v is zero, or when the two are
// collinear — there is then no plane, and any frame NewFrame handed back would be
// one it had invented.
//
// It returns [ErrNonFinite] when origin has a NaN or infinite component: the
// origin is stored as given, not normalized, so nothing downstream would ever
// catch it — and a frame with a NaN origin poisons every point that passes
// through [Frame.ToWorld] or [FromFrame]. That case is [ErrNonFinite] and not
// [ErrDegenerateFrame] because the axes are fine; it is the position that is
// not a position. ErrDegenerateFrame stays reserved for what it says: zero or
// collinear AXES (a non-finite AXIS is reported as ErrDegenerateFrame too — it is
// no direction, and that is the shipped contract).
//
// Axis MAGNITUDE is not a reason for refusal, however large, and neither is a
// wide dynamic range WITHIN an axis — down to the smallest denormal.
//
// The ANGLE between the axes has a floor, though, and it is deliberate: an angle
// below about two ULP (~5e-16 rad) is treated as collinear. At that separation
// the bits cannot say whether the caller meant a razor-thin real plane or handed
// in collinear input rounded by its own arithmetic — v = 1.1*u, stored, leaves
// exactly the same one-ulp determinant residue as an axis deliberately one ulp
// off — and a frame built there would carry a normal whose direction is that
// rounding noise. Below the floor NewFrame prefers the conservative error to the
// fabricated normal. A real angle of 1e-13 rad, three orders above the floor,
// builds fine.
//
// # Everything is judged on the axes AS GIVEN
//
// Collinearity is decided by the normal n = u × v, computed from the ORIGINAL u and
// v — never from a normalized u. Normalizing first is not the same question: it
// ROUNDS, and what it rounds away can be the entire evidence of degeneracy. For
// u = (MaxFloat64/2, 1e-20, 0) the unit vector is (0.9999999999999999, 0, 0) — the
// 1e-20 underflowed — so v = 2·u, which is LITERALLY collinear and spans no plane,
// showed a perpendicular against that ROUNDED u and got a frame built out of
// nothing. Judging n on the axes themselves makes the exactly-collinear case an
// exactly zero cross product, which is refused.
//
// The in-plane perpendicular then comes from a SECOND cross with the original u,
//
//	perp  =  n × u  =  (u × v) × u  =  v·(u·u) − u·(u·v)
//
// which is Lagrange's formula, and is |u|² times the Gram–Schmidt perpendicular:
// the same vector, a POSITIVE multiple, so V lands on the side of u that the
// caller's v was on. The operand order is load-bearing at both crosses — u × v, and
// then n × u, not u × n. Reversing either negates perp, which flips V, hence
// N = U × V, and silently inverts the frame's handedness.
//
// # Why the perpendicular is not a projection
//
// The textbook Gram–Schmidt perpendicular, v − un·(v·un), cannot be computed for
// such axes, and no rescaling saves it: it must first form the scalar v·un, and
// for u = (1, 1, 0) with v = (MaxFloat64, MaxFloat64, SmallestNonzeroFloat64) that
// scalar is √2·MaxFloat64 — a number float64 DOES NOT HAVE. The projection
// overflows to +Inf on the way to a perpendicular, (0, 0, 1), that is perfectly
// ordinary. Scaling v down first only trades the fault for its mirror image: the
// quarter that keeps the projection finite flushes that denormal Z — the whole of
// the perpendicular — to zero. Either way NewFrame reported "degenerate axes"
// about axes that are finite and plainly not collinear.
//
// The double cross never forms that scalar. The huge components cancel INSIDE the
// cross product, where they cancel against each other rather than against a sum
// that has already overflowed: for the pair above, u × v is (5e−324, −5e−324, 0),
// and crossing that with u recovers the Z direction the denormal was carrying,
// undisturbed. Only the DIRECTION of perp is wanted — it is normalized immediately
// — so its magnitude never has to be reconstructed, and any scaling applied along
// the way can simply be left in place.
//
// # The two things the deciding cross product needs
//
//   - Range. A cross product component is a difference of two products whose true
//     values span MaxFloat64² down past the denormals, and float64 cannot hold
//     either end (u × v overflows for u = (Max, Max, Max), v = (Max, Max, −Max)).
//     No vector-level rescaling fixes that: components can span more than 600
//     decimal orders (u = (Max/2, Max/2, 1e-20)), and the representable range
//     below 1.0 is only ~308 orders plus the denormals, so scaling such a vector
//     by ITS OWN largest component — any variant of it — flushes exactly the small
//     component that decides collinearity, and the "collinear or not" verdict
//     comes back wrong. So the deciding cross is not computed in float64's range
//     at all: [Vec.crossExp] carries each component in (mantissa, exponent) form,
//     where nothing can overflow or underflow between the axes and the verdict.
//   - Collinearity. The cross product of collinear axes is zero — the right signal,
//     and the one degeneracy check this needs — but in float64 two enormous products
//     cancelling leave a last-bit residue behind instead of a clean zero, and a
//     residue is a direction. The kernel zeroes any component that lies within the
//     rounding band of the two products that formed it (the same band
//     [Vec.crossFiltered] applies, taken on the aligned mantissas), so collinear
//     axes produce an exactly zero cross product and are refused, while a denormal
//     perpendicular that no rounding touched survives.
//
// Both axes are then normalized scale-free (see [Vec.direction]) rather than through
// [Vec.Normalize], whose zeroLen floor is sized for vectors of order one and would
// call a legitimate 1e-20 perpendicular "degenerate" for being small.
func NewFrame(origin, u, v Vec) (Frame, error) {
	if !origin.isFinite() {
		return Frame{}, ErrNonFinite
	}
	// A non-finite AXIS is ErrDegenerateFrame, not ErrNonFinite — the shipped
	// contract. Checked here rather than left to fall out of the arithmetic, so it
	// cannot depend on which of NaN's many products happens to arise below.
	if !u.isFinite() || !v.isFinite() {
		return Frame{}, ErrDegenerateFrame
	}
	// The plane normal, from the axes AS GIVEN — no normalization or rescaling has
	// touched them, so nothing has been rounded or flushed away. The exponent-
	// tracked kernel makes the zero-or-not call per component at the mantissa
	// level, where neither a MaxFloat64² product nor a 1e-20 straggler 600 decimal
	// orders below it can be lost to float64's exponent range. Exactly zero is
	// exactly the degeneracy: u is zero, or v is, or the two are collinear and span
	// no plane. THIS is the degeneracy test; the orthonormality post-check below
	// cannot be, because V is perpendicular to U by construction and would pass it
	// whatever nonsense it was built from.
	nDir, ok := u.crossExp(v).direction()
	if !ok {
		return Frame{}, ErrDegenerateFrame
	}
	un, ok := u.direction()
	if !ok {
		return Frame{}, ErrDegenerateFrame
	}
	// perp = n × u = (u × v) × u = |u|²·(v − û·(v·û)): the Gram–Schmidt
	// perpendicular scaled by a POSITIVE factor, so it points to v's side of u. The
	// operand order fixes the sign; u × n is this vector negated, and would invert
	// the frame's handedness. Only DIRECTIONS matter from here on, so the cross is
	// taken between the two unit vectors: every product is then at most 1 in
	// magnitude and the plain filtered cross cannot overflow.
	//
	// Reducing u to its direction is safe HERE in a way it was NOT safe one step
	// up: the degeneracy decision no longer depends on it, and a component of u
	// sitting more than 2⁻¹⁰⁷⁴ RELATIVE below u's largest one — the only kind
	// direction() can flush — cannot be represented in a unit vector anyway, so
	// nothing decisive is lost where it matters.
	perp := nDir.crossFiltered(un)
	// perp is taken for its DIRECTION, so it is normalized scale-free: its length is
	// legitimately 1e-20 for u = X, v = (MaxFloat64, 1e-20, 0), and Normalize's
	// zeroLen floor — a divide-by-zero guard sized for vectors of order one — would
	// call that direction "degenerate" merely for being small.
	vn, ok := perp.direction()
	if !ok {
		return Frame{}, ErrDegenerateFrame
	}
	// ASK whether the perpendicular is perpendicular, at the same tolerance
	// [Frame.IsValid] will hold it to: a frame NewFrame returns always passes
	// IsValid. Phrased positively, so a NaN that got this far fails it. It is a
	// post-check on the arithmetic, not the degeneracy signal — that was n above.
	if !(math.Abs(un.Dot(vn)) <= orthoTol) {
		return Frame{}, ErrDegenerateFrame
	}
	return Frame{origin: origin, u: un, v: vn}, nil
}

// Origin returns the world position of the frame's local (0, 0, 0).
func (f Frame) Origin() Vec { return f.origin }

// U returns the frame's first in-plane unit axis.
func (f Frame) U() Vec { return f.u }

// V returns the frame's second in-plane unit axis.
func (f Frame) V() Vec { return f.v }

// N returns the frame's unit normal, U × V. It is derived on every call, never
// stored.
func (f Frame) N() Vec { return f.u.Cross(f.v) }

// IsValid reports whether the frame's origin is finite and its axes are unit
// length and mutually orthogonal. The zero value Frame{} is not valid. Use it to
// vet a frame supplied by a caller before building geometry on it.
//
// A NaN or infinite component — in an axis OR in the origin — makes f invalid.
// The origin counts: a frame anchored nowhere maps every local coordinate to
// NaN, which is no more a frame than one with a collapsed axis. Every check is
// phrased positively (!(x <= tol) rather than x > tol) so that a comparison
// against NaN — which is false whichever way it is written — rejects rather than
// admits.
func (f Frame) IsValid() bool {
	if !f.origin.isFinite() {
		return false
	}
	if !(math.Abs(f.u.Len()-1) <= orthoTol) {
		return false
	}
	if !(math.Abs(f.v.Len()-1) <= orthoTol) {
		return false
	}
	return math.Abs(f.u.Dot(f.v)) <= orthoTol
}

// Equal reports whether f and o agree within tol, comparing the origin and both
// in-plane axes. The normal is derived from the axes, so it needs no separate
// check.
//
// tol is a comparison tolerance chosen by the caller. It is unrelated to the
// fixed threshold [Frame.IsValid] uses to police orthonormality.
func (f Frame) Equal(o Frame, tol float64) bool {
	return f.origin.Equal(o.origin, tol) &&
		f.u.Equal(o.u, tol) &&
		f.v.Equal(o.v, tol)
}

// ToWorld maps a local coordinate (u along U, v along V, w along N) to world
// space.
//
// It shares the package's accepted per-point limit: the terms are summed in a
// fixed order, so a coordinate near [math.MaxFloat64] can drive an intermediate
// sum to ±Inf even when the exact result is representable. ToWorld is infallible
// and so returns that non-finite Vec rather than an error — a wrong answer, not a
// refusal. It is the same trade [Transform.ApplyDir] documents, for the same
// reason (this runs once per point), and it cannot arise at any magnitude a real
// model contains. A caller working out at MaxFloat64 must check the result.
func (f Frame) ToWorld(local Vec) Vec {
	return f.origin.
		Add(f.u.Scale(local.X)).
		Add(f.v.Scale(local.Y)).
		Add(f.N().Scale(local.Z))
}

// ToWorldUV maps an in-plane 2D point (u, v) — the currency of a planar sketch
// — to world space (w = 0).
//
// It carries the same accepted per-point overflow limit as [Frame.ToWorld]: the
// terms are summed in a fixed order, so coordinates near [math.MaxFloat64] can
// drive an intermediate sum to ±Inf even when the exact result is representable.
// ToWorldUV is infallible and so returns a Vec with a non-finite component rather
// than an error — a wrong answer, not a refusal, which the caller must check for
// itself at those magnitudes.
func (f Frame) ToWorldUV(u, v float64) Vec {
	return f.origin.Add(f.u.Scale(u)).Add(f.v.Scale(v))
}

// ToLocal maps a world point to local coordinates. The third component is the
// signed distance off the plane (along N). It is the exact inverse of ToWorld
// (the transpose), valid because the frame is orthonormal.
//
// "Exact" is about the algebra, not the arithmetic: it carries the same accepted
// per-point overflow limit as [Frame.ToWorld], through the dot products. Each
// sums its terms in a fixed order, so a world point near [math.MaxFloat64] can
// drive an intermediate sum — or the world−origin subtraction ahead of it — to
// ±Inf where the exact local coordinate was representable. ToLocal is infallible
// and so returns a Vec with a non-finite component rather than an error: a wrong
// answer, not a refusal, and one the caller must check for at those magnitudes.
func (f Frame) ToLocal(world Vec) Vec {
	d := world.Sub(f.origin)
	return Vec{d.Dot(f.u), d.Dot(f.v), d.Dot(f.N())}
}
