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
// is along u and whose second axis lies in the u–v plane. The axes are
// orthonormalized with Gram–Schmidt (u is kept; v is made perpendicular to u;
// both are normalized). It returns [ErrDegenerateFrame] when u is zero or when v
// is collinear with u — either because the perpendicular component vanishes
// outright, or because the subtraction that computes it cancels down to rounding
// noise, which is collinearity as far as float64 can tell. Collinearity is judged
// by what comes out: the perpendicular must actually BE perpendicular to u, to the
// same tolerance [Frame.IsValid] applies, so a frame returned by NewFrame always
// passes IsValid.
//
// It returns [ErrNonFinite] when origin has a NaN or infinite component: the
// origin is stored as given, not normalized, so nothing downstream would ever
// catch it — and a frame with a NaN origin poisons every point that passes
// through [Frame.ToWorld] or [FromFrame]. That case is [ErrNonFinite] and not
// [ErrDegenerateFrame] because the axes are fine; it is the position that is
// not a position. ErrDegenerateFrame stays reserved for what it says: zero or
// collinear AXES.
//
// Axis MAGNITUDE is not a reason for refusal, however large, and neither is a
// wide dynamic range WITHIN an axis. Two things make that so, and both are exact
// powers of two, so neither costs a mantissa bit:
//
//   - The projection v·un is taken with [Vec.dotUnit], which scales the three
//     PRODUCTS rather than v, so the perpendicular of u = (1, 0, 0) with
//     v = (MaxFloat64, 1e-20, 0) — finite, plainly not collinear — is the 1e-20
//     it actually is, not the zero that scaling v by its own largest component
//     would have made of it.
//   - v is then quartered before the projection is subtracted off. The true
//     perpendicular of two finite axes can have components past MaxFloat64
//     (u = (MaxFloat64, MaxFloat64, MaxFloat64) with
//     v = (MaxFloat64, MaxFloat64, −MaxFloat64) is such a pair), so the subtraction
//     is done a quarter of the way down, where it cannot overflow. Only the
//     DIRECTION of the perpendicular is wanted, and direction is scale-invariant,
//     so the quarter is never undone. For the same reason the perpendicular is
//     normalized scale-free (see [Vec.direction]) rather than through
//     [Vec.Normalize], whose zeroLen floor is sized for vectors of order one and
//     would call a legitimate 1e-20 perpendicular "degenerate" for being small.
//
// Without all three, NewFrame reported "degenerate axes" about axes that were
// nothing of the kind.
func NewFrame(origin, u, v Vec) (Frame, error) {
	if !origin.isFinite() {
		return Frame{}, ErrNonFinite
	}
	un, ok := u.Normalize()
	if !ok {
		return Frame{}, ErrDegenerateFrame
	}
	// Quarter v — an exact, bit-for-bit scaling that is never undone, since only
	// the direction of the perpendicular is wanted. It buys headroom: the true
	// perpendicular of two finite axes can want a component past MaxFloat64, and
	// an overflow to ±Inf here is indistinguishable from a vanishing perpendicular
	// — a false "degenerate". Quartered, |vsᵢ| <= ¼·Max and |unᵢ·(vs·un)| <=
	// (√3/4)·Max, so their difference stays under Max.
	vs := v.Scale(0.25)
	// Remove the u-component of v, leaving the in-plane perpendicular. The
	// projection goes through dotUnit — un is a unit vector — so a v holding an
	// enormous component alongside a tiny one keeps the tiny one, which is often
	// the whole of the perpendicular.
	vp := vs.Sub(un.Scale(vs.dotUnit(un)))
	// The perpendicular is taken for its DIRECTION, so it is normalized scale-free:
	// its length is legitimately 1e-20 for v = (MaxFloat64, 1e-20, 0), and
	// Normalize's zeroLen floor — a divide-by-zero guard sized for vectors of order
	// one — would call that direction "degenerate" merely for being small. Zero, NaN
	// and infinite v have no direction at all, and those direction does reject.
	vn, ok := vp.direction()
	if !ok {
		return Frame{}, ErrDegenerateFrame
	}
	// What the floor was buying, bought properly. When v is collinear with u the
	// subtraction above cancels to rounding noise, and that noise — as the
	// (MaxFloat64, MaxFloat64, MaxFloat64) / half-of-it pair shows — is a multiple
	// of u, not a perpendicular to it. So ASK whether the perpendicular is
	// perpendicular, at the same tolerance [Frame.IsValid] will hold it to: a frame
	// NewFrame returns always passes IsValid. Phrased positively, so a NaN that got
	// this far fails it.
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
