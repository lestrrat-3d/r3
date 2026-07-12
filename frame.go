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
// below. It returns [ErrDegenerateFrame] when u is zero or when v is collinear with
// u — either because the perpendicular component vanishes outright, or because the
// arithmetic that computes it cancels down to rounding noise, which is collinearity
// as far as float64 can tell. The perpendicular that does come out must actually BE
// perpendicular to u, to the same tolerance [Frame.IsValid] applies, so a frame
// returned by NewFrame always passes IsValid.
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
// wide dynamic range WITHIN an axis — down to the smallest denormal.
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
// So the perpendicular is taken by a double cross product instead, via Lagrange's
// formula a × (b × c) = b·(a·c) − c·(a·b), which for a unit un gives
//
//	un × (v × un)  =  v·(un·un) − un·(un·v)  =  v − un·(v·un)
//
// — the Gram–Schmidt perpendicular exactly, SIGN INCLUDED (note the operand order:
// v × un, not un × v; the other order is the same vector negated, and would flip V,
// hence N = U × V, and silently invert the frame's handedness). It never forms the
// unrepresentable scalar. The huge components cancel INSIDE the cross product,
// where they cancel against each other rather than against a sum that has already
// overflowed: for the pair above, v × un is (−5e−324, 5e−324, 0), and crossing that
// with un recovers the Z direction the denormal was carrying, undisturbed.
//
// Only the DIRECTION of the perpendicular is wanted — it is normalized immediately
// — so the magnitude never has to be reconstructed, and any scaling applied along
// the way can simply be left in place.
//
// # The two things the cross products need
//
//   - Overflow. A cross product component is a difference of two products and can
//     reach 2·MaxFloat64, so either cross can still overflow on finite input (v ×
//     un does for u = (Max, Max, Max), v = (Max, Max, −Max)). When one does, it is
//     redone with the operand quartered — an exact power of two, which cannot cost a
//     mantissa bit — and that is safe here in a way it is NOT safe for the projection:
//     an overflowing cross product has magnitude past MaxFloat64, so whatever the
//     quarter underflows away was, relatively, some 10⁻⁶³⁰ of the result. It could
//     not have moved the direction. (Quartering the PROJECTION threw away denormals
//     that were the entire answer, which is a different thing altogether.) A quarter
//     always suffices: each product is then at most ¼·Max, so the difference stays
//     under Max.
//   - Collinearity. The cross product of collinear axes is zero — the right signal,
//     and the one degeneracy check this needs — but in float64 two enormous products
//     cancelling leave a last-bit residue behind instead of a clean zero, and a
//     residue is a direction. [Vec.crossFiltered] zeroes any component that lies
//     within the rounding band of the two products that formed it, so collinear axes
//     produce an exactly zero cross product and are refused, while a denormal
//     perpendicular that no rounding touched survives.
//
// The perpendicular is then normalized scale-free (see [Vec.direction]) rather than
// through [Vec.Normalize], whose zeroLen floor is sized for vectors of order one and
// would call a legitimate 1e-20 perpendicular "degenerate" for being small.
func NewFrame(origin, u, v Vec) (Frame, error) {
	if !origin.isFinite() {
		return Frame{}, ErrNonFinite
	}
	un, ok := u.Normalize()
	if !ok {
		return Frame{}, ErrDegenerateFrame
	}
	// A non-finite AXIS is ErrDegenerateFrame, not ErrNonFinite — the shipped
	// contract, and what Normalize above already reports for u. Checked here rather
	// than left to fall out of the arithmetic, so it cannot depend on which of NaN's
	// many products happens to arise below.
	if !v.isFinite() {
		return Frame{}, ErrDegenerateFrame
	}
	// c = v × un, then un × c = un × (v × un) = v − un·(v·un): the Gram–Schmidt
	// perpendicular, with the sign the operand order of the inner cross fixes. Each
	// cross is filtered (collinear axes must give an exactly zero cross product, not
	// a residue) and each is redone with its operand quartered if it overflowed —
	// which only happens when the true result is past MaxFloat64, where the quarter
	// can cost nothing that matters to a direction.
	c := v.crossFiltered(un)
	if !c.isFinite() {
		c = v.Scale(0.25).crossFiltered(un)
	}
	vp := un.crossFiltered(c)
	if !vp.isFinite() {
		vp = un.crossFiltered(c.Scale(0.25))
	}
	// The perpendicular is taken for its DIRECTION, so it is normalized scale-free:
	// its length is legitimately 1e-20 for v = (MaxFloat64, 1e-20, 0), and
	// Normalize's zeroLen floor — a divide-by-zero guard sized for vectors of order
	// one — would call that direction "degenerate" merely for being small. Zero (u
	// and v collinear, or v itself zero) has no direction at all, and that direction
	// does reject.
	vn, ok := vp.direction()
	if !ok {
		return Frame{}, ErrDegenerateFrame
	}
	// ASK whether the perpendicular is perpendicular, at the same tolerance
	// [Frame.IsValid] will hold it to: a frame NewFrame returns always passes
	// IsValid. Phrased positively, so a NaN that got this far fails it.
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
