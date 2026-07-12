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
// both are normalized). It returns [ErrDegenerateFrame] when u is zero or when
// v is collinear with u (the perpendicular component vanishes).
//
// It returns [ErrNonFinite] when origin has a NaN or infinite component: the
// origin is stored as given, not normalized, so nothing downstream would ever
// catch it — and a frame with a NaN origin poisons every point that passes
// through [Frame.ToWorld] or [FromFrame]. That case is [ErrNonFinite] and not
// [ErrDegenerateFrame] because the axes are fine; it is the position that is
// not a position. ErrDegenerateFrame stays reserved for what it says: zero or
// collinear AXES.
//
// Axis MAGNITUDE is not a reason for refusal, however large. The projection is
// taken on v scaled down by a power of two, so that a v whose own dot product
// would overflow — u = (MaxFloat64, MaxFloat64, MaxFloat64) with
// v = (MaxFloat64, MaxFloat64, −MaxFloat64), two finite axes that are plainly not
// collinear — still yields the perpendicular it has. The naive projection
// overflows on the way to a result that cancels back down to an ordinary number,
// and NewFrame then reported "degenerate axes" about axes that were nothing of
// the kind. The scaling is exact and the direction of the perpendicular is
// scale-invariant, so nothing is paid for it.
func NewFrame(origin, u, v Vec) (Frame, error) {
	if !origin.isFinite() {
		return Frame{}, ErrNonFinite
	}
	un, ok := u.Normalize()
	if !ok {
		return Frame{}, ErrDegenerateFrame
	}
	// Scale v into the binade below 1 first: v·un can overflow to ±Inf while the
	// perpendicular it feeds is perfectly finite, and an overflow here is
	// indistinguishable from a vanishing perpendicular — a false "degenerate".
	// Scaling by a power of two is bit-exact, and it never needs undoing: the
	// perpendicular is normalized, and direction does not care about scale. A
	// zero, NaN or infinite v has no scaling and no direction either, which is the
	// degenerate case Normalize would have caught anyway.
	vs, _, ok := v.scaledDown()
	if !ok {
		return Frame{}, ErrDegenerateFrame
	}
	// Remove the u-component of v, leaving the in-plane perpendicular.
	vp := vs.Sub(un.Scale(vs.Dot(un)))
	vn, ok := vp.Normalize()
	if !ok {
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
func (f Frame) ToWorld(local Vec) Vec {
	return f.origin.
		Add(f.u.Scale(local.X)).
		Add(f.v.Scale(local.Y)).
		Add(f.N().Scale(local.Z))
}

// ToWorldUV maps an in-plane 2D point (u, v) — the currency of a planar sketch
// — to world space (w = 0).
func (f Frame) ToWorldUV(u, v float64) Vec {
	return f.origin.Add(f.u.Scale(u)).Add(f.v.Scale(v))
}

// ToLocal maps a world point to local coordinates. The third component is the
// signed distance off the plane (along N). It is the exact inverse of ToWorld
// (the transpose), valid because the frame is orthonormal.
func (f Frame) ToLocal(world Vec) Vec {
	d := world.Sub(f.origin)
	return Vec{d.Dot(f.u), d.Dot(f.v), d.Dot(f.N())}
}
