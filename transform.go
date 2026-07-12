package r3

import (
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/units"
)

// ErrDegenerateAxis is returned by [Rotation] and [RotationAround] when the
// rotation axis is the zero vector: there is no rotation about a zero axis, and
// inventing one would fabricate a direction that was never given.
var ErrDegenerateAxis = errors.New("r3: degenerate rotation axis (zero vector)")

// ErrNotOrthonormal is returned by [FromBasis] when the basis vectors are not
// unit length and mutually orthogonal, so they describe a scale, a shear or a
// collapse rather than a rigid motion.
var ErrNotOrthonormal = errors.New("r3: basis is not orthonormal")

// Basis is the linear part of a [Transform]: the images of the world basis
// vectors X, Y and Z. It is how a rotation is read out of a Transform (see
// [Transform.Basis]) and fed back in ([FromBasis]) — for persistence, or to hand
// a placement to a geometry kernel, an exporter or a renderer.
//
// It is a plain value with no operations of its own. The linear part is natively
// three vectors, so it is exposed as three vectors: naming the axes rules out
// the row/column mix-up that indexing a 3x3 array invites.
type Basis struct {
	EX, EY, EZ Vec
}

// Transform is a rigid motion of ℝ³: an orthogonal linear map followed by a
// translation. It preserves distances and angles. Scale, shear and projection
// are NOT representable — if you need them, they belong in a separate affine
// type, not here: admitting scale would cost [Transform.Inverse] its exactness
// and make normals require the inverse transpose.
//
// Both determinants are allowed. det = +1 is a proper rigid motion (a rotation
// and/or translation); det = −1 is a reflection, which is still an isometry.
// [Transform.IsReflection] reports which, because a consumer that cares about
// orientation — a CAD kernel does; a reflected solid has inverted face normals —
// must be able to ask.
//
// Transform is an immutable value: every method returns a new Transform and
// none mutates the receiver.
//
// The zero value Transform{} is invalid, as reported by [Transform.IsValid].
// Build one with [Identity], [Translation], [Rotation], [RotationAround],
// [Reflection], [FromFrame] or [FromBasis], and derive others from it with
// [Transform.Then] and [Transform.Inverse]. Those are the only ways to obtain
// one, and none of them can produce a non-isometry: every one of them but
// [Identity] is fallible, and each validates what it PRODUCES — not merely what
// it consumes — returning [ErrNonFinite] rather than a Transform that is not a
// rigid motion. Composition is included on purpose: two valid transforms can
// compose to an overflowing translation, and a method that could hand that back
// would sink the invariant just as surely as a careless constructor. The
// invariant is enforced, not merely documented.
type Transform struct {
	ex, ey, ez Vec // images of the basis vectors — the linear part, by columns
	t          Vec // translation
}

// Identity returns the transform that leaves every point where it is.
func Identity() Transform {
	return Transform{
		ex: Vec{X: 1},
		ey: Vec{Y: 1},
		ez: Vec{Z: 1},
	}
}

// Translation returns the transform that displaces every point by v and leaves
// every direction untouched.
//
// It returns [ErrNonFinite] when v has a NaN or infinite component. A
// displacement by NaN is not a displacement, and the resulting Transform would
// send every point to NaN while claiming to be a rigid motion — so the error is
// the price of the invariant. [Identity] takes no input and therefore cannot
// fail; it stays infallible.
func Translation(v Vec) (Transform, error) {
	if !v.isFinite() {
		return Transform{}, ErrNonFinite
	}
	t := Identity()
	t.t = v
	return t, nil
}

// Rotation returns a right-handed rotation of angle about axis, through the
// origin. The axis need not be unit length; it is normalized.
//
// It returns [ErrDegenerateAxis] when axis is the zero vector, and wraps
// [units.ErrIncompatible] when angle does not measure an angle — which rejects a
// length, a bare scalar, and the zero units.Value (whose kind is dimensionless,
// not angle) alike. A forgotten angle is therefore an error rather than a silent
// identity rotation.
//
// It returns [ErrNonFinite] when angle is an angle of NaN or infinite magnitude.
// The unit KIND being right is not enough: math.Sincos of either returns NaN, so
// the basis would come out NaN with a nil error beside it. There is no rotation
// by an infinite amount and none by NaN, so there is nothing to return but an
// error.
//
// To rotate about an axis that does not pass through the origin, use
// [RotationAround].
func Rotation(axis Vec, angle units.Value) (Transform, error) {
	n, ok := axis.Normalize()
	if !ok {
		return Transform{}, ErrDegenerateAxis
	}
	rad, err := angle.In(units.Radian)
	if err != nil {
		return Transform{}, fmt.Errorf("r3: rotation angle: %w", err)
	}
	if !isFinite(rad) {
		return Transform{}, ErrNonFinite
	}
	sin, cos := math.Sincos(rad)
	return Transform{
		ex: rodrigues(Vec{X: 1}, n, sin, cos),
		ey: rodrigues(Vec{Y: 1}, n, sin, cos),
		ez: rodrigues(Vec{Z: 1}, n, sin, cos),
	}, nil
}

// rodrigues rotates v about the unit axis n by the angle whose sine and cosine
// are sin and cos.
func rodrigues(v, n Vec, sin, cos float64) Vec {
	return v.Scale(cos).
		Add(n.Cross(v).Scale(sin)).
		Add(n.Scale(n.Dot(v) * (1 - cos)))
}

// RotationAround returns a right-handed rotation of angle about the axis through
// center along axis. The center is a fixed point of the result.
//
// It returns the same errors as [Rotation], plus [ErrNonFinite] when center has
// a NaN or infinite component — a rotation about a pivot that is nowhere is not
// a rotation — and, like [Reflection], when the translation it COMPUTES is not
// finite even though every input was. The composed offset is center − R·center,
// so a pivot far enough out (near MaxFloat64) overflows to infinity. What is
// validated is the RESULT; the check rides on [Transform.Then], which performs
// it for every composition.
func RotationAround(center, axis Vec, angle units.Value) (Transform, error) {
	rot, err := Rotation(axis, angle)
	if err != nil {
		return Transform{}, err
	}
	// Move center to the origin, rotate there, move it back.
	there, err := Translation(center.Scale(-1))
	if err != nil {
		return Transform{}, err
	}
	back, err := Translation(center)
	if err != nil {
		return Transform{}, err
	}
	spun, err := there.Then(rot)
	if err != nil {
		return Transform{}, err
	}
	return spun.Then(back)
}

// Reflection returns the reflection across the plane of mirror — the plane
// through the frame's origin spanned by its U and V axes. The result is
// improper: its determinant is −1 and [Transform.IsReflection] reports true.
//
// It returns [ErrDegenerateFrame] when mirror is not a valid frame, because a
// zero frame has no plane to reflect across and would yield a Transform that is
// not an isometry at all.
//
// It returns [ErrNonFinite] when mirror's origin is not finite, and — the case
// input validation alone would never have caught — when the translation it
// COMPUTES is not finite even though every input was. The offset is 2(origin·n)n,
// so a plane a finite but enormous distance along its own normal (past
// MaxFloat64/2) doubles to +Inf. What is validated here is therefore the RESULT:
// a Transform that exists is a real rigid motion, with no asterisk.
func Reflection(mirror Frame) (Transform, error) {
	// Checked ahead of IsValid — which would also reject it — so that a plane
	// anchored nowhere is reported as the non-finite position it is, and not as a
	// complaint about axes that were never at fault. FromFrame orders it the same
	// way.
	if !mirror.Origin().isFinite() {
		return Transform{}, ErrNonFinite
	}
	if !mirror.IsValid() {
		return Transform{}, ErrDegenerateFrame
	}
	n := mirror.N()
	// Offset the plane from the origin: a point at distance d along n lands at
	// −d, so the whole reflection shifts by 2(origin·n)n.
	t := n.Scale(2 * mirror.Origin().Dot(n))
	if !t.isFinite() {
		return Transform{}, ErrNonFinite
	}
	return Transform{
		ex: householder(Vec{X: 1}, n),
		ey: householder(Vec{Y: 1}, n),
		ez: householder(Vec{Z: 1}, n),
		t:  t,
	}, nil
}

// householder reflects v across the plane through the origin with unit normal n.
func householder(v, n Vec) Vec {
	return v.Sub(n.Scale(2 * v.Dot(n)))
}

// FromFrame returns the transform that maps frame-local coordinates to world
// coordinates: FromFrame(f).Apply(local) is f.ToWorld(local), and its inverse is
// f.ToLocal. It is proper — a Frame is right-handed, so the determinant is +1.
//
// It returns [ErrDegenerateFrame] when f's axes are not orthonormal, and
// [ErrNonFinite] when f's origin is not finite — the origin becomes the
// transform's translation verbatim, so it is checked first and reported for what
// it is rather than being folded into a complaint about the axes.
//
// To move a body from one frame to another, compose:
//
//	from, err := r3.FromFrame(a) // handle err
//	to, err := r3.FromFrame(b)   // handle err
//	out, err := from.Inverse()   // handle err
//	place, err := out.Then(to)   // handle err
//
// which reads as "express the point in a's local coordinates, then plant those
// coordinates in b". [Transform.Inverse] and [Transform.Then] are fallible for
// the same reason the constructors are: the composed translation can overflow
// even when both operands are impeccable.
func FromFrame(f Frame) (Transform, error) {
	if !f.Origin().isFinite() {
		return Transform{}, ErrNonFinite
	}
	if !f.IsValid() {
		return Transform{}, ErrDegenerateFrame
	}
	return Transform{ex: f.U(), ey: f.V(), ez: f.N(), t: f.Origin()}, nil
}

// FromBasis rebuilds a transform from a linear part and a translation — the
// inverse of reading [Transform.Basis] and [Transform.Translation] out. It is
// the way a stored transform comes back from disk.
//
// It returns [ErrNotOrthonormal] unless b's vectors are unit length and mutually
// orthogonal, and [ErrNonFinite] when t is not finite. Those checks are what stop
// FromBasis from being a back door: neither a scale, nor a shear, nor a NaN
// position can enter the type through it. The translation is checked first, so a
// bad t is reported as the non-finite value it is instead of as a complaint about
// a basis that was perfectly fine.
func FromBasis(b Basis, t Vec) (Transform, error) {
	if !t.isFinite() {
		return Transform{}, ErrNonFinite
	}
	out := Transform{ex: b.EX, ey: b.EY, ez: b.EZ, t: t}
	if !out.IsValid() {
		return Transform{}, ErrNotOrthonormal
	}
	return out, nil
}

// Apply maps a point: the linear part, then the translation.
//
// Use [Transform.ApplyDir] for a direction or a normal. The distinction is not
// sugar — applying Apply to a normal translates it, which is silently wrong.
func (t Transform) Apply(p Vec) Vec {
	return t.ApplyDir(p).Add(t.t)
}

// ApplyDir maps a direction: the linear part only, with no translation. A
// direction has no position, so it does not move when space does.
//
// A normal transforms exactly like a direction here — no inverse transpose is
// needed. That is a dividend of excluding scale: under a general affine map a
// normal needs the inverse transpose, and everyone forgets. Here it cannot be
// got wrong.
func (t Transform) ApplyDir(d Vec) Vec {
	return t.ex.Scale(d.X).
		Add(t.ey.Scale(d.Y)).
		Add(t.ez.Scale(d.Z))
}

// Then composes: it returns the transform that applies t first and next second,
// so a.Then(b) applied to p equals b.Apply(a.Apply(p)).
//
// Read it left to right, in application order. Note that this is the REVERSE of
// matrix notation, where the same composition is written B·A — which is exactly
// why the method is named Then and not Mul.
//
// It returns [ErrNonFinite] when the composed translation overflows, which two
// individually valid transforms can do: translating by MaxFloat64 along X, then
// again by MaxFloat64 along X, lands at +Inf. That is why Then is fallible.
// Composition is where the invariant would otherwise leak — a constructor that
// validates what it produces buys nothing if a method can then produce an
// unvalidated Transform out of two valid ones. The linear part cannot overflow
// (an orthonormal basis mapped by an orthonormal basis stays bounded by 1), so
// the translation is the whole of the check.
func (t Transform) Then(next Transform) (Transform, error) {
	out := Transform{
		ex: next.ApplyDir(t.ex),
		ey: next.ApplyDir(t.ey),
		ez: next.ApplyDir(t.ez),
		t:  next.Apply(t.t),
	}
	if !out.t.isFinite() {
		return Transform{}, ErrNonFinite
	}
	return out, nil
}

// Inverse returns the transform that undoes t.
//
// It is exact and cheap: the linear part is orthogonal, so its inverse is its
// transpose — three dot products, never a matrix solve. This is the same
// property [Frame.ToLocal] relies on, and it is the reason scale is excluded
// from the type.
//
// It returns [ErrNonFinite] when the inverted translation overflows. Exactness
// is not the same as safety: the inverse translation is −Lᵀ·t, and each of those
// three dot products sums three products, so a huge-but-finite translation in a
// rotated basis can carry a component past MaxFloat64 even though t itself was
// perfectly valid. The result is validated, like everything else in the package.
func (t Transform) Inverse() (Transform, error) {
	// Transpose: the rows of the linear part become its columns.
	inv := Transform{
		ex: Vec{X: t.ex.X, Y: t.ey.X, Z: t.ez.X},
		ey: Vec{X: t.ex.Y, Y: t.ey.Y, Z: t.ez.Y},
		ez: Vec{X: t.ex.Z, Y: t.ey.Z, Z: t.ez.Z},
	}
	// Undo the translation in the un-rotated frame: −Lᵀ·t.
	inv.t = inv.ApplyDir(t.t).Scale(-1)
	if !inv.t.isFinite() {
		return Transform{}, ErrNonFinite
	}
	return inv, nil
}

// IsValid reports whether the linear part is orthonormal AND the translation is
// finite, i.e. whether t is a rigid motion. The zero value Transform{} is not
// valid. Use it to vet a transform supplied by a caller before building geometry
// on it.
//
// The translation counts. An orthonormal basis paired with a NaN translation
// still sends every point to NaN: it is a rigid motion of nothing. Checking only
// the linear part would call that valid.
//
// A NaN or infinite component makes t invalid. Every check is phrased
// positively (!(x <= tol) rather than x > tol) so that a comparison against NaN
// — which is false whichever way it is written — rejects rather than admits.
func (t Transform) IsValid() bool {
	if !t.t.isFinite() {
		return false
	}
	for _, v := range []Vec{t.ex, t.ey, t.ez} {
		if !(math.Abs(v.Len()-1) <= orthoTol) {
			return false
		}
	}
	if !(math.Abs(t.ex.Dot(t.ey)) <= orthoTol) {
		return false
	}
	if !(math.Abs(t.ey.Dot(t.ez)) <= orthoTol) {
		return false
	}
	return math.Abs(t.ez.Dot(t.ex)) <= orthoTol
}

// IsReflection reports whether t reverses orientation, i.e. whether its
// determinant is −1. A consumer that cares about handedness must ask: a
// reflected solid has inverted face normals, and a right-handed [Frame] cannot
// survive a reflection unchanged.
func (t Transform) IsReflection() bool {
	return t.ex.Cross(t.ey).Dot(t.ez) < 0
}

// Translation returns the displacement part of t.
func (t Transform) Translation() Vec { return t.t }

// Basis returns the linear part of t: the images of the world basis vectors.
// Together with [Transform.Translation] it is everything needed to reconstruct t
// via [FromBasis], or to hand the placement to a kernel or an exporter.
func (t Transform) Basis() Basis {
	return Basis{EX: t.ex, EY: t.ey, EZ: t.ez}
}

// Equal reports whether t and o agree within tol, comparing the linear part and
// the translation componentwise.
//
// tol is a comparison tolerance chosen by the caller. It is unrelated to the
// fixed threshold [Transform.IsValid] uses to police orthonormality.
func (t Transform) Equal(o Transform, tol float64) bool {
	return t.ex.Equal(o.ex, tol) &&
		t.ey.Equal(o.ey, tol) &&
		t.ez.Equal(o.ez, tol) &&
		t.t.Equal(o.t, tol)
}
