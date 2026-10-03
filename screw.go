package r3

import (
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/units"
)

// ErrImproper is returned by [Transform.Screw] when the transform is a reflection
// (determinant −1). A reflection is an isometry but not a motion: no screw — no
// rotation about any line, with any slide along it — produces it.
var ErrImproper = errors.New("r3: improper transform (a reflection has no screw motion)")

// Screw is a rigid motion written as a rotation by Angle about the line through
// Point along Axis, followed by a translation of Slide along Axis. Every proper
// [Transform] is one (Chasles).
//
// Like [Basis] it is a plain value with exported fields. The invariants it
// carries — a unit Axis, a Point on the axis — are what [Transform.Screw]
// produces, not what the type enforces: [Screw.At] takes Axis for its direction
// only and projects Point onto the axis, so a caller-built Screw is as good as a
// read one. The zero value Screw{} has no axis direction, and At rejects it with
// [ErrDegenerateAxis].
type Screw struct {
	Axis  Vec         // unit direction of the screw axis
	Point Vec         // a point on the axis
	Angle units.Value // rotation about Axis, right-handed, in radians
	Slide float64     // translation along Axis, signed, in the coordinate unit (millimetres)
}

// canonicalZero rewrites −0 as +0, so two results that differ only in the sign
// of a zero print the same.
func canonicalZero(x float64) float64 {
	if x == 0 {
		return 0
	}
	return x
}

func canonicalVec(v Vec) Vec {
	return Vec{X: canonicalZero(v.X), Y: canonicalZero(v.Y), Z: canonicalZero(v.Z)}
}

// Screw returns t written as a screw motion: a rotation by Angle about the line
// through Point along Axis, then a slide of Slide along Axis.
//
// Every output is a deterministic function of the input bits:
//
//   - Angle is in [0, π], never negative. A rotation by −θ about n is reported as
//     θ about −n. The angle is read from the matrix, so a full turn built by
//     Rotation(axis, 2π) is reported as the near-zero rotation it is stored as.
//   - For Angle in (0, π) the sign of Axis is forced by right-handedness. At
//     exactly π both signs are the same rotation: the sign comes from the
//     antisymmetric part of the linear part when that is nonzero, and otherwise
//     the first nonzero component of Axis, in X, Y, Z order, is positive.
//   - With Angle 0 and a nonzero translation, Axis is the unit direction of the
//     translation and Slide is its length. The identity has Axis (0, 0, 1),
//     Point (0, 0, 0), Angle 0 and Slide 0.
//   - Point is the point of the axis line closest to the origin, and the origin
//     itself when Angle is 0.
//   - Slide is signed: t·Axis.
//   - Every component of Axis and Point, and Slide, is written with −0 as +0.
//
// Smallness is not degeneracy: a rotation of 1e-13 radians is a rotation, and a
// tiny rotation with a sideways translation has a far-away Point, which is the
// true geometry of that input.
//
// It returns [ErrNonFinite] when t's translation is not finite, and
// [ErrNotOrthonormal] when its linear part is not a valid orthonormal basis (the
// zero value Transform{} included). It returns [ErrImproper] when t is a
// reflection. It returns [ErrNonFinite] again when the slide or the axis point is
// not representable as a float64.
func (t Transform) Screw() (Screw, error) {
	if err := t.validate(); err != nil {
		return Screw{}, err
	}
	if t.IsReflection() {
		return Screw{}, ErrImproper
	}

	// R − Rᵀ as a vector: a = 2·sinθ·n.
	a := Vec{X: t.ey.Z - t.ez.Y, Y: t.ez.X - t.ex.Z, Z: t.ex.Y - t.ey.X}
	trace := t.ex.X + t.ey.Y + t.ez.Z
	cosTheta := (trace - 1) / 2
	sinTheta := a.Len() / 2
	// Atan2, not Acos: accurate at both ends of [0, π], and it accepts the
	// slightly out-of-range arguments a within-tolerance input can produce.
	theta := math.Atan2(sinTheta, cosTheta)

	var axis Vec
	switch {
	case a == (Vec{}) && cosTheta > 0:
		// Regime A: the identity rotation. The axis is the translation's direction.
		if d, ok := t.t.direction(); ok {
			axis = d
		} else {
			axis = Vec{Z: 1}
		}
	case cosTheta >= 0:
		// Regime B: the antisymmetric part. direction, not Normalize, whose
		// zeroLen floor would refuse every rotation below ~5e-13 radians.
		d, ok := a.direction()
		if !ok {
			return Screw{}, ErrNotOrthonormal
		}
		axis = d
	default:
		// Regime C: the symmetric part, accurate exactly where a is not.
		axis = symmetricAxis(t, trace, a)
	}

	slide := t.t.dotUnit(axis)
	if !isFinite(slide) {
		return Screw{}, ErrNonFinite
	}

	var point Vec
	if theta != 0 {
		tPerp := t.t.Sub(axis.Scale(slide))
		sh, ch := math.Sincos(theta / 2)
		point = tPerp.Scale(0.5).Add(axis.Cross(tPerp).Scale(ch / sh / 2))
		if !point.isFinite() {
			return Screw{}, ErrNonFinite
		}
	}

	return Screw{
		Axis:  canonicalVec(axis),
		Point: canonicalVec(point),
		Angle: units.Radians(theta),
		Slide: canonicalZero(slide),
	}, nil
}

// symmetricAxis returns the regime-C axis: the longest column of
// S = R + Rᵀ − (trace − 1)·I = 2(1 − cosθ)·n·nᵀ, normalized and signed by a
// (the antisymmetric part, 2·sinθ·n).
func symmetricAxis(t Transform, trace float64, a Vec) Vec {
	tm1 := trace - 1
	dx, dy, dz := 2*t.ex.X-tm1, 2*t.ey.Y-tm1, 2*t.ez.Z-tm1
	var col Vec
	switch {
	case dx >= dy && dx >= dz:
		col = Vec{X: dx, Y: t.ex.Y + t.ey.X, Z: t.ex.Z + t.ez.X}
	case dy >= dz:
		col = Vec{X: t.ey.X + t.ex.Y, Y: dy, Z: t.ey.Z + t.ez.Y}
	default:
		col = Vec{X: t.ez.X + t.ex.Z, Y: t.ez.Y + t.ey.Z, Z: dz}
	}
	// The longest column has length > 1.15 in this regime, so it has a direction.
	cand, _ := col.direction()
	switch d := a.Dot(cand); {
	case d > 0:
		return cand
	case d < 0:
		return cand.Scale(-1)
	}
	// a·cand == 0: a is exactly zero and θ is exactly π. Both signs are the same
	// rotation; make the first nonzero component positive.
	for _, c := range [3]float64{cand.X, cand.Y, cand.Z} {
		if c > 0 {
			return cand
		}
		if c < 0 {
			return cand.Scale(-1)
		}
	}
	return cand
}

// At returns the transform that rotates by frac·Angle about the axis and slides
// frac·Slide along it. At(0) is [Identity]; At(1) is the whole motion. frac is
// any finite float64: values outside [0, 1] continue the same screw, so At(2) is
// the motion applied twice and At(−1) its inverse.
//
// Only the direction of Axis is used, so a non-unit Axis names the same screw,
// and any point on the axis line names the same screw as the closest one.
//
// The translation is computed as (1 − cosφ)·c − sinφ·(n × c) + frac·Slide·n, with
// 1 − cosφ written 2·sin²(φ/2) and c the point's offset from the axis through the
// origin, so a far-away Point never forms a large cancelling difference.
//
// It returns [ErrDegenerateAxis] when Axis has no direction (the zero Screw
// included), wraps [units.ErrIncompatible] when Angle does not measure an angle,
// and returns [ErrNonFinite] for a NaN or infinite frac, Angle, Point or Slide.
// The Transform it produces is validated, so [ErrNonFinite] or
// [ErrNotOrthonormal] is also returned if the result is not a rigid motion.
func (s Screw) At(frac float64) (Transform, error) {
	if !isFinite(frac) {
		return Transform{}, ErrNonFinite
	}
	n, ok := s.Axis.direction()
	if !ok {
		return Transform{}, ErrDegenerateAxis
	}
	theta, err := s.Angle.In(units.Radian)
	if err != nil {
		if errors.Is(err, units.ErrNotFinite) {
			return Transform{}, ErrNonFinite
		}
		return Transform{}, fmt.Errorf("r3: screw angle: %w", err)
	}
	if !isFinite(theta) {
		return Transform{}, ErrNonFinite
	}
	if !s.Point.isFinite() || !isFinite(s.Slide) {
		return Transform{}, ErrNonFinite
	}

	phi := frac * theta
	sin, cos := math.Sincos(phi)
	h := math.Sin(phi / 2)
	cperp := s.Point.Sub(n.Scale(s.Point.dotUnit(n)))
	out := Transform{
		ex: rodrigues(Vec{X: 1}, n, sin, cos),
		ey: rodrigues(Vec{Y: 1}, n, sin, cos),
		ez: rodrigues(Vec{Z: 1}, n, sin, cos),
		t: cperp.Scale(2 * h * h).
			Sub(n.Cross(cperp).Scale(sin)).
			Add(n.Scale(frac * s.Slide)),
	}
	if err := out.validate(); err != nil {
		return Transform{}, err
	}
	return out, nil
}

// Interpolate returns the pose a fraction frac of the way from t to to along the
// screw motion joining them. Interpolate(to, 0) is t and Interpolate(to, 1) is
// to.
//
// The relative motion t⁻¹ then to is decomposed with [Transform.Screw] and
// evaluated with [Screw.At], so the path is the shortest rotation (the angle is
// in [0, π]) with a uniform slide. When the two orientations are exactly π apart
// both arcs are equally short, and the sign of the screw axis picks one
// deterministically; the pose at frac = 1 is to either way.
//
// It returns the errors of [Transform.Inverse], [Transform.Then],
// [Transform.Screw] and [Screw.At]. In particular it returns [ErrImproper] when
// exactly one of t and to is a reflection: no continuous rigid path joins the two
// handedness classes. Two reflections interpolate.
func (t Transform) Interpolate(to Transform, frac float64) (Transform, error) {
	inv, err := t.Inverse()
	if err != nil {
		return Transform{}, err
	}
	rel, err := inv.Then(to)
	if err != nil {
		return Transform{}, err
	}
	sc, err := rel.Screw()
	if err != nil {
		return Transform{}, err
	}
	step, err := sc.At(frac)
	if err != nil {
		return Transform{}, err
	}
	return t.Then(step)
}
