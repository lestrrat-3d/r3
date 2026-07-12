// Package r3 is a self-contained coordinate-math layer for Euclidean 3-space
// (ℝ³): a [Vec] vector type, an orthonormal right-handed [Frame] that carries
// the bidirectional transform between a plane's local (u, v, w) coordinates and
// world (x, y, z), and a [Transform] that is a rigid motion of the space itself.
//
// # Scope
//
// The package holds what *lives in* 3-space and what *acts on* it — vectors,
// frames, and the transforms between them — and nothing else. It carries no
// document state and knows nothing of any application layered on top of it.
// 3D *shapes* (spheres, boxes, surfaces, solids) are deliberately out of scope:
// they belong to a geometry layer above, which imports this one for its
// coordinates.
//
// It depends on the standard library and on github.com/lestrrat-3d/units, whose
// package code in turn imports only the standard library — so nothing outside
// stdlib enters a build of this package. (units requires testify to run its own
// tests; that is a test-only dependency and never reaches a build.) Angles are
// typed: see [Rotation].
//
// # Invariants
//
// A [Frame] is ALWAYS orthonormal and right-handed. The only way to obtain one
// is [NewFrame], which orthonormalizes its axes and rejects degenerate input;
// the zero value Frame{} is invalid and is reported as such by [Frame.IsValid]
// so callers that accept a frame from outside can reject it.
// Because the axes are orthonormal, the inverse transform [Frame.ToLocal] is
// three dot products (the transpose), never a matrix solve.
//
// A [Transform] is ALWAYS an isometry: distances and angles survive it, and
// scale, shear and projection are unrepresentable rather than merely
// discouraged. Nothing in the package can produce a non-isometry, because every
// operation that yields a Transform — the constructors [Translation], [Rotation],
// [RotationAround], [Reflection], [FromFrame], [FromBasis], and the derivations
// [Transform.Then] and [Transform.Inverse] — validates what it produces, not
// merely what it consumes, and returns an error otherwise. [Identity] is the only
// infallible one, because it takes no input and has nothing to get wrong. That is
// what keeps [Transform.Inverse] exact: it is the transpose, never a solve. It is
// also why a normal transforms exactly like a direction ([Transform.ApplyDir]),
// with no inverse transpose anywhere in the package.
//
// Nothing non-finite may enter either type. A NaN or infinite angle, position,
// origin or translation is rejected with [ErrNonFinite] — and so is a
// translation that OVERFLOWS to infinity while every input was individually
// finite. That is not a corner case of one constructor: [Reflection] does it for
// a mirror plane far enough from the origin, [RotationAround] for a pivot far
// enough out, and [Transform.Then] and [Transform.Inverse] for the composition
// or the inverse of transforms that are each themselves valid. All of them are
// fallible, so all of them can say so. A Transform that exists is a real rigid
// motion, with no asterisk.
//
// Bigness alone is not a fault, and the package goes out of its way not to treat
// it as one: a vector whose length overflows still has a direction, and the cold
// paths — [NewFrame]'s orthonormalization, [Reflection]'s plane offset — do their
// arithmetic scaled, so an axis or a mirror plane out at MaxFloat64 is built
// rather than refused. There is ONE accepted exception, on the hot path.
// [Transform.ApplyDir] sums its three terms in a fixed order, so an intermediate
// sum can overflow where the final value would not: transforms of points whose
// coordinates approach MaxFloat64 may therefore be CONSERVATIVELY REJECTED with
// [ErrNonFinite] — by [Transform.Then] or [Transform.Inverse] — even though the
// exact result is representable. ApplyDir runs once per transformed point and
// making it overflow-safe would tax every point transform forever to serve
// coordinates that cannot exist (this library's unit is the millimetre; 1e308 mm
// is some 1e289 light-years). The failure is one-sided: an error, never a wrong
// answer, so the isometry invariant stands. See [Transform.ApplyDir].
//
// The price is that composing is fallible:
//
//	spin, err := r3.RotationAround(pivot, axis, units.Degrees(90)) // handle err
//	lift, err := r3.Translation(r3.NewVec(0, 0, 5))                // handle err
//	place, err := spin.Then(lift)                                  // handle err
//	back, err := place.Inverse()                                   // handle err
//
// which is the honest bill for the invariant: an err that is always nil in
// ordinary use is a small tax, and a silently infinite placement is not.
//
// [Vec.Normalize] returns a boolean rather than fabricating a unit vector from
// a zero vector; it is a divide-by-zero guard, not a geometric tolerance. But it
// is a guard against zero, NOT against bigness: a vector whose length overflows,
// such as (MaxFloat64, MaxFloat64, 0), still has a perfectly good direction, and
// Normalize returns it. The same refusal to invent geometry — and the same
// refusal to condemn geometry that is merely large — runs through the package:
// [Rotation] rejects a zero axis instead of picking one, and rejects an angle
// that is not an angle — including the zero units.Value, so a forgotten field
// cannot pass for a deliberate 0°.
package r3
