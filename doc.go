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
// github.com/lestrrat-3d/units is the ONLY non-stdlib dependency a build of this
// package pulls in (it is what `go list -deps .` reports beyond the standard
// library), and it is a shallow one: the package code of units imports nothing
// but the standard library itself, so the build graph stops there. (units
// requires testify to run its own tests; that is a test-only dependency of units
// and never reaches a build.) Angles are typed: see [Rotation].
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
// infallible one, because it takes no input and has nothing to get wrong. It is
// why a normal transforms exactly like a direction ([Transform.ApplyDir]), with no
// inverse transpose anywhere in the package.
//
// [Transform.Inverse] is EXACT — the transpose, never a solve — and that is a
// claim about the arithmetic, not just the algebra. The transpose inverts a TRULY
// orthonormal basis and nothing else: for a basis skewed by even 7e-10 it is an
// approximation whose round trip drifts by about 1e-8. So the linear part of every
// Transform is orthonormal to machine precision, not merely within the 1e-9
// tolerance that ADMITS one. [FromBasis], the one door a stored-and-reloaded basis
// comes in through, therefore orthonormalizes what it admits (Gram–Schmidt, as
// [NewFrame] does for a Frame's axes) instead of storing it verbatim. Admission is
// unchanged: a scale, a shear, a collapse are still rejected with
// [ErrNotOrthonormal] rather than silently corrected into a transform the caller
// never described. What is snapped straight is drift at the level of the tolerance
// itself. Handedness survives — an improper basis (det = −1) comes back improper,
// because all three vectors are orthonormalized in turn rather than the third being
// re-derived as EX × EY, which would quietly flip a reflection into a rotation.
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
// Bigness alone is not a fault, and neither is smallness beside it; the package
// goes out of its way to treat neither as one. A vector whose length overflows
// still has a direction, and the COLD paths — [NewFrame]'s orthonormalization,
// [Reflection]'s plane offset — do their arithmetic scaled, so an axis or a mirror
// plane out at MaxFloat64 is built rather than refused. The scaling is applied to
// the PRODUCTS of a dot product, never to the vector going into it: scaling the
// vector by its own largest component would flush a small-but-decisive component
// away, and a mirror plane at (MaxFloat64, 0, 1e-20) would then be reflected
// across as if it passed through the origin — silently, since nothing about the
// answer is infinite or NaN. Where a whole vector must be scaled down for headroom
// — [NewFrame]'s cross products, whose exact result can want a component past
// MaxFloat64 — it is done only when the unscaled arithmetic actually overflows,
// because scaling down unconditionally underflows a decisive denormal and calls
// THAT degenerate. For the same reason NewFrame judges collinearity on the axes AS
// GIVEN: normalizing an axis first rounds, and the tiny component it rounds away
// can be the whole evidence that two axes span no plane at all. These paths run
// once per frame or per feature, so the care costs nothing.
//
// The PER-POINT mappings are the accepted exception, and there are five of them:
// [Transform.Apply], [Transform.ApplyDir], [Frame.ToWorld], [Frame.ToWorldUV] and
// [Frame.ToLocal]. Each sums its terms in a fixed order, so an intermediate sum
// can reach ±Inf where the exact result is perfectly representable — with a basis
// row of (⅔, ⅔, −⅓), ⅔·Max + ⅔·Max is +Inf before the −⅓·Max that would have
// brought it back. These run once per transformed point and are the hottest code
// here; overflow-safe accumulation would tax every point forever to serve
// coordinates that cannot exist (this library's unit is the millimetre; 1e308 mm
// is some 1e289 light-years).
//
// Be precise about what that costs, because it is NOT uniform:
//
//   - A Transform is never silently wrong. [Transform.Then] and
//     [Transform.Inverse] are fallible: they catch the ±Inf and return
//     [ErrNonFinite], conservatively refusing a composition whose true value was
//     representable. The isometry invariant stands.
//   - The five mappings above are infallible — they have no error to return — so
//     called directly with coordinates near MaxFloat64 they hand back a Vec with a
//     non-finite component. That IS a wrong answer rather than an error, in
//     [Frame]'s world/local mapping exactly as much as in [Transform]'s. A caller
//     working at those magnitudes must check the returned Vec itself.
//
// See [Transform.ApplyDir].
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
