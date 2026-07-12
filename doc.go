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
// It depends on the standard library and on github.com/lestrrat-3d/units, which
// is itself stdlib-only. Angles are typed: see [Rotation].
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
// discouraged — no constructor can produce one, because the fallible
// constructors validate their input and return an error. That is what keeps
// [Transform.Inverse] exact: it is the transpose, never a solve. It is also why
// a normal transforms exactly like a direction ([Transform.ApplyDir]), with no
// inverse transpose anywhere in the package.
//
// [Vec.Normalize] returns a boolean rather than fabricating a unit vector from
// a zero vector; it is a divide-by-zero guard, not a geometric tolerance. The
// same refusal to invent geometry runs through the package: [Rotation] rejects a
// zero axis instead of picking one, and rejects an angle that is not an angle —
// including the zero units.Value, so a forgotten field cannot pass for a
// deliberate 0°.
package r3
