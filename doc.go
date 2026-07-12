// Package r3 is a self-contained coordinate-math layer for Euclidean 3-space
// (ℝ³): a [Vec] vector type and an orthonormal right-handed [Frame] that
// carries the bidirectional transform between a plane's local (u, v, w)
// coordinates and world (x, y, z).
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
// It depends only on the standard library.
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
// [Vec.Normalize] returns a boolean rather than fabricating a unit vector from
// a zero vector; it is a divide-by-zero guard, not a geometric tolerance.
package r3
