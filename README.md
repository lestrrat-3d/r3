# r3

Coordinate math for Euclidean 3-space (ℝ³) in Go: a `Vec` vector type, an
orthonormal right-handed `Frame` carrying the transform between a plane's local
`(u, v, w)` coordinates and world `(x, y, z)`, and a `Transform` — a rigid
motion of the space itself.

```go
import "github.com/lestrrat-3d/r3"

// The XZ datum: U = +X, V = +Z, so N = U × V = −Y.
f, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1))
if err != nil {
    return err // r3.ErrDegenerateFrame: zero or collinear axes
}

w := f.ToWorldUV(3, 4)  // a 2D point on the plane -> world
l := f.ToLocal(w)       // and back; exact, because the frame is orthonormal
```

A `Transform` moves things: place a body, pattern it, mirror it.

```go
import "github.com/lestrrat-3d/units"

// 30° about +Z, through (10, 0, 0). Angles are typed — no bare radians.
spin, err := r3.RotationAround(
    r3.NewVec(10, 0, 0), r3.NewVec(0, 0, 1), units.Degrees(30),
)
if err != nil {
    return err
}

lift, err := r3.Translation(r3.NewVec(0, 0, 5))
if err != nil {
    return err // r3.ErrNonFinite: a NaN or infinite displacement
}

// Compose left to right, in the order things happen. Fallible, because two
// valid transforms can compose to a translation that overflows to infinity.
place, err := spin.Then(lift)
if err != nil {
    return err // r3.ErrNonFinite
}

p := place.Apply(pt)      // a POINT: rotated, then moved
n := place.ApplyDir(nrm)  // a DIRECTION: rotated only — never translated

back, err := place.Inverse() // exact: the transpose, not a solve
if err != nil {
    return err // r3.ErrNonFinite
}
```

## Scope

The package holds what *lives in* 3-space and what *acts on* it — vectors,
frames, and the transforms between them. Nothing else. It carries no document
state, and depends only on the standard library and
[`lestrrat-3d/units`](https://github.com/lestrrat-3d/units), whose package code
in turn imports only the standard library — so nothing outside stdlib enters a
build of this package. (`units` requires testify to run its own tests; that is a
test-only dependency and never reaches a build.)

3D **shapes** (spheres, boxes, surfaces, solids) are deliberately **out of
scope**. They belong to a geometry layer above, which imports this one for its
coordinates. The name is the scope rule: if it lives in ℝ³, it belongs here; if
it *is* a shape, it does not.

## Invariants

- **A `Frame` is always orthonormal and right-handed.** The only constructor is
  `NewFrame`, which orthonormalizes with Gram–Schmidt and returns
  `ErrDegenerateFrame` on zero or collinear axes. The zero value `Frame{}` is
  invalid and says so via `IsValid()`, so a frame accepted from outside can be
  vetted before you build on it.
- **`N()` is derived, never stored** (`U × V`), so a frame cannot disagree with
  its own normal.
- **`ToLocal` is the transpose, not a matrix solve** — exact, because the axes
  are orthonormal.
- **A `Transform` is always an isometry.** Scale, shear and projection are
  **unrepresentable**, not merely discouraged: nothing in the package can build
  one. Every operation that yields a `Transform` but `Identity` is fallible — the
  constructors (`Translation`, `Rotation`, `RotationAround`, `Reflection`,
  `FromFrame`, `FromBasis`) *and* the derivations (`Then`, `Inverse`) — and each
  validates what it **produces**, not just what it consumes, rather than admit a
  non-isometry. That is what buys the next two properties.
- **Nothing non-finite gets in.** A NaN or infinite angle, position, origin or
  translation is rejected with `ErrNonFinite` — including a translation that
  *overflows* to infinity from inputs that were each individually finite: a
  `Reflection` across a plane far enough out, a `RotationAround` a pivot far
  enough out, or the `Then` of two transforms that are each perfectly valid. A
  `Transform` that exists is a real rigid motion, no asterisk. Composition being
  fallible is the bill for that; an always-nil `err` is a small tax, a silently
  infinite placement is not.
- **Bigness is not a fault — with one accepted exception.** Huge-but-finite input
  is *built*, not refused: `NewFrame` orthonormalizes axes out at `MaxFloat64`,
  and `Reflection` offsets a mirror plane that far out, by scaling their
  arithmetic. Those are cold paths. `ApplyDir` is not — it runs once per
  transformed point — so it sums its three terms in fixed order, and an
  intermediate sum can overflow where the final value would not. **Transforms of
  points whose coordinates approach `MaxFloat64` may therefore be conservatively
  rejected** with `ErrNonFinite` by `Then` or `Inverse`, even when the exact
  result is representable. That is deliberate: the unit here is the millimetre,
  and `1e308` mm is ~`1e289` light-years. The failure is one-sided — an error,
  never a wrong answer.
- **`Transform.Inverse` is exact** — the transpose of an orthogonal matrix, never
  a solve. Admitting scale would cost this.
- **A normal transforms like a direction** (`ApplyDir`). No inverse transpose,
  anywhere. Under a general affine map normals need one and everybody forgets;
  here it cannot be got wrong.
- **`Vec.Normalize` returns `(Vec, bool)`** and never fabricates a unit vector
  from a zero vector. It is a divide-by-zero guard, not a geometric tolerance —
  and not a size limit: a vector whose *length* overflows, like
  `(MaxFloat64, MaxFloat64, 0)`, still has a direction, and gets it
  (`(1/√2, 1/√2, 0)`).
- **Angles are typed** (`units.Value`), so `Rotation` rejects a length, a bare
  scalar, and the zero value alike — a forgotten angle is an error, not a silent
  0°.

## License

This project is **source-available**, and is licensed under the
[PolyForm Noncommercial License 1.0.0](LICENSE).

* **Noncommercial use is free.** Individuals, hobby and personal projects,
  research, education, nonprofits, and government may use, modify, and
  redistribute it at no cost, subject to the license terms.
* **Commercial / business use requires a separate license.** Any use by or for
  a business, or for commercial advantage, is not permitted under the
  noncommercial license. To obtain a commercial license, reach out on Bluesky
  at [@lestrrat.bsky.social](https://bsky.app/profile/lestrrat.bsky.social).

### Contributions

This repository does **not** accept external pull requests.
