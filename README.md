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
state. [`lestrrat-3d/units`](https://github.com/lestrrat-3d/units) is the **only**
non-stdlib dependency a build pulls in — the one thing `go list -deps .` reports
beyond the standard library — and its own package code imports nothing but the
standard library, so the build graph stops there. (`units` requires testify to run
its own tests; that is a test-only dependency of `units` and never reaches a
build.)

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
- **Bigness is not a fault — except in the per-point mappings.** Huge-but-finite
  input is *built*, not refused, on the **cold** paths: `NewFrame` orthonormalizes
  axes out at `MaxFloat64`, and `Reflection` offsets a mirror plane that far out,
  by scaling their arithmetic. They run once per frame or per feature, so the cost
  is nothing. The **per-point** mappings — `Transform.Apply`, `Transform.ApplyDir`,
  `Frame.ToWorld`, `Frame.ToWorldUV`, `Frame.ToLocal` — do not pay it: each sums
  its terms in fixed order, so an intermediate sum can reach `±Inf` where the exact
  result is representable. They run once per transformed point, and overflow-safe
  accumulation would tax every point forever to serve coordinates that cannot exist
  (the unit here is the millimetre; `1e308` mm is ~`1e289` light-years). The cost is
  **not uniform**, so be precise about it:
  - A `Transform` is never silently wrong. `Then` and `Inverse` are fallible: they
    catch the `±Inf` and return `ErrNonFinite`, conservatively refusing a
    composition whose true value was representable.
  - The five mappings above are **infallible**, so at those magnitudes they hand
    back a `Vec` with a non-finite component — a wrong answer, not an error, in
    `Frame`'s world/local mapping exactly as much as in `Transform`'s. At
    `MaxFloat64` you must check the returned `Vec`. Everywhere a real model lives,
    this cannot arise.
- **`Transform.Inverse` is exact** — the transpose of an orthogonal matrix, never
  a solve. Admitting scale would cost this, and so would storing a basis that is
  only *nearly* orthonormal: the transpose inverts a **truly** orthonormal basis and
  nothing else, and one skewed by even `7e-10` round-trips with a drift of ~`1e-8`.
  So the linear part of every `Transform` is orthonormal to machine precision, not
  merely inside the `1e-9` tolerance that *admits* one — `FromBasis`, the door a
  reloaded basis comes in through, **orthonormalizes** what it admits (Gram–Schmidt,
  as `NewFrame` does for a frame's axes) rather than storing it verbatim. What is
  *admitted* is unchanged: a scale, a shear or a collapse is still `ErrNotOrthonormal`,
  never silently corrected into a transform you did not describe. Only tolerance-level
  drift is snapped straight — and **handedness survives it**: a reflection
  (`det = −1`) comes back a reflection, because all three vectors are orthonormalized
  in turn instead of the third being re-derived as `EX × EY`, which would flip an
  improper basis proper without a word.
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
