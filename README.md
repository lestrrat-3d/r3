# r3

Coordinate math for Euclidean 3-space (ℝ³) in Go: a `Vec` vector type and an
orthonormal right-handed `Frame` carrying the transform between a plane's local
`(u, v, w)` coordinates and world `(x, y, z)`.

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

## Scope

The package holds what *lives in* 3-space and what *acts on* it — vectors,
frames, and the transforms between them. Nothing else. It carries no document
state and depends only on the standard library.

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
- **`Vec.Normalize` returns `(Vec, bool)`** and never fabricates a unit vector
  from a zero vector. It is a divide-by-zero guard, not a geometric tolerance.

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
