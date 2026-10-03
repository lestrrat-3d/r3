# Screw Motion Design

How r3 reads the screw parameters of a `Transform` — the rotation axis, the angle about it and the translation
along it — and how it interpolates between two transforms along that screw: the scope (§1), the public API (§2),
the conventions that make every result deterministic (§3), the decomposition algorithm regime by regime (§4),
the evaluation of a partial screw (§5), interpolation between two poses (§6), the accuracy the implementation
must meet (§7), the files and tests (§8), the options rejected (§9) and what is out of scope (§10).

Every rigid motion of ℝ³ with determinant +1 is a screw motion: a rotation by an angle `θ` about a line, followed
by a translation `d` along that same line (Chasles). r3 exposes that fact as a value a caller can read, and as a
path a caller can sample: the screw at fraction `s` rotates by `s·θ` and slides by `s·d` about the same line.

## 1. Scope

Two consumers set the scope, and the design serves them and nothing else.

- decad's `VerifyMotion` (decad `docs/motion-check-design.md` §2 and §11) stages a motion kind
  `Between{From, To r3.Transform}` on this feature. It needs the axis line, the angle and the axial translation
  as separate values, because its travel bound over a parameter step is a rotation term `ρ_max·|Δθ|` plus a
  translation term `|Δd|`, where `ρ_max` is the farthest any point of the mover sits from the axis line. An
  interpolated `Transform` alone cannot give it that bound.
- kinetograph, an animation toolkit, may sample the interpolated transform at many `s`.

decad's second gap in that §11 — a published bound on how far r3's own arithmetic deviates from the ideal isometry
— is not part of this design (§10).

## 2. The API

Everything lands in one new file, `screw.go`, beside `transform.go`. Nothing in `transform.go`, `frame.go` or
`vec.go` changes except that `Transform.Screw` and `Transform.Interpolate` are added to the list of derivations
their doc comments name.

```go
// ErrImproper is returned by Transform.Screw when the transform is a reflection
// (determinant −1). A reflection is an isometry but not a motion: no screw —
// no rotation about any line, with any slide along it — produces it.
var ErrImproper = errors.New("r3: improper transform (a reflection has no screw motion)")

// Screw is a rigid motion written as a rotation by Angle about the line through
// Point along Axis, followed by a translation of Slide along Axis.
type Screw struct {
    Axis  Vec         // unit direction of the screw axis
    Point Vec         // a point on the axis
    Angle units.Value // rotation about Axis, right-handed, in radians
    Slide float64     // translation along Axis, signed, in the coordinate unit (millimetres)
}

// Screw returns t written as a screw motion.
func (t Transform) Screw() (Screw, error)

// At returns the transform that rotates by s·Angle about the axis and slides
// s·Slide along it. At(0) is Identity(); At(1) is the whole motion.
func (s Screw) At(frac float64) (Transform, error)

// Interpolate returns the pose a fraction s of the way from t to to along the
// screw motion joining them. Interpolate(to, 0) is t; Interpolate(to, 1) is to.
func (t Transform) Interpolate(to Transform, s float64) (Transform, error)
```

`Screw` has exported fields, as `Basis` does: it is a plain value a consumer reads, and the invariants it carries
(a unit `Axis`, a `Point` on the axis) are what `Transform.Screw` produces, not what the type enforces. `At`
re-derives what it needs from the fields it is handed — it takes `Axis` for its direction only, so a non-unit
axis names the same screw, and any point on the axis line names the same screw as the closest one — and validates
the `Transform` it produces, as every producer in the package does. The zero value `Screw{}` has no axis
direction, and `At` rejects it with `ErrDegenerateAxis`.

`Angle` is a `units.Value` because every angle in the package is one (`Rotation` takes one). `Slide` is a bare
`float64` because every length in the package is one: `Vec` components, `Vec.Len`, `Frame.ToWorldUV`'s `u, v`.
The coordinate unit is the millimetre by convention, as the package doc states.

`Transform.Screw` and `Screw.At` are the primitives. `Transform.Interpolate` is `from⁻¹·to` decomposed and
evaluated (§6); a caller that samples one motion at many `s` and also wants the parameters calls `Screw` once and
`At` per sample, which is what decad does.

## 3. Conventions

Every output of `Transform.Screw` is a deterministic function of the input bits. These rules make it so.

| Quantity | Convention |
|---|---|
| `Angle` | In `[0, π]` radians. Never negative, never above `π`. A rotation by `−θ` about `n` is reported as `θ` about `−n`; a rotation by `θ ∈ (π, 2π)` about `n` is reported as `2π − θ` about `−n`. |
| `Axis` sign, `Angle ∈ (0, π)` | Forced: the sign that makes the rotation right-handed by `Angle`. |
| `Axis` sign, `Angle = π` exactly | Both signs describe the same rotation. The sign is taken from the antisymmetric part of the linear part when that part is nonzero (§4 step 4); when it is exactly zero, the sign makes the first nonzero component of `Axis`, in `X, Y, Z` order, positive. |
| `Axis`, `Angle = 0`, nonzero translation | The unit direction of the translation. `Slide` is then `|t| > 0`. |
| `Axis`, identity | `(0, 0, 1)`. `Point` is `(0, 0, 0)`, `Angle` is `0`, `Slide` is `0`. |
| `Point` | The point of the axis line closest to the origin, so `Point·Axis = 0` to rounding. `(0, 0, 0)` when `Angle = 0`, because every line parallel to the translation is then an axis and the one through the origin is chosen. |
| `Slide` | Signed: `t·Axis`. Negative when the translation runs against `Axis`. With `Angle = 0` the axis convention above makes it non-negative. |
| Negative zero | Every component of `Axis` and `Point`, and `Slide`, is written through `canonicalZero` (`−0 → +0`), so two inputs that differ only in the sign of a zero print the same and the example's `// Output:` block holds. `units.Radians` already canonicalizes `Angle`. |

Two consequences the reader should expect:

- A full turn is not reported as `2π`. `Rotation(axisZ, Radians(2π))` stores `sin(2π) ≈ −2.4e-16` in its
  off-diagonal entries, so its linear part is a rotation by `2.4e-16` about `−Z` to the last bit, and that is what
  `Screw` returns. The angle is read from the matrix, which does not remember how many turns built it.
- A rotation of exactly `π` built by `Rotation(n, Radians(π))` stores `sin(π_float64) ≈ +1.2e-16`, which is
  nonzero, so the axis sign comes from it (`+n`), not from the first-nonzero-component rule. That rule applies to a
  linear part that is symmetric to the bit, such as `FromBasis(Basis{EX: (−1,0,0), EY: (0,−1,0), EZ: (0,0,1)}, 0)`,
  which yields `Axis = (0, 0, 1)`.

## 4. `Transform.Screw`

Notation: the linear part `R` has columns `ex, ey, ez` (so `R_ij` is column `j`, component `i`: `R_32 = ey.Z`,
`R_23 = ez.Y`), and `t` is the translation. `ε` is the float64 unit roundoff, `2⁻⁵³`.

### Step 1 — admit the input

Run `t.validate()` first, exactly as `Then` and `Inverse` do: a non-finite translation is `ErrNonFinite`, and a
linear part that fails `IsValid` — the zero value `Transform{}`, a NaN entry, a basis skewed past `1e-9` — is
`ErrNotOrthonormal`. Then `IsReflection()` true is `ErrImproper`. The order puts the complaint on the field at
fault: a NaN translation is reported as the non-finite value it is, not as a complaint about a linear part that may
be fine.

### Step 2 — the angle

```text
a    = (ey.Z − ez.Y,  ez.X − ex.Z,  ex.Y − ey.X)     // R − Rᵀ, as a vector: a = 2·sin θ · n
cosθ = (ex.X + ey.Y + ez.Z − 1) / 2                  // (trace − 1) / 2
sinθ = a.Len() / 2
θ    = math.Atan2(sinθ, cosθ)
```

`sinθ ≥ 0` by construction, so `θ ∈ [0, π]`. `Atan2` is used, not `Acos` or `Asin`, because it is accurate at
both ends of the range and accepts arguments a within-tolerance input can produce (`sinθ` a few `1e-10` above `1`,
`cosθ` a few `1e-10` below `−1`), which `Acos` would turn into NaN.

The three differences in `a` are exact or within one rounding of exact, and when the entries themselves are small
(a near-identity rotation stores off-diagonal entries of size `θ`) the differences are small with the same relative
precision, so `sinθ` carries relative error `~ε` down to the smallest `θ` the input can state. Near `π` the entries
are of order one, `a` is small with absolute error `~ε`, and `θ = π − O(|a|)` carries absolute error `~ε`.

### Step 3 — the axis, by regime

The regime is chosen on `a` and `cosθ` as computed in step 2.

| Regime | Condition | Axis |
|---|---|---|
| A, translation | `a == Vec{}` (exactly zero, all three components) and `cosθ > 0` | `t.direction()` when it reports a direction; `Vec{Z: 1}` when `t` is zero |
| B, antisymmetric | `cosθ >= 0`, and not regime A | `a.direction()` |
| C, symmetric | `cosθ < 0` | the longest column of `R + Rᵀ − (trace − 1)·I`, normalized, signed by `a` (step 4) |

Regime A is exact, not thresholded. `a` is exactly zero iff the stored linear part is symmetric to the bit, and a
proper symmetric orthogonal matrix is the identity or a half-turn; `cosθ > 0` (in fact `≈ 1`) selects the
identity. Any nonzero `a`, however small, names a rotation the input states, and r3 does not snap it to zero: the
package's rule is that smallness is not degeneracy (`Rotation` accepts a `1e-20` axis for the same reason). The
cost is a far-away `Point` for a tiny rotation with a perpendicular translation (step 6), which is the true
geometry of that input.

Regime B uses `Vec.direction()`, the unexported scale-free normalizer, NEVER `Vec.Normalize`: `Normalize`'s
`zeroLen` floor of `1e-12` would refuse `a` for every rotation below `5e-13` radians, and such a rotation is a
rotation. `direction()` reports false only for the exact zero vector, which regime A has already taken. The
antisymmetric axis `a/|a|` has relative error `~ε/sinθ`: at most `√2·ε` anywhere in regime B, and `~ε/θ` as `θ → 0`.
The latter is accepted: for a tiny rotation the axis direction is determined by the input only to that precision,
and §7 shows it does not disturb the rebuild.

Regime C uses the symmetric part. `R + Rᵀ = 2·cosθ·I + 2(1 − cosθ)·n·nᵀ`, so `S = R + Rᵀ − (trace − 1)·I` equals
`2(1 − cosθ)·n·nᵀ`, and every column of `S` is a multiple of `n`: column `j` is `2(1 − cosθ)·n_j·n`. Its diagonal
is `S_jj = 2·R_jj − (trace − 1)`, and the column with the largest diagonal entry (ties resolved in `X, Y, Z` order)
has `|n_j| ≥ 1/√3`, so its length is at least `2(1 − cosθ)/√3 > 1.15` in this regime and `Normalize` or
`direction()` both serve. In terms of the stored columns:

```text
S_col_X = (2·ex.X − (trace − 1),  ex.Y + ey.X,            ex.Z + ez.X)
S_col_Y = (ey.X + ex.Y,            2·ey.Y − (trace − 1),  ey.Z + ez.Y)
S_col_Z = (ez.X + ex.Z,            ez.Y + ey.Z,            2·ez.Z − (trace − 1))
```

The entries of `S` are of order one with absolute error `~ε`, so the normalized column has relative error
`~ε/(2(1 − cosθ)) < ε` throughout regime C. Regime B's error there would be `ε/sinθ`, unbounded as `θ → π`.

The split sits at `cosθ = 0`, i.e. `θ = π/2`, because that is where the two error curves `ε/sinθ` and
`ε/(1 − cosθ)` cross; both are `~ε` there, so the two regimes agree to `~ε` at the boundary and no result jumps
when an input crosses it.

### Step 4 — the sign of a regime-C axis

A normalized column of `S` is `±n`. Let `cand` be it. `a·cand = 2·sinθ·(n·cand) = ±2·sinθ`, and `sinθ > 0` for
`θ ∈ (0, π)`, so:

- `a·cand > 0` → `Axis = cand`.
- `a·cand < 0` → `Axis = cand.Scale(−1)`.
- `a·cand == 0` → `a` is exactly zero and `θ` is exactly `π` (step 2 gives `Atan2(0, cosθ < 0) = π`). Both signs
  are the same rotation. Flip `cand` if needed so that its first nonzero component in `X, Y, Z` order is positive.

The sign read from `a` is reliable while `2·sinθ` exceeds the `~ε` noise in `a`, i.e. for every `θ` below
`π − 1e-15`; past that, an input is a half-turn to within its own rounding and either sign is a faithful
decomposition. The result is still deterministic: it is a function of the stored bits.

### Step 5 — the slide

```text
d = t.dotUnit(Axis)
```

`dotUnit` because `Axis` is a unit vector: the products cannot overflow and the sum is retried scaled if it does,
so `d` is `±Inf` exactly when the true value is past `MaxFloat64`. A non-finite `d` is `ErrNonFinite`: the screw
exists, but its slide is not a float64.

### Step 6 — the point

```text
if θ == 0:   Point = Vec{}
else:
    t_perp = t.Sub(Axis.Scale(d))                      // component of t across the axis
    sh, ch = math.Sincos(θ / 2)                        // sh > 0 on (0, π]
    Point  = t_perp.Scale(0.5).Add(Axis.Cross(t_perp).Scale(ch / sh / 2))
```

Derivation. A rotation by `θ` about the line through `c` along `n`, then a slide `d`, moves `p` to
`R(p − c) + c + d·n = R·p + (I − R)·c + d·n`, so `t_perp = (I − R)·c` for the `c ⊥ n` on the axis. For `v ⊥ n`,
`R·v = cosθ·v + sinθ·(n × v)`, so `(I − R)·v = (1 − cosθ)·v − sinθ·(n × v)`, and inverting that 2D rotation-scale in
the plane across `n` gives `c = ½·t_perp + ½·cot(θ/2)·(n × t_perp)`. At `θ = π` the second term vanishes and
`c = t_perp/2`. As `θ → 0` with `t_perp ≠ 0`, `|c| ≈ |t_perp|/θ` grows without bound: a tiny rotation with a
sideways translation is a rotation about a far-away axis, and that is the answer. `Point` carries relative error
`~ε` from the two terms, so its absolute error is `~ε·|t|/θ`; §7 shows the rebuild absorbs it.

A non-finite `Point` — `|t_perp|/θ` past `MaxFloat64`, or `t_perp` itself overflowing for `|t| > MaxFloat64/2` —
is `ErrNonFinite`. This refuses a valid transform, and it is the accepted limit: the axis point of such a motion is
not a float64 point. No model in millimetres is near it; with `|t| ≤ 1e6` the point is finite for every
`θ ≥ 1e-300`.

### Step 7 — assemble

Apply `canonicalZero` to every component of `Axis` and `Point` and to `d` (§3), then return
`Screw{Axis, Point, Angle: units.Radians(θ), Slide: d}`.

### Errors

| Input | Error |
|---|---|
| `t.Translation()` has a NaN or infinite component (includes a hand-built invalid transform) | `ErrNonFinite` |
| `t.IsValid()` false for the linear part: `Transform{}`, a NaN entry, a skew past `1e-9` | `ErrNotOrthonormal` |
| `t.IsReflection()` true | `ErrImproper` |
| `Slide` or `Point` not representable (step 5, step 6) | `ErrNonFinite` |

Every other valid proper transform decomposes with a nil error, the within-tolerance skewed one included: its angle
and axis are read as above, and its rebuild differs from it by at most its own skew (§7).

## 5. `Screw.At`

`At(s)` is `Rotation(Axis, s·Angle)` about the line through `Point`, then a slide of `s·Slide`, computed without
ever forming `c − R·c` (which cancels catastrophically when `Point` is far away).

```text
1. if !isFinite(s)                             → ErrNonFinite
2. n, ok = Axis.direction(); if !ok            → ErrDegenerateAxis       (zero, NaN or Inf axis; Screw{} lands here)
3. θ, err = Angle.In(units.Radian)
      units.ErrNotFinite                       → ErrNonFinite
      other (a length, a bare scalar, the zero units.Value)
                                               → fmt.Errorf("r3: screw angle: %w", err)   (wraps units.ErrIncompatible)
   if !isFinite(θ)                             → ErrNonFinite
4. if !Point.isFinite() || !isFinite(Slide)    → ErrNonFinite
5. φ = s·θ;  sin, cos = math.Sincos(φ);  h = math.Sin(φ / 2)
   R_s   = rodrigues(X, n, sin, cos), rodrigues(Y, n, sin, cos), rodrigues(Z, n, sin, cos)
   cperp = Point.Sub(n.Scale(Point.dotUnit(n)))                 // the point's offset from the axis through the origin
   t_s   = cperp.Scale(2·h·h).Sub(n.Cross(cperp).Scale(sin)).Add(n.Scale(s·Slide))
6. out = Transform{ex, ey, ez: R_s columns, t: t_s};  out.validate()  → ErrNonFinite / ErrNotOrthonormal
```

Steps 2–3 mirror `Rotation`'s own checks and error mapping. Step 5 reuses the package's `rodrigues`; the angle
check in step 3 is why `Rotation` itself is not called (it would re-run the checks, and `At` is the per-sample
path).

Why the translation is written that way:

- `t_s = (I − R_s)·c_perp + s·d·n`, and `(I − R_s)·v = (1 − cosφ)·v − sinφ·(n × v)` for `v ⊥ n`. Projecting `Point`
  to `c_perp` first makes the formula right for any point on the axis line a caller supplies, not only the closest
  one (`R_s` fixes the along-axis component, so it contributes nothing).
- `1 − cosφ` is computed as `2·sin²(φ/2)`, never as `1 − cos(φ)`. For `φ = 1e-8`, `1 − cos(φ)` evaluates to `0`
  where the true value is `5e-17`, and against a `|c_perp|` of `1e8` that is a `5e-9` translation error — above the
  §7 tolerance. `2·sin²(φ/2)` has relative error `~ε` for every `φ`.
- For a far `Point` (`|c_perp| ≈ |t|/θ`), both terms are of order `s·|t|` with relative error `~ε`, so the
  translation is accurate to `~ε·|t|` without ever forming a large cancelling difference.
- At `s = 0`: `sin = 0`, `cos = 1`, `h = 0`, so `rodrigues` returns each basis vector unchanged and `t_s` is exactly
  zero. `At(0)` equals `Identity()` bit for bit. At `θ = 0` (a pure translation), `φ = 0` for every `s`, and
  `t_s = n·(s·Slide)`: no separate branch is needed.

`s` is any finite float64. Values outside `[0, 1]` continue the same screw (`At(2)` is the motion applied twice,
`At(−1)` its inverse); nothing in the two consumers' needs calls for a range check, and the formula is meaningful
everywhere.

## 6. `Transform.Interpolate`

```text
inv, err  = t.Inverse()               // ErrNonFinite, ErrNotOrthonormal
rel, err  = inv.Then(to)              // the motion X with t.Then(X) == to:  X·p = to(t⁻¹(p))
sc, err   = rel.Screw()               // ErrImproper when exactly one of t, to is a reflection
step, err = sc.At(s)
return t.Then(step)
```

`rel` is the relative motion in world coordinates: `t.Then(rel)` is `to`, so `t.Then(sc.At(1))` is `to` and
`t.Then(sc.At(0))` is `t`. The alternative, the body-frame relative motion `to.Then(inv)` with the pose written
`sc'.At(s).Then(t)`, traces the same path: the two screws are conjugate by `t`, and `t·Y^s·t⁻¹ = (t·Y·t⁻¹)^s`.
There is no left/right choice to expose, and a test pins the two to agree (§8).

Shortest path. `rel.Screw()` reports `Angle ∈ [0, π]`, so the interpolation rotates through the smaller of the
two arcs between the orientations. Every other screw joining `t` to `to` has the same axis line and slide with an
angle of `Angle + 2πk`, `k ≠ 0`, and is longer; none is taken.

Half-turn. When the orientations are exactly `π` apart, the two arcs are equally short and the sign of `Axis` picks
one (§3). The choice is deterministic for a given pair of inputs, and the pose at `s = 1` is `to` whichever is
taken; only the intermediate poses differ between the two arcs. A caller that needs a particular arc states an
intermediate pose and interpolates in two legs.

Two reflections interpolate: their relative motion is proper. One reflection and one proper transform do not,
and `ErrImproper` says so: no continuous rigid path crosses between the two handedness classes.

## 7. Accuracy

The claims below are what the tests in §8 assert, stated with the tolerance each test uses. The analysis after
them says why the implementation meets them with margin; a measured run of the §4–§5 algorithm over 20,000 random
rigid motions with coordinates in `[−10, 10)`, angles spanning `[1e-16, π − 1e-16]`, gave a worst rebuild error of
`1.4e-14`.

| Claim | Tolerance |
|---|---|
| `sc.At(1).Equal(t, tol)` for every proper `t` built from the public constructors and their `Then` compositions with coordinates in `[−10, 10)` | `1e-12` |
| `sc.At(0).Equal(Identity(), 0)` | exact |
| `sc.At(0.5)` composed with itself equals `t` | `1e-12` |
| `t.Interpolate(to, 0).Equal(t, tol)` and `t.Interpolate(to, 1).Equal(to, tol)` for random proper pairs as above | `1e-12` |
| `Angle` of `Rotation(n, Radians(θ))` within `tol` of `θ`, for `θ ∈ (0, π]` | `1e-12` |
| `Axis` of `Rotation(n, Radians(θ))` within `tol` of `n̂` componentwise, for `θ ∈ [1e-3, π]` | `1e-12` |
| `Point` of `RotationAround(c, n, Radians(θ))` within `tol` of `c − n̂·(n̂·c)` componentwise, for `θ ∈ [1e-3, π]`, `|c| ≤ 10` | `1e-9` |
| `Point·Axis` for every decomposition | `1e-9·(1 + |Point|)` |
| rebuild of a within-tolerance skewed input (`TransformWithBasis`, `EX·EY = 7e-10`) | `2e-9` |

Why the rebuild is tight regardless of how well the axis is known. The rebuilt linear part `R'` matches `R`: in
regime B the antisymmetric part of `R'` is `a/2` by construction and the symmetric part differs by
`(1 − cosθ)·δ ≈ θ²·(ε/θ) = θ·ε` where `δ` is the axis error; in regime C the axis error is `< ε` outright. The
rebuilt translation is `(I − R')·c + d·n` with `c`, `d` and `t_perp` all defined from the SAME computed `n` and
`θ`, so algebraically it is `t_perp + d·n = t` for any `n`, and only rounding remains: `c` to relative `ε`, scaled
back down by `(I − R')`, leaves `~ε·|t|`. The two `1e-3` floors in the table exist because the ANGLE and POINT of
a tiny rotation are determined by the input only to `~ε/θ` and `~ε·|c|/θ²` — a property of the input, which the
rebuild does not inherit.

The skewed input is the one case the rebuild cannot match to `1e-12`: `R'` is orthonormal to machine precision
and `R` is not, so they differ by the skew itself, below `1e-9`. `2e-9` leaves room for the rounding on top.

## 8. Files and tests

| File | Contents |
|---|---|
| `screw.go` | `ErrImproper`, `Screw`, `Transform.Screw`, `Screw.At`, `Transform.Interpolate`, `canonicalZero`. Doc comments carry the §3 conventions and the §4 error table. |
| `screw_test.go` | Package `r3_test`. The tests below, using `transform_test.go`'s `randTransform`, `randVec`, `axisX/Y/Z` where they fit. |
| `screw_bench_test.go` | `BenchmarkTransformScrew`, `BenchmarkScrewAt`, `BenchmarkTransformInterpolate`, in the style of `transform_bench_test.go`. CI runs every benchmark once. |
| `examples/r3_screw_example_test.go` | `Example_r3_screw`, package `examples_test`, `// Output:` verified by `go test ./examples/`. |
| `README.md` | A short block after the `Transform` example showing `Screw` and `Interpolate`, and a pointer to this file under a new `## Design documents` heading. |
| `doc.go` | One sentence in the package overview naming `[Transform.Screw]` and `[Transform.Interpolate]`, and `Screw.At`, `Transform.Screw` and `Transform.Interpolate` added where the Invariants paragraph lists the fallible derivations. |

Tests. Each row is a `t.Run` name under the named top-level test; each must be seen to fail before it passes
(delete the formula it protects, or feed the wrong input, and watch it go red).

`TestTransformScrew`

| Subtest | Asserts | Would fail under |
|---|---|---|
| `identity` | `Axis = (0,0,1)`, `Point = 0`, `Angle = 0 rad`, `Slide = 0`, exactly | regime A missing |
| `a pure translation` | `Translation((1,2,3))`: `Angle = 0`, `Axis = (1,2,3)/√14`, `Slide = √14`, `Point = 0` within `1e-12` | wrong axis convention for `θ = 0` |
| `a rotation about an offset axis` | `RotationAround((10,0,0), Z, 90°)`: `Angle = π/2`, `Axis = Z`, `Point = (10,0,0)`, `Slide = 0` within `1e-12` | step 6 sign or factor |
| `a screw` | `Rotation(Z, 90°).Then(Translation((0,0,5)))`: `Slide = 5`, `Point = 0` | slide projection |
| `random rebuild` | 200 random proper transforms (`randTransform` minus the reflection case, plus pairs composed with `Then`): `At(1)` within `1e-12`, `At(0)` exact, `At(0.5)²` within `1e-12`, `Angle ∈ [0, π]`, `Point·Axis` small | any formula error |
| `angle and axis of a known rotation` | `Rotation(n, θ)` for `n` random and `θ` in `{1e-3, 0.5, π/2, 2, π − 1e-3, π}`: `Angle` within `1e-12`, `Axis` within `1e-12` of `n̂` (`±` at `θ = π`) | regime split, column choice |
| `near pi uses the symmetric part` | `Rotation((1,1,1), π − 1e-8)`: `Axis` within `1e-12` of `(1,1,1)/√3`, rebuild within `1e-12` | regime C replaced by B (`4e-8` error) |
| `an exact half-turn with a symmetric linear part` | `FromBasis({(−1,0,0),(0,−1,0),(0,0,1)}, 0)`: `Angle = π` exactly, `Axis = (0,0,1)` exactly (no `−0`) | canonical sign rule, `canonicalZero` |
| `a half-turn's axis sign follows the stored residue` | `Rotation(Z, 180°)` and `Rotation(Z, −180°)` both give `Angle = π` and `Axis = ±Z`; each rebuilds within `1e-12` | step 4 |
| `a tiny rotation with a sideways translation` | `Rotation(Z, 1e-8).Then(Translation((1,0,0)))`: `Angle = 1e-8` within `1e-18`, `Point ≈ (0.5, 1e8, 0)` within `1e-4`, rebuild within `1e-12` | `1 − cos` instead of `2 sin²` in `At` (`5e-9` error); `Normalize` instead of `direction` at `1e-13` |
| `a rotation below Normalize's floor` | `Rotation(Z, 1e-13).Then(Translation((1,0,0)))` decomposes with nil error and rebuilds within `1e-12` | `Normalize` in regime B |
| `a full turn is a near-zero rotation` | `Rotation(Z, 2π)`: `Angle < 1e-15`, `Slide = 0`, rebuild within `1e-12` | a claim of `2π` |
| `a within-tolerance skew is admitted` | `TransformWithBasis(EX=(1,0,0), EY=(7e-10,1,0), EZ=(0,0,1), t=(1,2,3))`: nil error, rebuild within `2e-9` | over-strict admission |
| `a reflection is improper` | `Reflection(xy)` → `ErrImproper`, zero `Screw` | — |
| `an invalid transform is refused` | `Transform{}` → `ErrNotOrthonormal`; `TransformWithTranslation(NaN)` → `ErrNonFinite` | check order |
| `an unrepresentable point is refused` | `Rotation(Z, 1e-300).Then(Translation((1e10, 0, 0)))`: `ErrNonFinite` (point `~1e310`) | missing finiteness check |

`TestScrewAt`

| Subtest | Asserts |
|---|---|
| `accepts any point on the axis` | `Screw{Axis: Z, Point: (10,0,7), Angle: 90°, Slide: 0}.At(1)` equals `RotationAround((10,0,0), Z, 90°)` within `1e-12`; a non-unit `Axis: (0,0,3)` gives the same |
| `extrapolates` | `At(2)` equals `At(1).Then(At(1))` within `1e-12`; `At(−1)` equals `At(1).Inverse()` |
| `rejects the zero value` | `Screw{}.At(0.5)` → `ErrDegenerateAxis` |
| `rejects a non-finite fraction` | `At(NaN)`, `At(+Inf)` → `ErrNonFinite` |
| `rejects an angle that is not an angle` | `Angle: units.Millimeters(1)` and the zero `units.Value` → wraps `units.ErrIncompatible`; `Angle: units.Radians(NaN)` → `ErrNonFinite` |
| `rejects a non-finite point or slide` | `Point: (NaN,0,0)` and `Slide: Inf` → `ErrNonFinite` |

`TestTransformInterpolate`

| Subtest | Asserts |
|---|---|
| `endpoints` | 200 random proper pairs: `Interpolate(to, 0)` within `1e-12` of `from`, `Interpolate(to, 1)` within `1e-12` of `to` |
| `world and body frames trace the same path` | for `s ∈ {0.25, 0.5, 0.75}`: `from.Then(rel.Screw().At(s))` equals `relBody.Screw().At(s).Then(from)` within `1e-12`, with `rel = from⁻¹.Then(to)` and `relBody = to.Then(from⁻¹)` |
| `takes the shorter arc` | `Identity().Interpolate(Rotation(Z, 270°), 0.5)` equals `Rotation(Z, −45°)` within `1e-12` |
| `a half-turn picks one arc deterministically` | `Identity().Interpolate(Rotation(Z, 180°), 0.5)` is a `π/2` rotation about `±Z`, and composing it with itself gives `Rotation(Z, 180°)` within `1e-12`; two calls return bit-identical results |
| `two reflections interpolate, one does not` | `Reflection(xy).Interpolate(Reflection(yz), 0.5)` succeeds and is a reflection; `Identity().Interpolate(Reflection(xy), 0.5)` → `ErrImproper` |
| `a pure translation moves linearly` | `Identity().Interpolate(Translation((2,4,6)), 0.25)` equals `Translation((0.5,1,1.5))` within `1e-12` |

`Example_r3_screw` builds `RotationAround((10,0,0), Z, 90°).Then(Translation((0,0,4)))`, decomposes it, prints
`Axis`, `Point`, `Angle` in degrees and `Slide` with `%.1f`, then prints the point `(11,0,0)` under
`Interpolate` from `Identity()` at `s = 0.5`. Expected output:

```text
axis:  (0.0, 0.0, 1.0)
point: (10.0, 0.0, 0.0)
angle: 90.0 deg
slide: 4.0
half:  (10.7, 0.7, 2.0)
```

(`(11,0,0)` rotated 45° about the axis through `(10,0,0)` lands at `(10 + cos 45°, sin 45°, 0) = (10.707, 0.707, 0)`,
lifted by half the slide.) The implementer verifies the block by running `go test ./examples/` and, if a digit
differs, corrects the block to the computed value, not the formula.

## 9. Rejected options

| Option | Why not |
|---|---|
| `Acos((trace − 1)/2)` for the angle | Infinite derivative at `0` and `π`, so the angle loses half its digits at both ends; and a within-tolerance input gives an argument past `±1`, which is NaN. `Atan2` of the two computed parts has neither problem. |
| Axis from `a/|a|` in every regime | Relative error `ε/sinθ`, unbounded near `π`: at `π − 1e-8` the rebuilt linear part is off by `4e-8`. The symmetric part is accurate exactly where `a` is not. |
| Axis from the symmetric part in every regime | Relative error `ε/(1 − cosθ)`, unbounded near `0`. |
| Snap angles below a threshold to `0` | Invents a regime boundary the input does not have, contradicts the package rule that smallness is not degeneracy, and buys nothing: the far `Point` it would avoid rebuilds exactly (§7). |
| `Point` through `RotationAround(Point, Axis, Angle)` in `At` | Forms `c − R·c`, which for `|c| ≈ |t|/θ` cancels to absolute error `ε·|t|/θ`. The direct `(1 − cosφ)·c − sinφ·(n × c)` form has relative error `ε` per term. |
| `1 − cos(φ)` in `At` | Evaluates to `0` for `φ` below `~1e-8`; the `2 sin²(φ/2)` form does not (§5). |
| `Vec.Normalize` for the regime-B axis | Its `1e-12` floor refuses every rotation below `~5e-13` rad. |
| Per-radian pitch (`Slide/Angle`) instead of the total slide | Infinite for a pure translation and ill-conditioned near `θ = 0`; the total slide is finite for every input and is the quantity decad's bound consumes. |
| `Angle` as a bare `float64` | Every angle in the package is a `units.Value`, and `Rotation` rejects a bare one. |
| `Slide` as a `units.Value` | Every length in the package — `Vec` components, `Vec.Len`, `ToWorldUV`'s `u, v` — is a bare float64 in the coordinate unit; a typed slide would be the only typed length in r3. |
| `Screw` with unexported fields and accessors | The type carries no invariant `At` relies on: `At` takes the axis for its direction and projects the point onto the axis, so a caller-built `Screw` is as good as a read one. `Basis` sets the precedent for a plain value with exported fields. |
| Restrict `At` to `s ∈ [0, 1]` | The formula is meaningful for every finite `s`, and neither consumer asked for the refusal. |
| Reject the identity (`From == To`) in `Screw` | decad refuses `From == To` itself (`ErrDegenerate`, its §2); r3 reports the identity faithfully as a zero screw so a caller can interpolate a pose to itself. |
| A `Transform.Log`/`Exp` pair (the Lie-algebra form `(ω, v)`) | The `(ω, v)` twist does not carry the axis point, which decad needs, without a further division by `|ω|²`; the screw form carries exactly the four quantities asked for. |
| Interpolate through dual quaternions (ScLERP) | Same path as the screw form, with a representation r3 does not have and would have to validate. |
| A separate `Motion`/`Path` type holding `From` and the screw | decad's `Motion` interface owns that shape (its §2); r3 supplies the primitive. |

## 10. Out of scope

A published bound on how far `Rotation`, `RotationAround`, `Translation`, `Then`, `Screw` and `At` deviate from
the ideal isometry they denote (decad `docs/motion-check-design.md` §11, second gap) is not designed here; decad
encloses that error itself.
