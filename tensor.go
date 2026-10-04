package r3

import (
	"errors"
	"math/big"
)

// ErrNotPositiveDefinite reports that a symmetric tensor has no positive
// quadratic form in every nonzero direction.
var ErrNotPositiveDefinite = errors.New("r3: symmetric tensor is not positive definite")

// ErrTensorPrecision reports that float64 rounding made a positive-definite
// tensor result singular or indefinite.
var ErrTensorPrecision = errors.New("r3: tensor result lost positive definiteness")

// SymmetricTensor is a symmetric linear map in three-space. Its six
// components are numerical coordinates; callers assign their physical units.
// The zero value is the zero map. A tensor built with NewSymmetricTensor
// contains only finite components.
type SymmetricTensor struct {
	xx, yy, zz float64
	xy, xz, yz float64
}

// NewSymmetricTensor builds a symmetric map from its three diagonal and three
// mixed components. It returns ErrNonFinite for any NaN or infinite component.
func NewSymmetricTensor(xx, yy, zz, xy, xz, yz float64) (SymmetricTensor, error) {
	if !isFinite(xx) || !isFinite(yy) || !isFinite(zz) ||
		!isFinite(xy) || !isFinite(xz) || !isFinite(yz) {
		return SymmetricTensor{}, ErrNonFinite
	}
	return SymmetricTensor{xx: xx, yy: yy, zz: zz, xy: xy, xz: xz, yz: yz}, nil
}

// Components returns the diagonal components followed by the XY, XZ, and YZ
// mixed components.
func (s SymmetricTensor) Components() (xx, yy, zz, xy, xz, yz float64) {
	return s.xx, s.yy, s.zz, s.xy, s.xz, s.yz
}

// Apply returns the tensor acting on v. Like Vec.Dot, this per-vector operation
// does not reject overflow; callers using extreme magnitudes must check the
// resulting vector for non-finite components.
func (s SymmetricTensor) Apply(v Vec) Vec {
	return Vec{
		X: s.xx*v.X + s.xy*v.Y + s.xz*v.Z,
		Y: s.xy*v.X + s.yy*v.Y + s.yz*v.Z,
		Z: s.xz*v.X + s.yz*v.Y + s.zz*v.Z,
	}
}

// Rotate returns R s Rᵀ, where R is the orthogonal part of by. Translation
// has no effect. Reflections use the same rule. An invalid transform returns
// ErrNonFinite or ErrNotOrthonormal. A non-finite result returns ErrNonFinite;
// an SPD input whose rounded result loses positivity returns ErrTensorPrecision.
// The finite result is numerical and carries no error bound for its components.
func (s SymmetricTensor) Rotate(by Transform) (SymmetricTensor, error) {
	if err := by.validate(); err != nil {
		return SymmetricTensor{}, err
	}
	b := by.Basis()
	// Each vector is Rᵀ times a world coordinate axis.
	x := Vec{X: b.EX.X, Y: b.EY.X, Z: b.EZ.X}
	y := Vec{X: b.EX.Y, Y: b.EY.Y, Z: b.EZ.Y}
	z := Vec{X: b.EX.Z, Y: b.EY.Z, Z: b.EZ.Z}
	sx, sy, sz := s.Apply(x), s.Apply(y), s.Apply(z)
	rotated, err := NewSymmetricTensor(
		x.Dot(sx), y.Dot(sy), z.Dot(sz),
		x.Dot(sy), x.Dot(sz), y.Dot(sz),
	)
	if err != nil {
		return SymmetricTensor{}, err
	}
	if s.IsPositiveDefinite() && !rotated.IsPositiveDefinite() {
		return SymmetricTensor{}, ErrTensorPrecision
	}
	return rotated, nil
}

// IsPositiveDefinite reports whether v·s.Apply(v) is positive for every
// nonzero real vector v. It evaluates Sylvester's minors exactly for the
// supplied float64 components, so rounding cannot admit a singular tensor.
func (s SymmetricTensor) IsPositiveDefinite() bool {
	_, _, ok := s.positiveMinors()
	return ok
}

// Inverse returns the inverse of a positive-definite tensor. It returns
// ErrNotPositiveDefinite for a singular or indefinite tensor, and ErrNonFinite
// when an inverse component cannot be represented as a finite float64. If
// rounding loses positive definiteness, it returns ErrTensorPrecision.
func (s SymmetricTensor) Inverse() (SymmetricTensor, error) {
	cofactor, determinant, ok := s.positiveMinors()
	if !ok {
		return SymmetricTensor{}, ErrNotPositiveDefinite
	}
	var values [6]float64
	for i, numerator := range cofactor {
		exact := new(big.Rat).Quo(numerator, determinant)
		value, _ := exact.Float64()
		if !isFinite(value) {
			return SymmetricTensor{}, ErrNonFinite
		}
		values[i] = value
	}
	inverse, err := NewSymmetricTensor(values[0], values[1], values[2],
		values[3], values[4], values[5])
	if err != nil {
		return SymmetricTensor{}, err
	}
	if !inverse.IsPositiveDefinite() {
		return SymmetricTensor{}, ErrTensorPrecision
	}
	return inverse, nil
}

func (s SymmetricTensor) positiveMinors() ([6]*big.Rat, *big.Rat, bool) {
	var zero [6]*big.Rat
	if !isFinite(s.xx) || !isFinite(s.yy) || !isFinite(s.zz) ||
		!isFinite(s.xy) || !isFinite(s.xz) || !isFinite(s.yz) {
		return zero, nil, false
	}
	xx, yy, zz := ratOf(s.xx), ratOf(s.yy), ratOf(s.zz)
	xy, xz, yz := ratOf(s.xy), ratOf(s.xz), ratOf(s.yz)
	if xx.Sign() <= 0 {
		return zero, nil, false
	}
	czz := ratSub(ratMul(xx, yy), ratMul(xy, xy))
	if czz.Sign() <= 0 {
		return zero, nil, false
	}
	cxx := ratSub(ratMul(yy, zz), ratMul(yz, yz))
	cyy := ratSub(ratMul(xx, zz), ratMul(xz, xz))
	cxy := ratSub(ratMul(xz, yz), ratMul(xy, zz))
	cxz := ratSub(ratMul(xy, yz), ratMul(xz, yy))
	cyz := ratSub(ratMul(xy, xz), ratMul(xx, yz))
	determinant := new(big.Rat).Add(ratMul(xx, cxx), ratMul(xy, cxy))
	determinant.Add(determinant, ratMul(xz, cxz))
	if determinant.Sign() <= 0 {
		return zero, nil, false
	}
	return [6]*big.Rat{cxx, cyy, czz, cxy, cxz, cyz}, determinant, true
}

func ratOf(value float64) *big.Rat {
	result := new(big.Rat).SetFloat64(value)
	return result
}

func ratMul(a, b *big.Rat) *big.Rat {
	return new(big.Rat).Mul(a, b)
}

func ratSub(a, b *big.Rat) *big.Rat {
	return new(big.Rat).Sub(a, b)
}
