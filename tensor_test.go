package r3_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSymmetricTensorRotatesInertia(t *testing.T) {
	inertia, err := r3.NewSymmetricTensor(650, 500, 250, 0, 0, 0)
	require.NoError(t, err)
	rotation, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(45))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.Vec{X: 100, Y: -20, Z: 7})
	require.NoError(t, err)
	placement, err := rotation.Then(shift)
	require.NoError(t, err)

	rotated, err := inertia.Rotate(placement)
	require.NoError(t, err)
	xx, yy, zz, xy, xz, yz := rotated.Components()
	require.InDelta(t, 575, xx, 1e-12)
	require.InDelta(t, 575, yy, 1e-12)
	require.InDelta(t, 250, zz, 1e-12)
	require.InDelta(t, 75, xy, 1e-12)
	require.InDelta(t, 0, xz, 1e-12)
	require.InDelta(t, 0, yz, 1e-12)
	require.True(t, rotated.IsPositiveDefinite())

	pureRotation, err := inertia.Rotate(rotation)
	require.NoError(t, err)
	require.Equal(t, pureRotation, rotated)
}

func TestSymmetricTensorInverseRoundTrip(t *testing.T) {
	inertia, err := r3.NewSymmetricTensor(575, 575, 250, 75, 0, 0)
	require.NoError(t, err)
	inverse, err := inertia.Inverse()
	require.NoError(t, err)
	xx, yy, zz, xy, xz, yz := inverse.Components()
	require.InDelta(t, 575.0/325000, xx, 1e-17)
	require.InDelta(t, 575.0/325000, yy, 1e-17)
	require.InDelta(t, 1.0/250, zz, 1e-17)
	require.InDelta(t, -75.0/325000, xy, 1e-17)
	require.Zero(t, xz)
	require.Zero(t, yz)

	v := r3.Vec{X: 10, Y: -4, Z: 7}
	recovered := inverse.Apply(inertia.Apply(v))
	require.InDelta(t, v.X, recovered.X, 1e-14)
	require.InDelta(t, v.Y, recovered.Y, 1e-14)
	require.InDelta(t, v.Z, recovered.Z, 1e-14)
}

func TestSymmetricTensorRejectsInvalidInputs(t *testing.T) {
	_, err := r3.NewSymmetricTensor(math.NaN(), 1, 1, 0, 0, 0)
	require.ErrorIs(t, err, r3.ErrNonFinite)
	_, err = r3.NewSymmetricTensor(1, math.Inf(1), 1, 0, 0, 0)
	require.ErrorIs(t, err, r3.ErrNonFinite)

	singular, err := r3.NewSymmetricTensor(1, 1, 1, 1, 0, 0)
	require.NoError(t, err)
	require.False(t, singular.IsPositiveDefinite())
	_, err = singular.Inverse()
	require.ErrorIs(t, err, r3.ErrNotPositiveDefinite)

	indefinite, err := r3.NewSymmetricTensor(1, 1, -1, 0, 0, 0)
	require.NoError(t, err)
	require.False(t, indefinite.IsPositiveDefinite())
	_, err = indefinite.Inverse()
	require.ErrorIs(t, err, r3.ErrNotPositiveDefinite)

	positive, err := r3.NewSymmetricTensor(1, 1, 1, 0, 0, 0)
	require.NoError(t, err)
	_, err = positive.Rotate(r3.Transform{})
	require.ErrorIs(t, err, r3.ErrNotOrthonormal)
}

func TestSymmetricTensorExactPositiveMinor(t *testing.T) {
	// The first two rows differ only by one representable float64 step.
	// Their determinant must stay positive instead of rounding to zero.
	nearOne := math.Nextafter(1, 0)
	tensor, err := r3.NewSymmetricTensor(1, 1, 1, nearOne, 0, 0)
	require.NoError(t, err)
	require.True(t, tensor.IsPositiveDefinite())
	inverse, err := tensor.Inverse()
	require.NoError(t, err)
	require.True(t, inverse.IsPositiveDefinite())
}

func TestSymmetricTensorRotateRejectsLostPositiveDefiniteness(t *testing.T) {
	tensor, err := r3.NewSymmetricTensor(1, 1e-20, 1, 0, 0, 0)
	require.NoError(t, err)
	require.True(t, tensor.IsPositiveDefinite())
	rotation, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(45))
	require.NoError(t, err)
	rotated, err := tensor.Rotate(rotation)
	require.ErrorIs(t, err, r3.ErrTensorPrecision)
	require.Equal(t, r3.SymmetricTensor{}, rotated)
}
