package r3

import "math"

// zeroLen is the threshold below which a vector is treated as having no
// direction. It is a divide-by-zero guard for Normalize, not a geometric
// tolerance.
const zeroLen = 1e-12

// Vec is a vector (or point) in 3-space: pure transient coordinate math. It
// carries no document state.
type Vec struct {
	X, Y, Z float64
}

// NewVec returns the vector (x, y, z).
func NewVec(x, y, z float64) Vec { return Vec{X: x, Y: y, Z: z} }

// Add returns v + o.
func (v Vec) Add(o Vec) Vec { return Vec{v.X + o.X, v.Y + o.Y, v.Z + o.Z} }

// Sub returns v − o.
func (v Vec) Sub(o Vec) Vec { return Vec{v.X - o.X, v.Y - o.Y, v.Z - o.Z} }

// Scale returns v scaled by s.
func (v Vec) Scale(s float64) Vec { return Vec{v.X * s, v.Y * s, v.Z * s} }

// Dot returns the dot product v · o.
func (v Vec) Dot(o Vec) float64 { return v.X*o.X + v.Y*o.Y + v.Z*o.Z }

// Cross returns the cross product v × o.
func (v Vec) Cross(o Vec) Vec {
	return Vec{
		v.Y*o.Z - v.Z*o.Y,
		v.Z*o.X - v.X*o.Z,
		v.X*o.Y - v.Y*o.X,
	}
}

// Len returns the Euclidean length of v.
func (v Vec) Len() float64 { return math.Sqrt(v.Dot(v)) }

// Equal reports whether v and o agree componentwise within tol. It is a
// tolerant comparison for floating-point results, not an exact one: two vectors
// that should be equal rarely are, bit for bit, once any arithmetic has touched
// them.
func (v Vec) Equal(o Vec, tol float64) bool {
	return math.Abs(v.X-o.X) <= tol &&
		math.Abs(v.Y-o.Y) <= tol &&
		math.Abs(v.Z-o.Z) <= tol
}

// Normalize returns the unit vector along v and true, or the zero vector and
// false when v is (near-)zero or has a NaN length. The boolean is
// deliberate: unlike a floor-against-zero helper, Normalize never fabricates a
// non-unit direction from a zero vector — callers must handle the false case.
//
// The guard is phrased positively (!(l >= zeroLen)): a NaN length compares
// false whichever way the test is written, so it must fail the accept test
// rather than pass a reject test.
func (v Vec) Normalize() (Vec, bool) {
	l := v.Len()
	if !(l >= zeroLen) {
		return Vec{}, false
	}
	return v.Scale(1 / l), true
}
