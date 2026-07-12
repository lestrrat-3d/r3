package examples_test

import (
	"fmt"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func Example_r3_transform() {
	// Place a part: swing it a quarter turn about the Z axis running through
	// (10, 0, 0), then lift it. The angle is typed, so there is no question of
	// whether 90 means degrees or radians.
	spin, err := r3.RotationAround(r3.NewVec(10, 0, 0), r3.NewVec(0, 0, 1), units.Degrees(90))
	if err != nil {
		fmt.Printf("failed to build rotation: %s\n", err)
		return
	}

	// Then composes left to right, in the order the motions happen.
	place := spin.Then(r3.Translation(r3.NewVec(0, 0, 5)))

	// A point carries a position, so it swings about the pivot and rises.
	corner := place.Apply(r3.NewVec(11, 0, 0))
	// A direction does not, so it only turns — ApplyDir never translates. A
	// face normal must go through here, or it comes out garbage.
	normal := place.ApplyDir(r3.NewVec(1, 0, 0))

	fmt.Printf("corner: (%.1f, %.1f, %.1f)\n", corner.X, corner.Y, corner.Z)
	fmt.Printf("normal: (%.1f, %.1f, %.1f)\n", normal.X, normal.Y, normal.Z)

	// The inverse is exact — the transpose of an orthogonal matrix, not a solve.
	back := place.Inverse().Apply(corner)
	fmt.Printf("back:   (%.1f, %.1f, %.1f)\n", back.X, back.Y, back.Z)

	// Output:
	// corner: (10.0, 1.0, 5.0)
	// normal: (0.0, 1.0, 0.0)
	// back:   (11.0, 0.0, 0.0)
}
