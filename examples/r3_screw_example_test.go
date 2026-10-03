package examples_test

import (
	"fmt"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func Example_r3_screw() {
	// A quarter turn about the Z axis through (10, 0, 0), then a lift of 4.
	spin, err := r3.RotationAround(r3.NewVec(10, 0, 0), r3.NewVec(0, 0, 1), units.Degrees(90))
	if err != nil {
		fmt.Printf("failed to build rotation: %s\n", err)
		return
	}
	lift, err := r3.Translation(r3.NewVec(0, 0, 4))
	if err != nil {
		fmt.Printf("failed to build translation: %s\n", err)
		return
	}
	place, err := spin.Then(lift)
	if err != nil {
		fmt.Printf("failed to compose: %s\n", err)
		return
	}

	// Screw reads the motion back as an axis line, an angle about it and a
	// slide along it.
	sc, err := place.Screw()
	if err != nil {
		fmt.Printf("failed to decompose: %s\n", err)
		return
	}
	deg, err := sc.Angle.In(units.Degree)
	if err != nil {
		fmt.Printf("failed to read angle: %s\n", err)
		return
	}
	fmt.Printf("axis:  (%.1f, %.1f, %.1f)\n", sc.Axis.X, sc.Axis.Y, sc.Axis.Z)
	fmt.Printf("point: (%.1f, %.1f, %.1f)\n", sc.Point.X, sc.Point.Y, sc.Point.Z)
	fmt.Printf("angle: %.1f deg\n", deg)
	fmt.Printf("slide: %.1f\n", sc.Slide)

	// Interpolate walks the same screw: halfway there is half the rotation and
	// half the slide.
	half, err := r3.Identity().Interpolate(place, 0.5)
	if err != nil {
		fmt.Printf("failed to interpolate: %s\n", err)
		return
	}
	p := half.Apply(r3.NewVec(11, 0, 0))
	fmt.Printf("half:  (%.1f, %.1f, %.1f)\n", p.X, p.Y, p.Z)

	// Output:
	// axis:  (0.0, 0.0, 1.0)
	// point: (10.0, 0.0, 0.0)
	// angle: 90.0 deg
	// slide: 4.0
	// half:  (10.7, 0.7, 2.0)
}
