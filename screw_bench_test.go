package r3_test

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

var (
	screwSink r3.Screw
	// Reuses transformSink and errTransformSink from transform_bench_test.go.
)

func BenchmarkTransformScrew(b *testing.B) {
	transform, other := benchmarkTransformPair(b)
	placed, err := transform.Then(other)
	if err != nil {
		b.Fatal(err)
	}
	b.ReportAllocs()

	for b.Loop() {
		screwSink, errTransformSink = placed.Screw()
	}
}

func BenchmarkScrewAt(b *testing.B) {
	sc := r3.Screw{
		Axis:  r3.NewVec(0, 0, 1),
		Point: r3.NewVec(10, 0, 0),
		Angle: units.Degrees(37),
		Slide: 4.5,
	}
	b.ReportAllocs()

	for b.Loop() {
		transformSink, errTransformSink = sc.At(0.5)
	}
}

func BenchmarkTransformInterpolate(b *testing.B) {
	from, to := benchmarkTransformPair(b)
	b.ReportAllocs()

	for b.Loop() {
		transformSink, errTransformSink = from.Interpolate(to, 0.5)
	}
}
