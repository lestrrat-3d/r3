package r3_test

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

var (
	transformVecSink  r3.Vec
	transformSink     r3.Transform
	transformBoolSink bool
	errTransformSink  error
)

func benchmarkTransformPair(b *testing.B) (r3.Transform, r3.Transform) {
	b.Helper()

	first, err := r3.Rotation(r3.NewVec(1.25, -2.5, 3.75), units.Degrees(37))
	if err != nil {
		b.Fatal(err)
	}
	second, err := r3.Translation(r3.NewVec(4.5, -6.25, 7.75))
	if err != nil {
		b.Fatal(err)
	}
	return first, second
}

func BenchmarkTransformApply(b *testing.B) {
	transform, _ := benchmarkTransformPair(b)
	point := r3.NewVec(8.25, -3.5, 1.75)
	b.ReportAllocs()

	for b.Loop() {
		transformVecSink = transform.Apply(point)
	}
}

func BenchmarkTransformApplyDir(b *testing.B) {
	transform, _ := benchmarkTransformPair(b)
	direction := r3.NewVec(-1.25, 2.5, 4.75)
	b.ReportAllocs()

	for b.Loop() {
		transformVecSink = transform.ApplyDir(direction)
	}
}

func BenchmarkTransformThen(b *testing.B) {
	first, second := benchmarkTransformPair(b)
	b.ReportAllocs()

	for b.Loop() {
		transformSink, errTransformSink = first.Then(second)
	}
}

func BenchmarkTransformInverse(b *testing.B) {
	transform, _ := benchmarkTransformPair(b)
	b.ReportAllocs()

	for b.Loop() {
		transformSink, errTransformSink = transform.Inverse()
	}
}

func BenchmarkTransformIsValid(b *testing.B) {
	transform, _ := benchmarkTransformPair(b)
	b.ReportAllocs()

	for b.Loop() {
		transformBoolSink = transform.IsValid()
	}
}
