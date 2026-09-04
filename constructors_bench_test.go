package r3_test

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

var (
	constructorTransformSink r3.Transform
	constructorFrameSink     r3.Frame
	errConstructorSink       error
)

func benchmarkConstructorFrame(b *testing.B) r3.Frame {
	b.Helper()

	frame, err := r3.NewFrame(
		r3.NewVec(2.5, -1.75, 4.25),
		r3.NewVec(1.25, -2.5, 3.75),
		r3.NewVec(-0.5, 2.25, 1.5),
	)
	if err != nil {
		b.Fatal(err)
	}
	return frame
}

func BenchmarkIdentity(b *testing.B) {
	b.ReportAllocs()

	for b.Loop() {
		constructorTransformSink = r3.Identity()
	}
}

func BenchmarkTranslation(b *testing.B) {
	translation := r3.NewVec(4.5, -6.25, 7.75)
	b.ReportAllocs()

	for b.Loop() {
		constructorTransformSink, errConstructorSink = r3.Translation(translation)
	}
}

func BenchmarkRotation(b *testing.B) {
	axis := r3.NewVec(1.25, -2.5, 3.75)
	angle := units.Degrees(37)
	b.ReportAllocs()

	for b.Loop() {
		constructorTransformSink, errConstructorSink = r3.Rotation(axis, angle)
	}
}

func BenchmarkRotationAround(b *testing.B) {
	center := r3.NewVec(2.5, -1.75, 4.25)
	axis := r3.NewVec(1.25, -2.5, 3.75)
	angle := units.Degrees(37)
	b.ReportAllocs()

	for b.Loop() {
		constructorTransformSink, errConstructorSink = r3.RotationAround(center, axis, angle)
	}
}

func BenchmarkNewFrame(b *testing.B) {
	origin := r3.NewVec(2.5, -1.75, 4.25)
	u := r3.NewVec(1.25, -2.5, 3.75)
	v := r3.NewVec(-0.5, 2.25, 1.5)
	b.ReportAllocs()

	for b.Loop() {
		constructorFrameSink, errConstructorSink = r3.NewFrame(origin, u, v)
	}
}

func BenchmarkReflection(b *testing.B) {
	mirror := benchmarkConstructorFrame(b)
	b.ReportAllocs()

	for b.Loop() {
		constructorTransformSink, errConstructorSink = r3.Reflection(mirror)
	}
}

func BenchmarkFromFrame(b *testing.B) {
	frame := benchmarkConstructorFrame(b)
	b.ReportAllocs()

	for b.Loop() {
		constructorTransformSink, errConstructorSink = r3.FromFrame(frame)
	}
}

func BenchmarkFromBasis(b *testing.B) {
	frame := benchmarkConstructorFrame(b)
	basis := r3.Basis{EX: frame.U(), EY: frame.V(), EZ: frame.N()}
	translation := r3.NewVec(4.5, -6.25, 7.75)
	b.ReportAllocs()

	for b.Loop() {
		constructorTransformSink, errConstructorSink = r3.FromBasis(basis, translation)
	}
}
