package r3_test

import (
	"testing"

	"github.com/lestrrat-3d/r3"
)

var (
	vecSink      r3.Vec
	vecFloatSink float64
	vecBoolSink  bool
)

func BenchmarkNewVec(b *testing.B) {
	b.ReportAllocs()

	for b.Loop() {
		vecSink = r3.NewVec(3.25, -4.5, 5.75)
	}
}

func BenchmarkVecAdd(b *testing.B) {
	a := r3.NewVec(3.25, -4.5, 5.75)
	c := r3.NewVec(-1.5, 2.25, 6.75)
	b.ReportAllocs()

	for b.Loop() {
		vecSink = a.Add(c)
	}
}

func BenchmarkVecSub(b *testing.B) {
	a := r3.NewVec(3.25, -4.5, 5.75)
	c := r3.NewVec(-1.5, 2.25, 6.75)
	b.ReportAllocs()

	for b.Loop() {
		vecSink = a.Sub(c)
	}
}

func BenchmarkVecScale(b *testing.B) {
	vector := r3.NewVec(3.25, -4.5, 5.75)
	b.ReportAllocs()

	for b.Loop() {
		vecSink = vector.Scale(1.75)
	}
}

func BenchmarkVecDot(b *testing.B) {
	a := r3.NewVec(3.25, -4.5, 5.75)
	c := r3.NewVec(-1.5, 2.25, 6.75)
	b.ReportAllocs()

	for b.Loop() {
		vecFloatSink = a.Dot(c)
	}
}

func BenchmarkVecCross(b *testing.B) {
	a := r3.NewVec(3.25, -4.5, 5.75)
	c := r3.NewVec(-1.5, 2.25, 6.75)
	b.ReportAllocs()

	for b.Loop() {
		vecSink = a.Cross(c)
	}
}

func BenchmarkVecLen(b *testing.B) {
	vector := r3.NewVec(3.25, -4.5, 5.75)
	b.ReportAllocs()

	for b.Loop() {
		vecFloatSink = vector.Len()
	}
}

func BenchmarkVecNormalize(b *testing.B) {
	vector := r3.NewVec(3.25, -4.5, 5.75)
	b.ReportAllocs()

	for b.Loop() {
		vecSink, vecBoolSink = vector.Normalize()
	}
}
