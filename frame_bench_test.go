package r3_test

import (
	"testing"

	"github.com/lestrrat-3d/r3"
)

var (
	frameVecSink  r3.Vec
	frameBoolSink bool
)

func benchmarkFrame(b *testing.B) r3.Frame {
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

func BenchmarkFrameToWorld(b *testing.B) {
	frame := benchmarkFrame(b)
	local := r3.NewVec(3.25, -4.5, 5.75)
	b.ReportAllocs()

	for b.Loop() {
		frameVecSink = frame.ToWorld(local)
	}
}

func BenchmarkFrameToWorldUV(b *testing.B) {
	frame := benchmarkFrame(b)
	u, v := 3.25, -4.5
	b.ReportAllocs()

	for b.Loop() {
		frameVecSink = frame.ToWorldUV(u, v)
	}
}

func BenchmarkFrameToLocal(b *testing.B) {
	frame := benchmarkFrame(b)
	world := frame.ToWorld(r3.NewVec(3.25, -4.5, 5.75))
	b.ReportAllocs()

	for b.Loop() {
		frameVecSink = frame.ToLocal(world)
	}
}

func BenchmarkFrameIsValid(b *testing.B) {
	frame := benchmarkFrame(b)
	b.ReportAllocs()

	for b.Loop() {
		frameBoolSink = frame.IsValid()
	}
}
