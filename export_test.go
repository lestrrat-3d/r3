package r3

// This file is compiled only into the test binary. It exists so the external
// r3_test package can build the states that the exported constructors now refuse
// to produce — a Frame with a non-finite origin, a Transform with a non-finite
// translation — and assert that the last-line-of-defence guards in
// [Frame.IsValid], [Transform.IsValid] and [FromFrame] catch them anyway. The
// fields are unexported, so there is no other way to reach those branches, and a
// guard that cannot be tested is a guard nobody trusts.

// FrameWithOrigin returns a frame with the standard X/Y axes and the given
// origin, bypassing the validation NewFrame performs.
func FrameWithOrigin(origin Vec) Frame {
	return Frame{origin: origin, u: Vec{X: 1}, v: Vec{Y: 1}}
}

// TransformWithTranslation returns a transform with an identity linear part and
// the given translation, bypassing the validation the constructors perform.
func TransformWithTranslation(t Vec) Transform {
	return Transform{ex: Vec{X: 1}, ey: Vec{Y: 1}, ez: Vec{Z: 1}, t: t}
}
