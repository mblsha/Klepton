import simd

/// The guest's virtual tracking space relative to physical visionOS tracking.
/// Compositor Services still receives the physical device anchor.
struct GuestTrackingTransform {
    let rotation: simd_quatf
    let translation: SIMD3<Float>

    var matrix: simd_float4x4 {
        var result = simd_float4x4(rotation)
        result.columns.3 = SIMD4(translation, 1)
        return result
    }

    func point(_ position: SIMD3<Float>) -> SIMD3<Float> {
        rotation.act(position) + translation
    }

    func pose(_ pose: (SIMD3<Float>, simd_quatf)) -> (SIMD3<Float>, simd_quatf) {
        (point(pose.0), rotation * pose.1)
    }
}
