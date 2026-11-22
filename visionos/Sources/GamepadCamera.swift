import Foundation
import simd

/// The guest's virtual tracking space relative to physical visionOS tracking.
/// Compositor Services still receives the physical device anchor.
struct CameraTrackingTransform {
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

struct GamepadCamera {
    struct Input {
        var move: SIMD2<Float> = .zero
        var look: SIMD2<Float> = .zero
        var leftTrigger = false
        var rightTrigger = false
        var reset = false

        var speedMultiplier: Float {
            switch (leftTrigger, rightTrigger) {
            case (true, true): return 4
            case (false, false): return 1
            default: return 2
            }
        }
    }

    private static let deadZone: Float = 0.18
    private static let walkSpeed: Float = 0.85
    private static let yawSpeed: Float = 75 * .pi / 180
    private static let pitchSpeed: Float = 55 * .pi / 180
    private static let pitchLimit: Float = 60 * .pi / 180
    private static let up = SIMD3<Float>(0, 1, 0)

    private var previousTime: TimeInterval?
    private var yaw: Float = 0
    private var pitch: Float = 0
    private var physicalYaw: Float = 0
    private var offset: SIMD3<Float> = .zero

    mutating func update(_ input: Input, physicalHead: simd_float4x4,
                         at time: TimeInterval) -> CameraTrackingTransform? {
        if input.reset {
            yaw = 0
            pitch = 0
            offset = .zero
            previousTime = time
            return nil
        }

        let head = SIMD3<Float>(physicalHead.columns.3.x, physicalHead.columns.3.y,
                               physicalHead.columns.3.z)
        let back = physicalHead.columns.2
        if simd_length_squared(SIMD2(back.x, back.z)) > 0.0001 {
            physicalYaw = atan2(back.x, back.z)
        }

        if let previousTime, time > previousTime, time - previousTime <= 0.1 {
            let dt = Float(time - previousTime)
            let look = Self.filtered(input.look)
            let move = Self.filtered(input.move)
            let oldYaw = simd_quatf(angle: yaw, axis: Self.up)
            yaw = (yaw - look.x * Self.yawSpeed * dt).remainder(dividingBy: 2 * .pi)
            pitch = min(Self.pitchLimit, max(-Self.pitchLimit, pitch + look.y * Self.pitchSpeed * dt))
            let newYaw = simd_quatf(angle: yaw, axis: Self.up)
            let heading = simd_quatf(angle: yaw + physicalYaw, axis: Self.up)
            // Turn around the current head position, even when the player has
            // walked away from ARKit's origin. Walk along the view's horizontal plane.
            offset += oldYaw.act(head) - newYaw.act(head)
            let travel = Self.walkSpeed * input.speedMultiplier * dt
            offset += heading.act(SIMD3(move.x, 0, -move.y)) * travel
        }
        previousTime = time

        guard yaw != 0 || pitch != 0 || offset != .zero else { return nil }
        let turn = simd_quatf(angle: yaw, axis: Self.up)
        let right = simd_quatf(angle: physicalYaw, axis: Self.up).act(SIMD3(1, 0, 0))
        let rotation = turn * simd_quatf(angle: pitch, axis: right)
        let virtualHead = turn.act(head) + offset
        // Pitch tilts the view and hands about the head without moving the
        // head up or down as the player walks in physical space.
        return CameraTrackingTransform(rotation: rotation,
                                       translation: virtualHead - rotation.act(head))
    }

    private static func filtered(_ axes: SIMD2<Float>) -> SIMD2<Float> {
        let length = simd_length(axes)
        guard length.isFinite, length > deadZone else { return .zero }
        let magnitude = (min(length, 1) - deadZone) / (1 - deadZone)
        return axes * (magnitude / length)
    }
}
