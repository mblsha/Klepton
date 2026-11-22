import Foundation
import simd

@main
enum GamepadCameraTest {
    static func near(_ actual: SIMD3<Float>, _ expected: SIMD3<Float>) {
        precondition(simd_length(actual - expected) < 0.0001, "\(actual) != \(expected)")
    }

    static func main() {
        let up = SIMD3<Float>(0, 1, 0)
        let forward = SIMD3<Float>(0, 0, -1)
        let head = SIMD3<Float>(2, 1.6, -3)
        let physicalYaw = simd_quatf(angle: .pi / 2, axis: up)
        var physical = simd_float4x4(physicalYaw)
        physical.columns.3 = SIMD4(head, 1)

        var walk = GamepadCamera()
        precondition(walk.update(.init(move: SIMD2(0.1, 0), look: SIMD2(0, 0.1)),
                                 physicalHead: physical, at: 0) == nil)
        precondition(walk.update(.init(move: SIMD2(0.1, 0), look: SIMD2(0, 0.1)),
                                 physicalHead: physical, at: 0.05) == nil)
        var space: CameraTrackingTransform?
        for frame in 1...60 {
            space = walk.update(.init(move: SIMD2(0, 1)), physicalHead: physical,
                                at: 0.05 + Double(frame) / 60)
        }
        let translated = space!.point(head)
        near(translated, head + physicalYaw.act(forward) * 0.85)
        near(walk.update(.init(), physicalHead: physical, at: 1.10)!.point(head), translated)
        near(walk.update(.init(move: SIMD2(0, 1)), physicalHead: physical, at: 3)!.point(head), translated)

        for (left, right, multiplier) in [(true, false, Float(2)), (false, true, 2), (true, true, 4)] {
            var boosted = GamepadCamera()
            _ = boosted.update(.init(), physicalHead: physical, at: 0)
            let input = GamepadCamera.Input(move: SIMD2(0, 1), leftTrigger: left, rightTrigger: right)
            for frame in 1...60 {
                space = boosted.update(input, physicalHead: physical, at: Double(frame) / 60)
            }
            near(space!.point(head), head + physicalYaw.act(forward) * (0.85 * multiplier))
        }

        var turn = GamepadCamera()
        _ = turn.update(.init(), physicalHead: physical, at: 0)
        let hand = head + SIMD3<Float>(0.2, -0.4, -0.6)
        for frame in 1...60 {
            space = turn.update(.init(look: SIMD2(1, 0)), physicalHead: physical, at: Double(frame) / 60)
        }
        let yaw = space!
        near(yaw.point(head), head)
        near(yaw.rotation.act(physicalYaw.act(forward)),
             simd_quatf(angle: (90 - 75) * .pi / 180, axis: up).act(forward))
        precondition(abs(simd_distance(yaw.point(hand), yaw.point(head)) - simd_distance(hand, head)) < 0.0001)

        for frame in 61...180 {
            space = turn.update(.init(look: SIMD2(0, 1)), physicalHead: physical, at: Double(frame) / 60)
        }
        let tilt = space!
        near(tilt.point(head), head)
        precondition(abs(tilt.rotation.act(physicalYaw.act(forward)).y - sin(.pi / 3)) < 0.0001)
        let projectedHand = tilt.matrix * SIMD4(hand, 1)
        near(SIMD3(projectedHand.x, projectedHand.y, projectedHand.z), tilt.point(hand))

        var pitchedMovement = turn
        let viewForward = tilt.rotation.act(physicalYaw.act(forward))
        let ascent = pitchedMovement.update(.init(move: SIMD2(0, 1)), physicalHead: physical, at: 3.05)!
        near(ascent.point(head), head + viewForward * (0.85 * 0.05))
        let back = pitchedMovement.update(.init(move: SIMD2(0, -1)), physicalHead: physical, at: 3.10)!
        near(back.point(head), head)
        let sideways = pitchedMovement.update(.init(move: SIMD2(1, 0)), physicalHead: physical, at: 3.15)!
        near(sideways.point(head), head + simd_normalize(simd_cross(viewForward, up)) * (0.85 * 0.05))

        var physicalLook = GamepadCamera()
        let down = simd_quatf(angle: -.pi / 4, axis: SIMD3<Float>(1, 0, 0))
        var lookingDown = simd_float4x4(physicalYaw * down)
        lookingDown.columns.3 = SIMD4(head, 1)
        _ = physicalLook.update(.init(), physicalHead: lookingDown, at: 0)
        let descended = physicalLook.update(.init(move: SIMD2(0, 1)), physicalHead: lookingDown, at: 0.05)!
        near(descended.point(head), head + (physicalYaw * down).act(forward) * (0.85 * 0.05))

        physical.columns.3.x += 1
        let afterPhysicalStep = turn.update(.init(), physicalHead: physical, at: 3.05)!
        precondition(abs(afterPhysicalStep.point(head + SIMD3(1, 0, 0)).y - head.y) < 0.0001)

        let move = GamepadCamera.Input(move: SIMD2(0, 1), look: SIMD2(1, 0), rightTrigger: true)
        precondition(turn.update(move, physicalHead: physical, at: 3.1) != nil)
        var reset = move
        reset.reset = true
        precondition(turn.update(reset, physicalHead: physical, at: 3.15) == nil)
        precondition(turn.update(reset, physicalHead: physical, at: 3.2) == nil)
        precondition(turn.update(.init(), physicalHead: physical, at: 3.25) == nil)
        let afterReset = turn.update(.init(move: SIMD2(0, 1)), physicalHead: physical, at: 3.3)!
        let movedHead = head + SIMD3<Float>(1, 0, 0)
        near(afterReset.point(movedHead), movedHead + physicalYaw.act(forward) * (0.85 * 0.05))
        near(afterReset.rotation.act(forward), forward)
        print("PASS: camera-relative ascent and descent, level strafe, boosts, hand registration and reset")
    }
}
