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

        physical.columns.3.x += 1
        let afterPhysicalStep = turn.update(.init(), physicalHead: physical, at: 3.05)!
        precondition(abs(afterPhysicalStep.point(head + SIMD3(1, 0, 0)).y - head.y) < 0.0001)
        print("PASS: dead zone, horizontal movement, yaw and pitch, tracked hand registration, disconnect and resume")
    }
}
