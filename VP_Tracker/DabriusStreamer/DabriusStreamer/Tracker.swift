//
//  Tracker.swift
//  DabriusStreamer (visionOS)
//

import Foundation
import Combine       // for ObservableObject
import simd
import ARKit         // visionOS: ARKitSession + Providers

// Public sample consumed by ContentView/AppState
struct Sample {
    var head: simd_float4x4
    var rightWrist: simd_float4x4
    var rightWristRollDeg: Float
    var rightPinchM: Float?
    var handConf: Float
    var headConf: Float
    var occluded: Bool
}

@MainActor
final class Tracker: ObservableObject {

    // ARKit session + providers (visionOS)
    private let session = ARKitSession()
    private let world   = WorldTrackingProvider()
    private let hands   = HandTrackingProvider()

    private var worldTask: Task<Void, Never>?
    private var handTask:  Task<Void, Never>?

    // Latest values returned by sample()
    private var lastHead: simd_float4x4 = matrix_identity_float4x4
    private var lastRightWrist: simd_float4x4 = matrix_identity_float4x4
    private var lastRollDeg: Float = 0
    private var lastPinchM: Float? = nil
    private var lastHandConf: Float = 0
    private var lastHeadConf: Float = 0
    private var lastOccluded: Bool = true

    private var started = false

    // Start “VR” tracking; system will prompt for camera on first run
    func startVR() {
        guard !started else { return }
        started = true

        Task { @MainActor in
            do {
                if HandTrackingProvider.isSupported {
                    try await session.run([world, hands])  // will trigger permission on 1st run
                    attachWorldTask()
                    attachHandTask()
                    print("✓ ARKit session started (world + hands)")
                } else {
                    try await session.run([world])
                    attachWorldTask()
                    print("⚠️ Hand tracking not supported; running world only")
                }
            } catch {
                print("❌ ARKitSession.run failed: \(error.localizedDescription)")
                started = false
            }
        }
    }

    func stopVR() {
        guard started else { return }
        started = false
        worldTask?.cancel(); worldTask = nil
        handTask?.cancel();  handTask  = nil
        // stop() is synchronous on your SDK
        session.stop()
        print("✓ ARKit session stopped.")
    }

    func sample() -> Sample? {
        Sample(
            head: lastHead,
            rightWrist: lastRightWrist,
            rightWristRollDeg: lastRollDeg,
            rightPinchM: lastPinchM,
            handConf: lastHandConf,
            headConf: lastHeadConf,
            occluded: lastOccluded
        )
    }

    // MARK: - Provider update streams

    private func attachWorldTask() {
        worldTask = Task.detached { [weak self] in
            guard let self else { return }
            for await update in self.world.anchorUpdates {
                let anchor = update.anchor   // DeviceAnchor
                await MainActor.run {
                    self.lastHead = anchor.originFromAnchorTransform
                    self.lastHeadConf = anchor.isTracked ? 1.0 : 0.5
                }
            }
        }
    }

    private func attachHandTask() {
        handTask = Task.detached { [weak self] in
            guard let self else { return }
            for await update in self.hands.anchorUpdates {
                let anchor = update.anchor   // HandAnchor
                await MainActor.run {
                    self.process(handAnchor: anchor)
                }
            }
        }
    }

    // MARK: - Hand processing (right hand)

    private func process(handAnchor: HandAnchor) {
        guard handAnchor.chirality == .right,
              let skeleton = handAnchor.handSkeleton else {
            lastHandConf = max(0.0, lastHandConf - 0.1)
            lastOccluded = true
            return
        }

        // World-from-anchor
        let T_wa = handAnchor.originFromAnchorTransform

        // Wrist joint world pose (joint is non-optional on this SDK)
        let wristJoint   = skeleton.joint(.wrist)
        let T_aw_wrist   = wristJoint.anchorFromJointTransform   // anchor (hand) space
        let T_wwrist     = T_wa * T_aw_wrist                     // world space
        lastRightWrist   = T_wwrist

        // Wrist roll (deg) from rotation matrix
        let R = simd_float3x3(columns: (
            SIMD3(T_wwrist.columns.0.x, T_wwrist.columns.0.y, T_wwrist.columns.0.z),
            SIMD3(T_wwrist.columns.1.x, T_wwrist.columns.1.y, T_wwrist.columns.1.z),
            SIMD3(T_wwrist.columns.2.x, T_wwrist.columns.2.y, T_wwrist.columns.2.z)
        ))
        lastRollDeg = extractRollDegrees(R)

        // Pinch distance (thumbTip ↔ indexFingerTip), in meters
        let T_aw_thumb  = skeleton.joint(.thumbTip).anchorFromJointTransform
        let T_aw_index  = skeleton.joint(.indexFingerTip).anchorFromJointTransform
        let thumbWorld  = (T_wa * T_aw_thumb).columns.3
        let indexWorld  = (T_wa * T_aw_index).columns.3
        let dx = indexWorld.x - thumbWorld.x
        let dy = indexWorld.y - thumbWorld.y
        let dz = indexWorld.z - thumbWorld.z
        lastPinchM = sqrtf(dx*dx + dy*dy + dz*dz)

        lastHandConf = 0.9
        lastOccluded = false
    }

    // MARK: - Math

    private func extractRollDegrees(_ R: simd_float3x3) -> Float {
        // Roll around local X axis; adjust to your convention if needed.
        let roll = atan2f(R[2,1], R[2,2])
        return roll * 180.0 / .pi
    }
}
