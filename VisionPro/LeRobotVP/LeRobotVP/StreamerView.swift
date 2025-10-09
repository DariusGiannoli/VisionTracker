import SwiftUI
import RealityKit
import ARKit
import simd

private struct PosePayload: Encodable {
    let seq: Int
    let t_client: Double
    let hand: String
    let pos: [Double]   // [x,y,z]
    let rot: [Double]   // [w,x,y,z]
    let pinch_mm: Double?

    enum CodingKeys: String, CodingKey { case type, seq, t_client, hand, pos, rot, pinch_mm }

    func encode(to encoder: Encoder) throws {
        var c = encoder.container(keyedBy: CodingKeys.self)
        try c.encode("pose", forKey: .type)       // constant
        try c.encode(seq, forKey: .seq)
        try c.encode(t_client, forKey: .t_client)
        try c.encode(hand, forKey: .hand)
        try c.encode(pos, forKey: .pos)
        try c.encode(rot, forKey: .rot)
        try c.encodeIfPresent(pinch_mm, forKey: .pinch_mm)
    }
}

struct StreamerView: View {
    @EnvironmentObject var app: AppState

    private let session = ARKitSession()
    private let hands = HandTrackingProvider()

    @State private var udp = UDPClient()
    @State private var ctl = ControlClient()
    @State private var lastAnchor: HandAnchor?   // keep latest for calibration

    var body: some View {
        RealityView { _ in }
        .task {
            guard HandTrackingProvider.isSupported else { return }
            try? await session.run([hands])
        }
        .task {
            // Connect when a host is already present at launch
            connectIfNeeded()
            sendHelloAndConfig()
        }
        .onChange(of: app.serverHost) { oldValue, newValue in
            guard !newValue.isEmpty else { return }
            connectIfNeeded()
            sendHelloAndConfig()
        }
        
        .task {
            // Main hand-tracking loop
            for await update in hands.anchorUpdates {
                guard app.streamingEnabled else { continue }
                guard update.event == .added || update.event == .updated else { continue }

                let anchor = update.anchor
                lastAnchor = anchor

                if app.useRightHand && anchor.chirality != .right { continue }
                if !app.useRightHand && anchor.chirality != .left  { continue }

                // Wrist pose in app origin
                var worldFromWrist = anchor.originFromAnchorTransform
                worldFromWrist = app.originFromApp * worldFromWrist

                // Position
                let p = worldFromWrist.columns.3
                let pos = [Double(p.x), Double(p.y), Double(p.z)]

                // Rotation quaternion (w,x,y,z)
                let q = quaternion(from: worldFromWrist)
                let rot = [Double(q.0), Double(q.1), Double(q.2), Double(q.3)]

                // Pinch distance (mm) — avoid NaN in JSON
                let pinch = computePinchMM(anchor)
                let handStr = (anchor.chirality == .right ? "right" : "left")
                let now = Date().timeIntervalSince1970
                let seq = (Int(now * 1000) & 0x7fffffff)

                let payload = PosePayload(
                    seq: seq,
                    t_client: now,
                    hand: handStr,
                    pos: pos,
                    rot: rot,
                    pinch_mm: pinch
                )

                if let data = try? JSONEncoder().encode(payload) {
                    udp.send(data)
                }
            }
        }
        .overlay(alignment: .bottom) {
            HStack(spacing: 12) {
                Button(app.streamingEnabled ? "Stop" : "Start") {
                    app.streamingEnabled.toggle()
                    ctl.sendJSON(["type": app.streamingEnabled ? "start" : "stop"])
                }
                Button("Calibrate origin") {
                    if let anchor = lastAnchor {
                        // Inverse of current wrist pose as origin offset
                        app.originFromApp = anchor.originFromAnchorTransform.inverse
                        ctl.sendJSON(["type":"calibrate","mode":"capture_wrist_as_origin"])
                    }
                }
                Toggle(app.useRightHand ? "Right hand" : "Left hand", isOn: $app.useRightHand)
                    .toggleStyle(.switch)
                    .onChange(of: app.useRightHand) { oldValue, newValue in
                        pushHandSetting()
                    }
                Button("E-STOP") {
                    app.streamingEnabled = false
                    ctl.sendJSON(["type":"estop"])
                }
            }
            .padding()
            .glassBackgroundEffect()
        }
    }

    private func connectIfNeeded() {
        guard !app.serverHost.isEmpty else { return }
        udp.connect(host: app.serverHost, port: app.dataPort)
        ctl.connect(host: app.serverHost, port: app.controlPort)
    }

    private func sendHelloAndConfig() {
        guard !app.serverHost.isEmpty else { return }
        ctl.sendJSON(["type":"hello","app":"LeRobotVP","ver":"1.0"])
        pushConfig()
        pushHandSetting()
    }

    private func pushHandSetting() {
        ctl.sendJSON(["type":"configure", "hand": app.useRightHand ? "right" : "left"])
    }

    private func pushConfig() {
        ctl.sendJSON([
            "type":"configure",
            "ee_bounds": app.eeBounds,
            "max_ee_step_m": app.maxEEStepM,
            "smoothing": ["alpha": app.smoothingAlpha],
            "pinch": ["close_mm": app.pinchCloseMM, "open_mm": app.pinchOpenMM],
            "pos_scale": app.posScale
        ])
    }
}

// Quaternion extraction from 4x4 transform (rotation only)
fileprivate func quaternion(from m: simd_float4x4) -> (Double, Double, Double, Double) {
    let c0 = SIMD3<Float>(m.columns.0.x, m.columns.0.y, m.columns.0.z)
    let c1 = SIMD3<Float>(m.columns.1.x, m.columns.1.y, m.columns.1.z)
    let c2 = SIMD3<Float>(m.columns.2.x, m.columns.2.y, m.columns.2.z)
    let r3 = simd_float3x3(columns: (c0, c1, c2))
    let q  = simd_quatf(r3)
    return (Double(q.vector.w), Double(q.vector.x), Double(q.vector.y), Double(q.vector.z))
}


// Safe pinch computation (returns nil instead of NaN)
// --- helper at bottom of the file
fileprivate func computePinchMM(_ anchor: HandAnchor) -> Double? {
    // handSkeleton may be nil → early out
    guard let skel = anchor.handSkeleton else { return nil }

    // Joint names on visionOS: indexFingerTip (not indexTip), thumbTip exists
    let thumbJoint = skel.joint(.thumbTip)
    let indexJoint = skel.joint(.indexFingerTip)

    // Make sure both joints are tracked
    guard thumbJoint.isTracked, indexJoint.isTracked else { return nil }

    // Joint transforms are relative to the hand anchor; bring them to world/app origin
    let wT = anchor.originFromAnchorTransform * thumbJoint.anchorFromJointTransform
    let wI = anchor.originFromAnchorTransform * indexJoint.anchorFromJointTransform

    // Build SIMD3 explicitly to keep the type-checker happy
    let pT = SIMD3<Float>(x: wT.columns.3.x, y: wT.columns.3.y, z: wT.columns.3.z)
    let pI = SIMD3<Float>(x: wI.columns.3.x, y: wI.columns.3.y, z: wI.columns.3.z)

    let distMeters: Float = simd_length(pT - pI)
    let mm = Double(distMeters * 1000.0)
    return mm.isFinite ? mm : nil
}

