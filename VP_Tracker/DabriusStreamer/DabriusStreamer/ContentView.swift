//
//  ContentView.swift
//  DabriusStreamer (visionOS)
//
//  Behavior:
//  - Start => launches WS server + AR tracking, hides UI (blank window).
//  - Double-tap (double-pinch) anywhere to reopen controls.
//  - Long-press (~0.5s) fallback to reopen.
//  - Invisible hot-corner (top-left) fallback to reopen.
//  - Streams head-relative wrist pose, wrist roll, pinch + "active" motor flags.
//

import SwiftUI
import Combine
import Foundation

// Uses: Tracker.swift, StreamServer.swift, PoseMath.swift (toArray())

// MARK: - Local IP helper (IPv4 on en* interfaces)
func currentIPv4Address() -> String? {
    var addr: String?
    var ifaddr: UnsafeMutablePointer<ifaddrs>?
    if getifaddrs(&ifaddr) == 0 {
        var p = ifaddr
        while p != nil {
            let iface = p!.pointee
            if let sa = iface.ifa_addr, sa.pointee.sa_family == UInt8(AF_INET) {
                let name = String(cString: iface.ifa_name)
                if name.hasPrefix("en") {
                    var host = [CChar](repeating: 0, count: Int(NI_MAXHOST))
                    var addr_copy = iface.ifa_addr.pointee
                    _ = getnameinfo(&addr_copy, socklen_t(sa.pointee.sa_len),
                                    &host, socklen_t(host.count), nil, 0, NI_NUMERICHOST)
                    addr = String(cString: host)
                    break
                }
            }
            p = iface.ifa_next
        }
        freeifaddrs(ifaddr)
    }
    return addr
}

// MARK: - App State
@MainActor
final class AppState: ObservableObject {
    static let shared = AppState()

    @Published var running = false
    @Published var fps = 30
    @Published var port: UInt16 = 8211
    @Published var status = "Idle"
    @Published var ipAddress: String = "—"
    @Published var uiHidden = false

    // Motor toggles (sent to Python in "active")
    @Published var active: [String: Bool] = [
        "shoulder_pan":  true,
        "shoulder_lift": true,
        "elbow_flex":    true,
        "wrist_flex":    true,
        "wrist_roll":    true,
        "gripper":       true
    ]

    let server = StreamServer()
    let tracker = Tracker()
    private var timer: Timer?

    func start() {
        ipAddress = currentIPv4Address() ?? "Unknown"
        do {
            try server.start(port: port, path: "/stream")
            // refresh once more in case interface changed after start
            ipAddress = currentIPv4Address() ?? ipAddress

            tracker.startVR()              // ARKitSession providers (prompts on first run)
            running = true
            status  = "Streaming on ws://\(ipAddress):\(port)/stream"
            uiHidden = true

            timer = Timer.scheduledTimer(withTimeInterval: 1.0 / Double(fps),
                                         repeats: true) { [weak self] _ in self?.tick() }
        } catch {
            status = "Failed: \(error.localizedDescription)"
        }
    }

    func stop() {
        running = false
        uiHidden = false
        timer?.invalidate(); timer = nil
        tracker.stopVR()
        server.stop()
        status = "Stopped"
    }

    private func tick() {
        guard running, let s = tracker.sample() else { return }

        // Head-relative wrist
        let Hinv = s.head.inverse
        let Wrel = Hinv * s.rightWrist

        let payload: [String: Any] = [
            "schema": "dabrius/v1",
            "t": CACurrentMediaTime(),
            "head_rel": [
                "right_wrist_44": Wrel.toArray(),
                "right_wrist_roll_deg": s.rightWristRollDeg,
                "right_pinch_m": s.rightPinchM as Any
            ],
            "quality": [
                "hand_conf": s.handConf, "head_conf": s.headConf, "occluded": s.occluded
            ],
            "active": active
        ]
        server.broadcast(json: payload)
    }
}

// MARK: - UI
struct ContentView: View {
    @StateObject var app = AppState.shared
    @State private var showInfo = false

    private let motorLabels: [(key: String, title: String, desc: String)] = [
        ("shoulder_pan","Shoulder Pan (ID 1)","Hand left/right"),
        ("shoulder_lift","Shoulder Lift (ID 2)","Hand up/down"),
        ("elbow_flex","Elbow Flex (ID 3)","Hand forward/back"),
        ("wrist_flex","Wrist Flex (ID 4)","Wrist pitch"),
        ("wrist_roll","Wrist Roll (ID 5)","Wrist roll"),
        ("gripper","Gripper (ID 6)","Thumb–index pinch")
    ]

    var body: some View {
        ZStack {
            // ---------- NO-UI STREAMING MODE ----------
            if app.running && app.uiHidden {
                ZStack {
                    // Full transparent, clickable surface
                    Color.clear
                        .ignoresSafeArea()
                        .contentShape(Rectangle())
                        // High-priority double-tap (double-pinch)
                        .highPriorityGesture(
                            TapGesture(count: 2).onEnded {
                                app.uiHidden = false
                            }
                        )
                        // Long-press fallback (pinch and hold)
                        .simultaneousGesture(
                            LongPressGesture(minimumDuration: 0.5).onEnded { _ in
                                app.uiHidden = false
                            }
                        )

                    // Invisible hot-corner (top-left) fallback
                    VStack {
                        HStack {
                            Button(action: { app.uiHidden = false }) {
                                Color.clear.frame(width: 64, height: 64)
                            }
                            .contentShape(Rectangle())
                            .opacity(0.01) // clickable but invisible
                            Spacer()
                        }
                        Spacer()
                    }
                }
            }
            // ---------- CONTROL UI ----------
            else {
                VStack(spacing: 14) {
                    Text("Dabrius Streamer").font(.title2)
                    Text("© Darius Giannoli and Gabriel Taieb")
                        .font(.footnote).foregroundStyle(.secondary)

                    // IP + URL
                    HStack(spacing: 10) {
                        Text("IP: \(app.ipAddress)")
                        Text("URL: ws://\(app.ipAddress):\(app.port)/stream")
                            .font(.footnote).foregroundStyle(.secondary)
                        Button {
                            UIPasteboard.general.string = "ws://\(app.ipAddress):\(app.port)/stream"
                        } label: { Label("Copy URL", systemImage: "doc.on.doc") }
                        .buttonStyle(.bordered)
                    }

                    // FPS + Port + Info
                    HStack(spacing: 16) {
                        Picker("FPS", selection: $app.fps) {
                            Text("20").tag(20); Text("30").tag(30); Text("60").tag(60)
                        }
                        .pickerStyle(.segmented).frame(maxWidth: 240)

                        Text("Port: \(app.port)").font(.callout).foregroundStyle(.secondary)

                        Button { showInfo = true } label: {
                            Label("Info", systemImage: "info.circle")
                        }.buttonStyle(.bordered)
                    }

                    // Motor toggles
                    VStack(alignment: .leading, spacing: 8) {
                        Text("Active Motors").font(.headline)
                        ForEach(motorLabels, id: \.key) { m in
                            Toggle(isOn: Binding(
                                get: { app.active[m.key] ?? true },
                                set: { app.active[m.key] = $0 }
                            )) {
                                VStack(alignment: .leading, spacing: 2) {
                                    Text(m.title)
                                    Text(m.desc).font(.caption).foregroundStyle(.secondary)
                                }
                            }
                            .disabled(app.running) // lock while streaming
                        }
                    }
                    .padding(.top, 6)

                    // Start / Stop
                    HStack(spacing: 12) {
                        Button(app.running ? "Stop" : "Start") {
                            app.running ? app.stop() : app.start()
                        }
                        .buttonStyle(.borderedProminent)
                    }

                    Text(app.status).font(.footnote).foregroundStyle(.secondary)
                }
                .padding(24)
                .sheet(isPresented: $showInfo) {
                    InfoView(ip: app.ipAddress, port: Int(app.port))
                }
                .onAppear { app.ipAddress = currentIPv4Address() ?? "Unknown" }
            }
        }
    }
}

// MARK: - Info sheet
struct InfoView: View {
    @Environment(\.dismiss) private var dismiss
    let ip: String
    let port: Int
    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("How it works").font(.title3).bold()
            Text("This app opens a local WebSocket endpoint (ws://\(ip):\(port)/stream) and streams head-relative wrist pose (4×4), wrist roll (deg), and pinch distance (m) at the selected FPS.")
            Text("20 / 30 / 60 = frames per second. Higher FPS reduces latency and looks smoother, but uses more battery/CPU.")
            Divider()
            Text("Body → Motor mapping").font(.headline)
            VStack(alignment: .leading, spacing: 6) {
                Text("• Hand left/right → Shoulder Pan (ID 1)")
                Text("• Hand up/down → Shoulder Lift (ID 2)")
                Text("• Hand forward/back → Elbow Flex (ID 3)")
                Text("• Wrist pitch → Wrist Flex (ID 4)")
                Text("• Wrist roll → Wrist Roll (ID 5)")
                Text("• Thumb–index pinch → Gripper (ID 6)")
            }
            Spacer()
            Button("Close") { dismiss() }.buttonStyle(.borderedProminent)
        }
        .padding(24)
        .presentationDetents([.medium, .large])
        .interactiveDismissDisabled(false)
    }
}
