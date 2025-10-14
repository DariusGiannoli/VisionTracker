//
//  ContentView.swift
//  DabriusStreamer
//
//  Created by Gabriel TAIEB on 14/10/2025.
//

//
//  ContentView.swift
//  DabriusStreamer
//

//
//  ContentView.swift
//  DabriusStreamer
//

//
//  ContentView.swift
//  DabriusStreamer
//

import SwiftUI
import Combine
import Foundation
import simd
import Network

// Uses: Tracker (from Tracker.swift) and StreamServer (from StreamServer.swift)

// MARK: - Local IP helper (IPv4 on en* interfaces)
func currentIPv4Address() -> String? {
    var addr: String?
    var ifaddr: UnsafeMutablePointer<ifaddrs>?
    if getifaddrs(&ifaddr) == 0 {
        var ptr = ifaddr
        while ptr != nil {
            let iface = ptr!.pointee
            if let sa = iface.ifa_addr, sa.pointee.sa_family == UInt8(AF_INET) {
                let name = String(cString: iface.ifa_name)
                if name.hasPrefix("en") { // Wi-Fi/Ethernet
                    var host = [CChar](repeating: 0, count: Int(NI_MAXHOST))
                    var addr_copy = iface.ifa_addr.pointee
                    _ = getnameinfo(&addr_copy,
                                    socklen_t(sa.pointee.sa_len),
                                    &host, socklen_t(host.count),
                                    nil, 0, NI_NUMERICHOST)
                    addr = String(cString: host)
                    break
                }
            }
            ptr = iface.ifa_next
        }
        freeifaddrs(ifaddr)
    }
    return addr
}

// MARK: - App State

final class AppState: ObservableObject {
    // Streaming
    @Published var running = false
    @Published var fps = 30           // 20 / 30 / 60 Hz
    @Published var port: UInt16 = 8211
    @Published var status = "Idle"
    @Published var ipAddress: String = "—"

    // Motor toggles sent to Python in "active"
    @Published var active: [String: Bool] = [
        "shoulder_pan":  true,
        "shoulder_lift": true,
        "elbow_flex":    true,
        "wrist_flex":    true,
        "wrist_roll":    true,
        "gripper":       true
    ]

    // UI state
    @Published var showInfo = false
    @Published var uiHidden = false   // when true and running => blank window

    let server = StreamServer()
    let tracker = Tracker()
    private var timer: Timer?

    func start() {
        // Resolve IP up-front so user can copy it before/after starting
        ipAddress = currentIPv4Address() ?? "Unknown"
        do {
            try server.start(port: port, path: "/stream")
            running = true
            status = "Streaming on ws://\(ipAddress):\(port)/stream"
            uiHidden = true      // ALWAYS hide UI on Start (MIT-like behavior)
            timer = Timer.scheduledTimer(withTimeInterval: 1.0/Double(fps), repeats: true) { _ in self.tick() }
        } catch {
            status = "Failed: \(error.localizedDescription)"
        }
    }

    func stop() {
        running = false
        uiHidden = false
        timer?.invalidate(); timer = nil
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
                "right_wrist_44": Wrel.toArray(),          // from PoseMath.swift
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
    @StateObject var app = AppState()

    let motorLabels: [(key: String, title: String, desc: String)] = [
        ("shoulder_pan","Shoulder Pan (ID 1)","Hand left/right"),
        ("shoulder_lift","Shoulder Lift (ID 2)","Hand up/down"),
        ("elbow_flex","Elbow Flex (ID 3)","Hand forward/back"),
        ("wrist_flex","Wrist Flex (ID 4)","Wrist pitch"),
        ("wrist_roll","Wrist Roll (ID 5)","Wrist roll"),
        ("gripper","Gripper (ID 6)","Thumb–index pinch")
    ]

    var body: some View {
        ZStack {
            // Streaming-only (blank) — no chrome, no hit-testing
            if app.uiHidden && app.running {
                Color.clear
                    .ignoresSafeArea()
                    .frame(width: 1, height: 1)
                    .opacity(0.001)
                    .allowsHitTesting(false)
            } else {
                // Main controls
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
                            let s = "ws://\(app.ipAddress):\(app.port)/stream"
                            UIPasteboard.general.string = s
                        } label: {
                            Label("Copy URL", systemImage: "doc.on.doc")
                        }
                        .buttonStyle(.bordered)
                    }

                    // FPS + Info
                    HStack(spacing: 16) {
                        Picker("FPS", selection: $app.fps) {
                            Text("20").tag(20)
                            Text("30").tag(30)
                            Text("60").tag(60)
                        }
                        .pickerStyle(.segmented)
                        .frame(maxWidth: 240)

                        Text("Port: \(app.port)")
                            .font(.callout).foregroundStyle(.secondary)

                        Button {
                            app.showInfo = true
                        } label: {
                            Label("Info", systemImage: "info.circle")
                        }
                        .buttonStyle(.bordered)
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
                            .disabled(app.running)
                        }
                    }
                    .padding(.top, 6)

                    // Start/Stop
                    HStack(spacing: 12) {
                        Button(app.running ? "Stop" : "Start") {
                            app.running ? app.stop() : app.start()
                        }
                        .buttonStyle(.borderedProminent)
                    }
                    Text(app.status).font(.footnote).foregroundStyle(.secondary)
                }
                .padding(24)
                .sheet(isPresented: $app.showInfo) { InfoView(app: app) }
                .onAppear { app.ipAddress = currentIPv4Address() ?? "Unknown" }
            }
        }
    }
}

struct InfoView: View {
    @Environment(\.dismiss) private var dismiss
    @ObservedObject var app: AppState
    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("How it works").font(.title3).bold()
            Text("The app opens a local WebSocket endpoint (default ws://\(app.ipAddress):\(app.port)/stream) and streams head-relative wrist pose (4×4), wrist roll (deg), and pinch distance (m) at the selected FPS.")
            Text("20 / 30 / 60 = frames per second. Higher FPS lowers latency and looks smoother but uses more battery and CPU.")
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
            Button("Close") { dismiss() }
                .buttonStyle(.borderedProminent)
        }
        .padding(24)
        .presentationDetents([.medium, .large])
        .interactiveDismissDisabled(false)
    }
}
