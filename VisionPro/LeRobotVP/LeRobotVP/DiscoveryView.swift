//
//  DiscoveryView.swift
//  LeRobotVP
//
//  Created by Gabriel TAIEB on 09/10/2025.
//

import SwiftUI
import Network

struct DiscoveryView: View {
    @EnvironmentObject var app: AppState
    @State private var candidates: [String] = []
    @State private var browser: NWBrowser?

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("LeRobot Control Connection").font(.title2)

            HStack(spacing: 8) {
                TextField("Server IP (e.g., 192.168.1.50)", text: $app.serverHost)
                    .textFieldStyle(.roundedBorder)
                Button("Connect") { connect() }
                    .disabled(app.serverHost.isEmpty)
            }

            Divider()

            Text("mDNS discovery (optional)").font(.headline)
            List(candidates, id: \.self) { host in
                Button(host) {
                    app.serverHost = host
                    connect()
                }
            }
            .frame(height: 200)

            Divider()

            HStack(spacing: 16) {
                Text("Control Port: \(app.controlPort)")
                Text("Data Port: \(app.dataPort)")
            }

            Divider()

            Text("Pinch thresholds (mm)").font(.headline)
            HStack {
                Text("Close")
                Slider(value: $app.pinchCloseMM, in: 10...80)
                Text(String(format: "%.0f", app.pinchCloseMM))
                Spacer().frame(width: 12)
                Text("Open")
                Slider(value: $app.pinchOpenMM, in: 10...120)
                Text(String(format: "%.0f", app.pinchOpenMM))
            }

            Divider()

            Text("Safety").font(.headline)
            HStack {
                Text("pos scale")
                Slider(value: $app.posScale, in: 0.5...2.0)
                Text(String(format: "%.2f", app.posScale))
            }
            HStack {
                Text("max step (m)")
                Slider(value: $app.maxEEStepM, in: 0.002...0.05)
                Text(String(format: "%.3f", app.maxEEStepM))
            }
            HStack {
                Text("smoothing α")
                Slider(value: $app.smoothingAlpha, in: 0.0...0.8)
                Text(String(format: "%.2f", app.smoothingAlpha))
            }

            Spacer()
        }
        .padding()
        .onAppear { startBonjour() }
        .onDisappear { browser?.cancel() }
    }

    private func connect() {
        // StreamerView (or your network layer) should observe AppState.serverHost
        // and initiate connections to control/data endpoints as needed.
    }

    private func startBonjour() {
        let params = NWParameters.tcp
        // Resolve TXT records too
        let descriptor = NWBrowser.Descriptor.bonjourWithTXTRecord(type: "_lerobot._tcp", domain: nil)
        let b = NWBrowser(for: descriptor, using: params)

        b.browseResultsChangedHandler = { results, _ in
            var found = Set<String>()

            for r in results {
                // r.metadata is NOT optional on your SDK; pattern-match it
                if case let .bonjour(txt) = r.metadata {
                    // NWTXTRecord → dictionary: [String:String]
                    if let host = txt.dictionary["host"], !host.isEmpty {
                        found.insert(host)
                        continue
                    }
                    if let ip = txt.dictionary["ip"], !ip.isEmpty {
                        found.insert(ip)
                        continue
                    }
                }

                // Fallback: show service name if no TXT host/ip
                if case let .service(name, _, _, _) = r.endpoint {
                    found.insert(name)
                }
            }

            DispatchQueue.main.async {
                self.candidates = Array(found).sorted()
            }
        }

        b.stateUpdateHandler = { _ in /* optional logging */ }
        b.start(queue: .main)
        self.browser = b
    }
}
