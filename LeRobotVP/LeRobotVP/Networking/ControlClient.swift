//
//  ControlClient.swift
//  LeRobotVP
//
//  Created by Gabriel TAIEB on 09/10/2025.
//

import Foundation
import Network

final class ControlClient {
    private var connection: NWConnection?

    func connect(host: String, port: UInt16) {
        let h = NWEndpoint.Host(host)
        let p = NWEndpoint.Port(rawValue: port)!
        let c = NWConnection(host: h, port: p, using: .tcp)
        c.stateUpdateHandler = { _ in }
        c.start(queue: .main)
        connection = c
    }

    func sendJSON(_ json: [String: Any]) {
        guard let data = try? JSONSerialization.data(withJSONObject: json) else { return }
        // newline-delimited JSON for easy framing
        let payload = data + Data("\n".utf8)
        connection?.send(content: payload, completion: .contentProcessed { _ in })
    }
}
