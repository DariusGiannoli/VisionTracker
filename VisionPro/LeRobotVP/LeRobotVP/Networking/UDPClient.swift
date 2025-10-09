//
//  UDPClient.swift
//  LeRobotVP
//
//  Created by Gabriel TAIEB on 09/10/2025.
//


import Foundation
import Network

final class UDPClient {
    private var connection: NWConnection?

    func connect(host: String, port: UInt16) {
        let params = NWParameters.udp
        let h = NWEndpoint.Host(host)
        let p = NWEndpoint.Port(rawValue: port)!
        let c = NWConnection(host: h, port: p, using: params)
        c.stateUpdateHandler = { _ in }
        c.start(queue: .main)
        self.connection = c
    }

    func send(_ data: Data) {
        connection?.send(content: data, completion: .contentProcessed { _ in })
    }
}
