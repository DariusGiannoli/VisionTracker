//
//  StreamServer.swift
//  DabriusStreamer
//
//  Created by Gabriel TAIEB on 14/10/2025.
//


import Foundation
import Network

final class StreamServer {
    private var listener: NWListener?
    private var connections: [NWConnection] = []
    private let queue = DispatchQueue(label: "dabrius.ws.server")

    func start(port: UInt16, path: String) throws {
        let params = NWParameters(tls: nil)
        let wsOptions = NWProtocolWebSocket.Options()
        wsOptions.autoReplyPing = true
        params.defaultProtocolStack.applicationProtocols.insert(wsOptions, at: 0)

        let p = NWEndpoint.Port(rawValue: port)!
        let listener = try NWListener(using: params, on: p)
        listener.service = nil

        listener.newConnectionHandler = { [weak self] conn in
            self?.setup(conn: conn)
        }
        listener.stateUpdateHandler = { state in
            print("Listener state: \(state)")
        }
        listener.start(queue: queue)
        self.listener = listener
    }

    func stop() {
        connections.forEach { $0.cancel() }
        connections.removeAll()
        listener?.cancel()
        listener = nil
    }

    private func setup(conn: NWConnection) {
        connections.append(conn)
        conn.stateUpdateHandler = { state in
            print("Conn state: \(state)")
            if case .failed(let e) = state { print("Conn failed: \(e)") }
            if case .cancelled = state { self.connections.removeAll { $0 === conn } }
        }
        conn.start(queue: queue)
        receive(on: conn)
    }

    private func receive(on conn: NWConnection) {
        conn.receiveMessage { [weak self] (data, ctx, isComplete, error) in
            if let error = error { print("recv error: \(error)") }
            // We ignore incoming data; this is a push stream
            self?.receive(on: conn)
        }
    }

    func broadcast(json: [String: Any]) {
        guard !connections.isEmpty else { return }
        guard let data = try? JSONSerialization.data(withJSONObject: json, options: []) else { return }
        let metadata = NWProtocolWebSocket.Metadata(opcode: .text)
        let ctx = NWConnection.ContentContext(identifier: "msg", metadata: [metadata])
        connections.forEach { $0.send(content: data, contentContext: ctx, isComplete: true, completion: .idempotent) }
    }
}
