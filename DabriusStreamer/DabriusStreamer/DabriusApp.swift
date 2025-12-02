//
//  DabriusApp.swift
//  DabriusStreamer
//
//  Created by Gabriel TAIEB on 14/10/2025.
//

import SwiftUI

@main
struct DabriusApp: App {
    var body: some Scene {
        // The ID "main" lets us programmatically dismiss this window.
        WindowGroup(id: "main") {
            ContentView()
                .environmentObject(AppState.shared)
        }
    }
}

