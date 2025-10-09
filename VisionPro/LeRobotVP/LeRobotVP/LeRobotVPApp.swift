//
//  LeRobotVPApp.swift
//  LeRobotVP
//
//  Created by Gabriel TAIEB on 09/10/2025.
//

import SwiftUI

@main
struct LeRobotVPApp: App {
    @StateObject private var appState = AppState()

    var body: some SwiftUI.Scene {
        // An Immersive Space so we can access ARKit hand tracking
        ImmersiveSpace(id: "MainSpace") {
            StreamerView()
                .environmentObject(appState)
        }
        .upperLimbVisibility(.hidden) // optional: hide system arms

        // A small window to manage connection & settings
        WindowGroup("Control") {
            DiscoveryView()
                .environmentObject(appState)
        }
    }
}

