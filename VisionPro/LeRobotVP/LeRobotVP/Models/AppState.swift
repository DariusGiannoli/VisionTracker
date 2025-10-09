//
//  AppState.swift
//  LeRobotVP
//
//  Created by Gabriel TAIEB on 09/10/2025.
//

import Foundation
import simd

final class AppState: ObservableObject {
    // Discovered/selected server
    @Published var serverHost: String = ""
    @Published var controlPort: UInt16 = 8766
    @Published var dataPort: UInt16 = 8765

    // Runtime toggles
    @Published var useRightHand: Bool = true
    @Published var streamingEnabled: Bool = false

    // Calibration: transform from app-origin to robot-base (set on "Calibrate")
    @Published var originFromApp: simd_float4x4 = matrix_identity_float4x4

    // Pinch thresholds (mm) — Double for SwiftUI Slider bindings
    @Published var pinchCloseMM: Double = 25
    @Published var pinchOpenMM:  Double = 55

    // Safety / mapping (also mirrored to Python via control plane)
    @Published var posScale: Double = 1.0
    @Published var eeBounds: [String: [Double]] = [
        "x": [-0.25, 0.25], "y": [0.00, 0.40], "z": [0.00, 0.35]
    ]
    @Published var maxEEStepM: Double = 0.01
    @Published var smoothingAlpha: Double = 0.2
}
