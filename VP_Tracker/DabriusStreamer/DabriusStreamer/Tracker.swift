//
//  Sample.swift
//  DabriusStreamer
//
//  Created by Gabriel TAIEB on 14/10/2025.
//


import Foundation
import simd

struct Sample {
    var head: simd_float4x4
    var rightWrist: simd_float4x4
    var rightWristRollDeg: Float
    var rightPinchM: Float?
    var handConf: Float
    var headConf: Float
    var occluded: Bool
}

final class Tracker {
    // TODO: replace with real head + hand tracking (RealityKit/ARKit)
    func sample() -> Sample? {
        // Fake a stable pose for now; plug your real data here
        let I = simd_float4x4(diagonal: SIMD4<Float>(repeating: 1))
        return Sample(head: I,
                      rightWrist: I,
                      rightWristRollDeg: 0,
                      rightPinchM: 0.07,
                      handConf: 0.95,
                      headConf: 0.99,
                      occluded: false)
    }
}
