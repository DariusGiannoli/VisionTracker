//
//  Pose.swift
//  DabriusStreamer
//
//  Created by Gabriel TAIEB on 14/10/2025.
//


import simd

extension simd_float4x4 {

    /// Row/column element access
    subscript(row r: Int, col c: Int) -> Float {
        get {
            precondition((0..<4).contains(r) && (0..<4).contains(c), "Index out of range")
            let column: SIMD4<Float>
            switch c {
            case 0: column = columns.0
            case 1: column = columns.1
            case 2: column = columns.2
            default: column = columns.3
            }
            switch r {
            case 0: return column.x
            case 1: return column.y
            case 2: return column.z
            default: return column.w
            }
        }
        set {
            precondition((0..<4).contains(r) && (0..<4).contains(c), "Index out of range")
            var column: SIMD4<Float>
            switch c {
            case 0: column = columns.0
            case 1: column = columns.1
            case 2: column = columns.2
            default: column = columns.3
            }
            switch r {
            case 0: column.x = newValue
            case 1: column.y = newValue
            case 2: column.z = newValue
            default: column.w = newValue
            }
            switch c {
            case 0: columns.0 = column
            case 1: columns.1 = column
            case 2: columns.2 = column
            default: columns.3 = column
            }
        }
    }

    /// Row-major nested array (handy for JSON encoding)
    func toArray() -> [[Float]] {
        (0..<4).map { r in (0..<4).map { c in self[row: r, col: c] } }
    }

    /// Flat row-major array if you ever need it
    func toFlatArrayRowMajor() -> [Float] {
        var out: [Float] = []
        out.reserveCapacity(16)
        for r in 0..<4 { for c in 0..<4 { out.append(self[row: r, col: c]) } }
        return out
    }
}

