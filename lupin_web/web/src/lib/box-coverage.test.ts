import { describe, expect, it } from 'vitest'

import { frontFaceCoverage } from './box-coverage'
import type { OccupancyGrid, Point32 } from '@/types/ros'

function makeGrid(
  width: number, height: number, resolution: number, ox: number, oy: number, data: number[],
): OccupancyGrid {
  return {
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'map' },
    info: {
      map_load_time: { sec: 0, nanosec: 0 }, resolution, width, height,
      origin: { position: { x: ox, y: oy, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
    },
    data,
  }
}
const pt = (x: number, y: number): Point32 => ({ x, y, z: 0 })

// Box front face (FL→FR) is the vertical segment x=1.25, y from 0.4 to -0.4.
const FACE: Point32[] = [pt(1.25, 0.4), pt(1.25, -0.4), pt(0.85, -0.4), pt(0.85, 0.4)]

describe('frontFaceCoverage', () => {
  it('is ~1 when the whole front face overlaps occupied cells', () => {
    const W = 20, H = 20
    const data = new Array(W * H).fill(0)
    for (let row = 0; row < H; row++) data[row * W + 12] = 100 // occupied col at x≈1.25
    const cov = frontFaceCoverage(FACE, makeGrid(W, H, 0.1, 0, -1, data), { bandM: 0.05 })
    expect(cov).toBeGreaterThan(0.9)
  })

  it('is 0 on an empty grid', () => {
    const W = 20, H = 20
    const cov = frontFaceCoverage(FACE, makeGrid(W, H, 0.1, 0, -1, new Array(W * H).fill(0)))
    expect(cov).toBe(0)
  })

  it('is 0 for a null grid or degenerate face', () => {
    expect(frontFaceCoverage(FACE, null)).toBe(0)
    expect(frontFaceCoverage([pt(0, 0)], makeGrid(4, 4, 0.1, 0, 0, new Array(16).fill(100)))).toBe(0)
  })
})
