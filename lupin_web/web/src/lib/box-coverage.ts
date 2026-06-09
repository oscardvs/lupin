import type { OccupancyGrid, Point32 } from '@/types/ros'

/**
 * Fraction [0,1] of a box's FRONT face (corners[0]→corners[1]) corroborated by
 * the lidar: of `samples` points along that edge, how many have an occupied
 * cell (value ≥ `threshold`) within `bandM` metres. Returns 0 for a null/empty
 * grid or a degenerate (<2-corner) face. Corners are FL, FR, BR, BL — the
 * twin/perception order, so the front face is always corners[0]→corners[1].
 */
export function frontFaceCoverage(
  corners: Point32[],
  grid: OccupancyGrid | null | undefined,
  { samples = 24, bandM = 0.1, threshold = 65 }: {
    samples?: number; bandM?: number; threshold?: number
  } = {},
): number {
  if (!grid || corners.length < 2) return 0
  const { width, height, resolution } = grid.info
  if (width <= 0 || height <= 0 || resolution <= 0) return 0
  const ox = grid.info.origin.position.x
  const oy = grid.info.origin.position.y
  const a = corners[0]
  const b = corners[1]
  const r = Math.max(1, Math.ceil(bandM / resolution))
  let covered = 0
  for (let i = 0; i < samples; i++) {
    const t = (i + 0.5) / samples
    const col = Math.floor((a.x + (b.x - a.x) * t - ox) / resolution)
    const row = Math.floor((a.y + (b.y - a.y) * t - oy) / resolution)
    if (occupiedNear(grid, col, row, r, threshold)) covered++
  }
  return covered / samples
}

function occupiedNear(
  grid: OccupancyGrid, col: number, row: number, r: number, threshold: number,
): boolean {
  const { width, height } = grid.info
  for (let dr = -r; dr <= r; dr++) {
    const rr = row + dr
    if (rr < 0 || rr >= height) continue
    for (let dc = -r; dc <= r; dc++) {
      const cc = col + dc
      if (cc < 0 || cc >= width) continue
      if (grid.data[rr * width + cc] >= threshold) return true
    }
  }
  return false
}
