import { Canvas, useFrame } from '@react-three/fiber'
import { animated, useSpring } from '@react-spring/three'
import { useEffect, useMemo, useRef, useState } from 'react'
import * as THREE from 'three'

import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from '@/components/ui/card'
import { useTwinState } from '@/lib/twin'
import {
  computeTulipHealth,
  formatDriver,
  type TulipHealth,
  type TulipState,
} from '@/lib/tulip-health'
import { cn } from '@/lib/utils'

/**
 * Ambient health indicator for the greenhouse. Renders a single tulip
 * whose pose, colour, and motion track the worst-deviation across all
 * observed tags. Decorative but data-driven — operators glance at it for
 * a vibe-check before drilling into the Greenhouse State table.
 *
 * Build-rolled from THREE primitives (cylinder stem, plane petals, plane
 * leaves) — ~500 verts, ortho camera, no shadows. If the r3f stack ever
 * feels overweight we can swap for a 2D SVG with the same state machine.
 */
export function TulipHealthIndicator({ className }: { className?: string }) {
  const twin = useTwinState()
  const rawHealth = useMemo(
    () => computeTulipHealth(twin?.tags ?? []),
    [twin],
  )
  // Smooth the score with an EMA so a single transient outlier doesn't
  // flip the indicator. Without this the demo cries wolf the moment any
  // sensor reading brushes its ideal-range edge — the brief said "one bad
  // *plant* drives state", not "one bad *sample*". Effective window is
  // ~3 snapshots at α=0.4, ~4 s at the twin's 1 Hz publish rate.
  const health = useEmaSmoothed(rawHealth)

  const [hovered, setHovered] = useState(false)

  return (
    <Card
      className={cn('flex flex-col', className)}
      onMouseEnter={() => setHovered(true)}
      onMouseLeave={() => setHovered(false)}
    >
      <CardHeader>
        <CardTitle>
          Greenhouse Health
          <span className="tag tag-accent ml-auto">TULIP-01</span>
        </CardTitle>
        <CardDescription className="font-mono text-xs">
          {health.state === 'no_data'
            ? 'awaiting first observation…'
            : `${health.state} · score ${health.score.toFixed(2)}`}
        </CardDescription>
      </CardHeader>
      <CardContent className="relative flex flex-1 items-center justify-center">
        <div className="relative aspect-square w-[200px] max-w-full">
          <Canvas
            orthographic
            camera={{
              position: [4.0, 2.6, 5.0],   // ~15° elevation, 20° azimuth
              zoom: 80,
              near: 0.1, far: 100,
            }}
            gl={{ antialias: true, alpha: true }}
            dpr={[1, 2]}
          >
            <ambientLight intensity={0.4} />
            <directionalLight position={[3, 4, 2]} intensity={0.85} />
            <Tulip state={health.state} />
          </Canvas>
          {hovered && (
            <div className="pointer-events-none absolute bottom-1 left-1 right-1 rounded-sm border border-hairline bg-card/90 px-2 py-1 text-[10px] backdrop-blur">
              <div className="font-mono">
                Health: <span className="text-foreground">{health.score.toFixed(2)}</span>
                {' · '}
                <span className="text-foreground">{health.state}</span>
              </div>
              {health.driver && (
                <div className="mt-0.5 truncate text-muted-foreground">
                  driven by {formatDriver(health.driver)}
                </div>
              )}
            </div>
          )}
        </div>
      </CardContent>
    </Card>
  )
}

/**
 * Exponential-moving-average smoother for the tulip score. Re-thresholds
 * the smoothed score to derive the displayed state, while passing the
 * latest worst-driver string through unchanged (the operator wants to
 * see the *current* worst tag in the tooltip, even mid-smoothing).
 *
 * α = 0.4 → effective window ≈ 1/α ≈ 2.5 snapshots, i.e. ~2-3 s at the
 * twin's 1 Hz publish rate. Tight enough to react quickly to real
 * stress, loose enough to ignore single noisy frames.
 */
function useEmaSmoothed(raw: TulipHealth): TulipHealth {
  const ALPHA = 0.4
  const scoreRef = useRef<number | null>(null)

  // Reset the EMA when twin loses all data — otherwise the smoothed
  // score would keep falling toward 0 from whatever value it held.
  useEffect(() => {
    if (raw.state === 'no_data') scoreRef.current = null
  }, [raw.state])

  if (raw.state === 'no_data') {
    return raw
  }
  const prev = scoreRef.current
  const next = prev == null ? raw.score : ALPHA * raw.score + (1 - ALPHA) * prev
  scoreRef.current = next
  const state: TulipState =
    next >= 0.7 ? 'critical'
    : next >= 0.3 ? 'stressed'
    : 'healthy'
  return { state, score: next, driver: raw.driver }
}

/** Per-state visual targets driven through @react-spring/three. */
interface TulipPose {
  stemTilt: number
  petalDroop: number
  petalColor: string
  leafColor: string
  saturation: number
  motion: number  // 1 = vibrant sway, 0 = still
  opacity: number
}

const POSE_BY_STATE: Record<TulipState, TulipPose> = {
  healthy: {
    stemTilt: 0,
    petalDroop: 0,
    petalColor: '#dc3545',
    leafColor: '#4a8c3a',
    saturation: 1,
    motion: 1,
    opacity: 1,
  },
  stressed: {
    stemTilt: 0.18,
    petalDroop: 0.35,
    petalColor: '#b56267',
    leafColor: '#7a8c5a',
    saturation: 0.7,
    motion: 0.5,
    opacity: 1,
  },
  critical: {
    stemTilt: 0.45,
    petalDroop: 0.85,
    petalColor: '#7a4a36',
    leafColor: '#6b6238',
    saturation: 0.4,
    motion: 0.15,
    opacity: 1,
  },
  no_data: {
    stemTilt: 0,
    petalDroop: 0.2,
    petalColor: '#8a8a8a',
    leafColor: '#7a7a7a',
    saturation: 0.0,
    motion: 0,
    opacity: 0.45,
  },
}

function Tulip({ state }: { state: TulipState }) {
  const target = POSE_BY_STATE[state]

  // Tween the whole pose 800 ms when state changes — soft transitions
  // avoid the jarring "wilt-snap" you'd get from instant updates.
  const sp = useSpring({
    to: target,
    config: { mass: 1.4, tension: 120, friction: 22 },
  })

  // Idle sway — mounts on the stem and leaves so the whole plant moves
  // together. Frequency is slow (~0.5 Hz at full motion).
  const swayRef = useRef<THREE.Group>(null)
  useFrame((s) => {
    if (!swayRef.current) return
    const t = s.clock.getElapsedTime()
    const m = (sp.motion.get() as unknown as number)
    swayRef.current.rotation.z = Math.sin(t * 0.5) * 0.06 * m
  })

  return (
    <animated.group rotation-z={sp.stemTilt}>
      <group ref={swayRef}>
        {/* Stem */}
        <mesh position={[0, 1.2, 0]} castShadow={false} receiveShadow={false}>
          <cylinderGeometry args={[0.05, 0.07, 2.4, 16]} />
          <animated.meshStandardMaterial
            color={sp.leafColor}
            roughness={0.6}
            metalness={0.0}
            transparent
            opacity={sp.opacity}
          />
        </mesh>

        {/* Leaves — broad blades on either side of the stem */}
        <mesh position={[0.35, 0.7, 0]} rotation={[0, 0, -0.6]}>
          <planeGeometry args={[0.55, 0.9]} />
          <animated.meshStandardMaterial
            color={sp.leafColor}
            side={THREE.DoubleSide}
            transparent
            opacity={sp.opacity}
            roughness={0.7}
          />
        </mesh>
        <mesh position={[-0.32, 0.95, 0]} rotation={[0, Math.PI, 0.5]}>
          <planeGeometry args={[0.45, 0.7]} />
          <animated.meshStandardMaterial
            color={sp.leafColor}
            side={THREE.DoubleSide}
            transparent
            opacity={sp.opacity}
            roughness={0.7}
          />
        </mesh>

        {/* Petals — six planes around the head, droop pose blends in */}
        <Petals droop={sp.petalDroop} color={sp.petalColor} opacity={sp.opacity} />
      </group>
    </animated.group>
  )
}

/**
 * Six petals arranged radially around the bloom centre. The droop value
 * folds them downward — at droop=0 they fan upward in a tulip cup; at
 * droop=1 they collapse to nearly vertical-down (wilted).
 */
// react-spring/three's SpringValue types are deeply parameterised. The
// petals don't care — they consume them via .get() inside useFrame and
// pass the SpringValues straight through to animated material props. Use
// `any` rather than chasing the exact generic signature.
// eslint-disable-next-line @typescript-eslint/no-explicit-any
type SpringAny = any

function Petals({
  droop, color, opacity,
}: {
  droop: SpringAny
  color: SpringAny
  opacity: SpringAny
}) {
  const groupRef = useRef<THREE.Group>(null)
  const PETAL_COUNT = 6
  useFrame(() => {
    const g = groupRef.current
    if (!g) return
    const d = droop.get()
    for (let i = 0; i < g.children.length; i++) {
      const child = g.children[i] as THREE.Mesh
      const baseTilt = -0.65            // healthy "fan up" tilt off vertical
      const wiltedTilt = 0.85            // collapse-down tilt
      const tilt = baseTilt + (wiltedTilt - baseTilt) * d
      child.rotation.x = tilt
    }
  })
  return (
    <group ref={groupRef} position={[0, 2.55, 0]}>
      {Array.from({ length: PETAL_COUNT }).map((_, i) => {
        const yaw = (i / PETAL_COUNT) * Math.PI * 2
        return (
          <group key={i} rotation={[0, yaw, 0]}>
            <mesh position={[0, 0.18, 0.18]}>
              <planeGeometry args={[0.35, 0.55]} />
              <animated.meshStandardMaterial
                color={color}
                side={THREE.DoubleSide}
                transparent
                opacity={opacity}
                roughness={0.55}
              />
            </mesh>
          </group>
        )
      })}
    </group>
  )
}
