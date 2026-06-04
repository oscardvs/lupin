import { ContactShadows } from '@react-three/drei'
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
 * The bloom is a *closed goblet*: six curved petal shells in two offset
 * whorls (3 outer + 3 inner) that cup inward and overlap into a
 * recognizable tulip cup, rather than a flat fan. Stem + leaves are swept
 * along Bézier curves so nothing reads as flat cardboard. Petals carry a
 * waxy sheen + faint translucency; leaves are matte. A soft three-point
 * rig + contact shadow + vignette give the ~260px card a flattering,
 * still-life composition.
 *
 * Cost stays modest (~1.5k verts, ortho camera, single blurred contact
 * shadow). If the r3f stack ever feels overweight we can swap for a 2D
 * SVG with the same state machine.
 */
export function TulipHealthIndicator({ className }: { className?: string }) {
  const { state: twin } = useTwinState()
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
        <div className="relative aspect-square w-[210px] max-w-full">
          <Canvas
            orthographic
            camera={{
              position: [2.4, 1.7, 4.6],
              zoom: 86,
              near: 0.1, far: 100,
            }}
            gl={{
              antialias: true,
              alpha: true,
              toneMapping: THREE.ACESFilmicToneMapping,
              toneMappingExposure: 1.15,
            }}
            dpr={[1, 2]}
          >
            <TulipScene state={health.state} />
          </Canvas>
          {/* Soft radial vignette — pulls focus to the bloom and seats the
              plant into the card without a hard frame. Pure CSS, no cost. */}
          <div
            aria-hidden
            className="pointer-events-none absolute inset-0 rounded-sm"
            style={{
              background:
                'radial-gradient(120% 95% at 50% 38%, transparent 52%, hsl(var(--card)/0.55) 100%)',
            }}
          />
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
    petalColor: '#e23b4e',
    leafColor: '#4f9438',
    saturation: 1,
    motion: 1,
    opacity: 1,
  },
  stressed: {
    stemTilt: 0.18,
    petalDroop: 0.4,
    petalColor: '#bc5f63',
    leafColor: '#7e8a52',
    saturation: 0.7,
    motion: 0.5,
    opacity: 1,
  },
  critical: {
    stemTilt: 0.48,
    petalDroop: 0.92,
    petalColor: '#7d4632',
    leafColor: '#6a5f33',
    saturation: 0.4,
    motion: 0.12,
    opacity: 1,
  },
  no_data: {
    stemTilt: 0,
    petalDroop: 0.22,
    petalColor: '#8a8a8a',
    leafColor: '#7a7a7a',
    saturation: 0.0,
    motion: 0,
    opacity: 0.45,
  },
}

/**
 * Scene wrapper: lighting rig + grounded plant. Kept separate from the
 * Canvas host so the lights/shadow live in the same subtree as the geometry
 * and we can reason about composition in one place.
 *
 * Lighting is a soft three-point still-life rig:
 *   - hemisphere for a cool-sky / warm-ground ambient wash
 *   - key spot, warm, high front-right, with a gentle penumbra
 *   - cool fill from the back-left to keep shadow sides from going muddy
 *   - a low warm rim to pop the bloom's silhouette off the dark card
 */
function TulipScene({ state }: { state: TulipState }) {
  return (
    <>
      <hemisphereLight args={['#eaf4ff', '#243016', 0.55]} />
      <spotLight
        position={[3.2, 5.2, 3.4]}
        angle={0.7}
        penumbra={0.9}
        intensity={42}
        distance={20}
        decay={2}
        color="#fff3e2"
      />
      <directionalLight
        position={[-2.6, 1.4, -1.8]}
        intensity={0.5}
        color="#cfe6ff"
      />
      {/* Warm rim, low and behind — separates petals from the card */}
      <pointLight position={[-0.4, 1.0, -2.4]} intensity={6} color="#ffcaa0" />

      <Tulip state={state} />

      {/* Soft contact shadow grounds the plant; matches RobotTwin's look. */}
      <ContactShadows
        position={[0, -0.02, 0]}
        opacity={0.42}
        scale={3.4}
        blur={3.0}
        far={2.2}
        resolution={256}
        color="#000000"
      />
    </>
  )
}

function Tulip({ state }: { state: TulipState }) {
  const target = POSE_BY_STATE[state]

  // Tween the whole pose 800 ms when state changes — soft transitions
  // avoid the jarring "wilt-snap" you'd get from instant updates.
  const sp = useSpring({
    to: target,
    config: { mass: 1.4, tension: 120, friction: 22 },
  })

  // Idle sway — mounts on the stem and bloom so the whole plant moves
  // together. Two slightly detuned sines on z (lean) and a touch on x
  // (breathing toward/away from camera) read as a living plant in a faint
  // draught rather than a metronome. Scaled by `motion` so a wilting tulip
  // goes still.
  const swayRef = useRef<THREE.Group>(null)
  useFrame((s) => {
    if (!swayRef.current) return
    const t = s.clock.getElapsedTime()
    const m = sp.motion.get() as unknown as number
    swayRef.current.rotation.z = (Math.sin(t * 0.5) + Math.sin(t * 0.83) * 0.4) * 0.045 * m
    swayRef.current.rotation.x = Math.sin(t * 0.37 + 1.1) * 0.02 * m
  })

  return (
    <animated.group rotation-z={sp.stemTilt} position={[0, -1.05, 0]}>
      <group ref={swayRef}>
        <Stem color={sp.leafColor} opacity={sp.opacity} />
        <Leaves color={sp.leafColor} opacity={sp.opacity} />
        {/* Bloom rides at the top of the stem; droop pose folds the petals. */}
        <Bloom droop={sp.petalDroop} color={sp.petalColor} opacity={sp.opacity} />
      </group>
    </animated.group>
  )
}

// react-spring/three's SpringValue types are deeply parameterised. The
// children don't care — they consume them via .get() inside useFrame and
// pass the SpringValues straight through to animated material props. Use
// `any` rather than chasing the exact generic signature.
// eslint-disable-next-line @typescript-eslint/no-explicit-any
type SpringAny = any

/* ------------------------------------------------------------------ stem */

/**
 * Stem swept along a gentle S-curve so it has organic life instead of a
 * dead-straight cylinder. TubeGeometry with a slight taper-by-radius (we
 * fake the taper with a thin tube + a small base flare mesh — TubeGeometry
 * is constant-radius, and the curvature sells it more than the taper). The
 * curve leans the bloom very slightly forward into the key light.
 */
function useStemGeometry(): THREE.TubeGeometry {
  return useMemo(() => {
    const curve = new THREE.CubicBezierCurve3(
      new THREE.Vector3(0.0, 0.0, 0.0),
      new THREE.Vector3(-0.12, 0.85, 0.05),
      new THREE.Vector3(0.1, 1.7, 0.04),
      new THREE.Vector3(0.0, 2.5, 0.0),
    )
    return new THREE.TubeGeometry(curve, 40, 0.045, 12, false)
  }, [])
}

function Stem({ color, opacity }: { color: SpringAny; opacity: SpringAny }) {
  const geometry = useStemGeometry()
  return (
    <mesh geometry={geometry}>
      <animated.meshStandardMaterial
        color={color}
        roughness={0.7}
        metalness={0.0}
        transparent
        opacity={opacity}
      />
    </mesh>
  )
}

/* ----------------------------------------------------------------- leaves */

/**
 * A single sword-shaped tulip leaf as a curved surface. Built parametric:
 *   - v ∈ [0,1] runs base→tip; width swells then tapers to a point
 *   - the blade arches lengthwise (rises then nods over) — the spine curve
 *   - each cross-section is *cupped* (a shallow V across the width) so the
 *     leaf catches light on its channel instead of reading as a flat plane
 *
 * Returned in the leaf's local frame with the base at the origin and the
 * blade growing up +Y; callers rotate/position/scale and mirror via scale.x.
 */
function useLeafGeometry(): THREE.BufferGeometry {
  return useMemo(() => {
    const COLS = 7 // across the width
    const ROWS = 18 // along the length
    const positions: number[] = []
    const normals: number[] = []
    const uvs: number[] = []
    const indices: number[] = []

    const LENGTH = 1.5

    for (let r = 0; r <= ROWS; r++) {
      const v = r / ROWS
      // Width profile: narrow base, swell ~30% up, taper to a point.
      const width = 0.34 * Math.sin(Math.pow(v, 0.7) * Math.PI) * (1 - 0.15 * v)
      // Spine: arch up then nod over at the tip (gravity on a long blade).
      const spineY = LENGTH * (v - 0.22 * v * v)
      const spineZ = 0.12 * Math.sin(v * Math.PI * 0.85) - 0.55 * v * v
      // Cup depth across the section — deeper mid-blade, flat at the tip.
      const cup = 0.16 * Math.sin(v * Math.PI)
      for (let c = 0; c <= COLS; c++) {
        const u = c / COLS // 0..1 across
        const x = (u - 0.5) * 2 * width
        // V-channel: lift the edges relative to the centre rib.
        const channel = cup * Math.pow(Math.abs(u - 0.5) * 2, 1.6)
        positions.push(x, spineY + channel * 0.0, spineZ + channel)
        uvs.push(u, v)
        normals.push(0, 0, 1) // recomputed below
      }
    }

    const stride = COLS + 1
    for (let r = 0; r < ROWS; r++) {
      for (let c = 0; c < COLS; c++) {
        const a = r * stride + c
        const b = a + 1
        const d = a + stride
        const e = d + 1
        indices.push(a, d, b, b, d, e)
      }
    }

    const geo = new THREE.BufferGeometry()
    geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3))
    geo.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2))
    geo.setIndex(indices)
    geo.computeVertexNormals()
    return geo
  }, [])
}

/**
 * Two leaves clasping the stem base, mirrored, each rotated outward and
 * given a slight yaw so they don't sit in a single flat plane (more
 * volume, better silhouette from the 3/4 camera).
 */
function Leaves({ color, opacity }: { color: SpringAny; opacity: SpringAny }) {
  const geometry = useLeafGeometry()
  return (
    <group position={[0, 0.18, 0]}>
      {/* Right leaf */}
      <group rotation={[0.12, 0.5, -0.34]}>
        <mesh geometry={geometry}>
          <LeafMaterial color={color} opacity={opacity} />
        </mesh>
      </group>
      {/* Left leaf — mirrored, shorter, leaning the other way */}
      <group rotation={[0.1, -0.6, 0.42]} scale={[-1, 0.86, 1]} position={[0, 0.06, 0]}>
        <mesh geometry={geometry}>
          <LeafMaterial color={color} opacity={opacity} />
        </mesh>
      </group>
    </group>
  )
}

/**
 * Matte leaf material with a faint waxy floor (low clearcoat) and a hint
 * of sheen for the velvety edge highlight real tulip foliage has. Slightly
 * rougher than petals; double-sided so the cupped underside reads.
 */
function LeafMaterial({ color, opacity }: { color: SpringAny; opacity: SpringAny }) {
  return (
    <animated.meshPhysicalMaterial
      color={color}
      side={THREE.DoubleSide}
      transparent
      opacity={opacity}
      roughness={0.82}
      metalness={0}
      clearcoat={0.12}
      clearcoatRoughness={0.7}
      sheen={0.4}
      sheenRoughness={0.8}
      sheenColor={'#c9e7a0'}
    />
  )
}

/* ------------------------------------------------------------------ bloom */

/**
 * One tulip petal as a curved 3D shell. Built parametric in the petal's
 * local frame so the *base* sits at the origin and the petal grows up +Y,
 * giving a natural hinge at the base when we tilt about local X:
 *
 *   - v ∈ [0,1] base→tip; the goblet profile (radius from the petal's own
 *     centreline) is narrow at the base, bulges at the belly (~55% up),
 *     then narrows to a soft point — the egg/goblet silhouette of a tulip.
 *   - each horizontal section is an *arc* (not flat): the petal curls
 *     inward across its width (concave toward the bloom centre) so, with 6
 *     petals, the sections nest into a closed cup.
 *   - the tip leans inward (+ a touch of recurve) the way a closed tulip's
 *     petals kiss at the top.
 *
 * The curl + inward lean are what make six of these read as a *cup* rather
 * than a fan — the single most important fix from the flat-ShapeGeometry
 * version.
 */
function usePetalGeometry(): THREE.BufferGeometry {
  return useMemo(() => buildPetal({ belly: 0.5, curl: 1.0, height: 1.0 }), [])
}

/** Slightly smaller, more tightly curled inner-whorl petal. */
function useInnerPetalGeometry(): THREE.BufferGeometry {
  return useMemo(() => buildPetal({ belly: 0.46, curl: 1.18, height: 0.94 }), [])
}

function buildPetal(opts: { belly: number; curl: number; height: number }): THREE.BufferGeometry {
  const { belly, curl, height } = opts
  const COLS = 10 // across the width (arc resolution)
  const ROWS = 16 // along the length
  const positions: number[] = []
  const uvs: number[] = []
  const indices: number[] = []

  for (let r = 0; r <= ROWS; r++) {
    const v = r / ROWS
    // Goblet half-width profile: 0 at base, max at the belly, ~point at tip.
    // sin gives the rounded belly; the (1 - 0.55 v) pinch closes the tip.
    const halfWidth =
      belly * Math.sin(Math.pow(v, 0.85) * Math.PI) * (1 - 0.5 * Math.pow(v, 2.2)) +
      0.02
    // Spine of the petal: rises, and the upper third leans inward (-Z) with
    // a slight recurve so the tips converge above the cup.
    const spineY = height * 1.15 * Math.sin((v * Math.PI) / 2) // ease toward the top
    const lean = -0.34 * Math.pow(v, 1.6) + 0.06 * Math.pow(v, 3.5) // in then tiny flare
    const spineZ = lean
    // Radius of the inward curl for this section — petal hugs tighter near
    // the tip. Larger radius = flatter section.
    const arcAmt = curl * (0.55 + 0.5 * v) // curl strength up the petal
    for (let c = 0; c <= COLS; c++) {
      const u = c / COLS - 0.5 // -0.5..0.5 across
      const ang = u * Math.PI * 0.92 * arcAmt * 0.5
      // Wrap the flat width into an arc that bows toward -Z (bloom centre).
      const x = Math.sin(ang) * halfWidth * 1.18
      const zCurl = (Math.cos(ang) - 1) * halfWidth * 0.85 // negative → concave toward centre
      positions.push(x, spineY, spineZ + zCurl)
      uvs.push(c / COLS, v)
    }
  }

  const stride = COLS + 1
  for (let r = 0; r < ROWS; r++) {
    for (let c = 0; c < COLS; c++) {
      const a = r * stride + c
      const b = a + 1
      const d = a + stride
      const e = d + 1
      indices.push(a, d, b, b, d, e)
    }
  }

  const geo = new THREE.BufferGeometry()
  geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3))
  geo.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2))
  geo.setIndex(indices)
  geo.computeVertexNormals()
  return geo
}

/**
 * The bloom: two offset whorls of three petals (outer + inner, rotated 60°
 * apart) for the classic six-tepal tulip, closing into a goblet. The droop
 * value rotates every petal open/down about its base hinge:
 *   - droop 0  → petals upright, tips kissing → tight closed cup
 *   - droop 1  → petals splay down and out → wilted, opened-then-collapsed
 * Inner petals droop a touch less so the bloom keeps a core even when sad.
 *
 * Petal radius (offset from the bloom axis) is small so the bases meet near
 * the centre and the cup closes; the curl in the geometry does the rest.
 */
function Bloom({
  droop, color, opacity,
}: {
  droop: SpringAny
  color: SpringAny
  opacity: SpringAny
}) {
  const outerGeo = usePetalGeometry()
  const innerGeo = useInnerPetalGeometry()
  const outerRef = useRef<THREE.Group>(null)
  const innerRef = useRef<THREE.Group>(null)

  // Resolve the droop→tilt mapping per frame without allocating. Each petal
  // group's child mesh is tilted about local X from its base hinge.
  useFrame(() => {
    const d = droop.get() as number
    applyDroop(outerRef.current, d, /*closed*/ -0.16, /*open*/ 1.15)
    applyDroop(innerRef.current, d, -0.1, 0.82)
  })

  return (
    <group position={[0, 2.5, 0]}>
      {/* faint inner shadow / throat so the cup centre doesn't look hollow-bright */}
      <mesh position={[0, 0.12, 0]}>
        <sphereGeometry args={[0.16, 16, 12]} />
        <animated.meshStandardMaterial
          color={color}
          roughness={0.9}
          transparent
          opacity={opacity}
        />
      </mesh>

      <group ref={outerRef}>
        {[0, 1, 2].map((i) => (
          <group key={i} rotation={[0, (i / 3) * Math.PI * 2, 0]}>
            <mesh geometry={outerGeo} position={[0, 0, 0.05]}>
              <PetalMaterial color={color} opacity={opacity} />
            </mesh>
          </group>
        ))}
      </group>

      <group ref={innerRef} rotation={[0, Math.PI / 3, 0]}>
        {[0, 1, 2].map((i) => (
          <group key={i} rotation={[0, (i / 3) * Math.PI * 2, 0]}>
            <mesh geometry={innerGeo} position={[0, 0, 0.035]}>
              <PetalMaterial color={color} opacity={opacity} inner />
            </mesh>
          </group>
        ))}
      </group>
    </group>
  )
}

/** Tilt every petal in a whorl about its base hinge by lerping closed→open. */
function applyDroop(
  group: THREE.Group | null,
  d: number,
  closedTilt: number,
  openTilt: number,
) {
  if (!group) return
  const tilt = closedTilt + (openTilt - closedTilt) * d
  for (let i = 0; i < group.children.length; i++) {
    // child is a yaw-group; its first child is the petal mesh
    const yawGroup = group.children[i] as THREE.Group
    const petal = yawGroup.children[0] as THREE.Mesh
    petal.rotation.x = tilt
  }
}

/**
 * Tulip petals are faintly waxy and slightly translucent — light bleeds
 * through the thin tepal at the edges. We get there with MeshPhysicalMaterial:
 *   - low roughness + clearcoat for the waxy surface highlight
 *   - sheen (warm) for the soft velvet falloff toward grazing angles
 *   - a little transmission + thickness for the back-lit translucency,
 *     with iridescence off (would read as soap-bubble, wrong for a petal)
 * Inner-whorl petals are a hair glossier and thinner so the throat glows.
 */
function PetalMaterial({
  color, opacity, inner = false,
}: {
  color: SpringAny
  opacity: SpringAny
  inner?: boolean
}) {
  return (
    <animated.meshPhysicalMaterial
      color={color}
      side={THREE.DoubleSide}
      transparent
      opacity={opacity}
      roughness={inner ? 0.26 : 0.34}
      metalness={0}
      clearcoat={0.5}
      clearcoatRoughness={0.4}
      sheen={0.7}
      sheenRoughness={0.5}
      sheenColor={'#ffd9c2'}
      transmission={inner ? 0.28 : 0.18}
      thickness={inner ? 0.35 : 0.55}
      ior={1.36}
      attenuationColor={'#ff6b5e'}
      attenuationDistance={1.4}
    />
  )
}
