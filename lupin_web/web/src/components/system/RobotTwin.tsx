/**
 * RobotTwin — a live, URDF-faithful 3D twin of the Mirte Master.
 *
 * The articulated arm uses the REAL meshes and the REAL kinematic chain from
 * `mirte_master_description` (arm.xacro): each link's STL is nested under its
 * actual joint origin + rotated about its actual axis, including the full mimic
 * gripper linkage. The chassis + mecanum wheels are primitives at the exact
 * URDF collision dimensions / wheel positions (the real frame_link.STL is 21 MB
 * — far too heavy to ship to a browser). Driven by the SAME `joint_states` /
 * `odom` the 2D widgets read.
 *
 * Guardrails: small canvas, dpr ≤1.5, AA off, lights-only (offline-safe), pause
 * on tab-hidden, reduced-motion drops auto-rotate/parallax, "STALE" on silence.
 */
import { ContactShadows } from '@react-three/drei'
import { Canvas, useFrame, useLoader } from '@react-three/fiber'
import { Suspense, useEffect, useRef, useState } from 'react'
import * as THREE from 'three'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'

import { damp, prefersReducedMotion } from '@/lib/motion'
import { useTopic } from '@/lib/ros'
import { useSettings } from '@/lib/settings'
import { cn } from '@/lib/utils'
import { ROS_TYPE, type JointState, type Odometry } from '@/types/ros'

const MESH = '/urdf/meshes'
// Loaded as one array → indices below.
const URLS = [
  `${MESH}/arm_Rot.STL`, // 0 shoulder_pan
  `${MESH}/arm_Shoulder.STL`, // 1 shoulder_lift
  `${MESH}/arm_Elbow.STL`, // 2 elbow
  `${MESH}/arm_Wrist.STL`, // 3 wrist
  `${MESH}/Gripper.STL`, // 4 gripper jaw
  `${MESH}/_Gripper_r.STL`, // 5 mimic -1
  `${MESH}/_gripper_link_r.STL`, // 6 mimic 1.4
  `${MESH}/gripper_finger_r.STL`, // 7 fixed on link_r
  `${MESH}/_gripper_link_l.STL`, // 8 mimic -1.4
  `${MESH}/gripper_finger_l.STL`, // 9 fixed on link_l
]

const X = new THREE.Vector3(1, 0, 0)
const Y = new THREE.Vector3(0, 1, 0)
const Z = new THREE.Vector3(0, 0, 1)

/** URDF rpy (fixed-axis roll,pitch,yaw) → quaternion array [x,y,z,w]. R = Rz·Ry·Rx. */
function rpy(r: number, p: number, y: number): [number, number, number, number] {
  const q = new THREE.Quaternion()
    .setFromAxisAngle(Z, y)
    .multiply(new THREE.Quaternion().setFromAxisAngle(Y, p))
    .multiply(new THREE.Quaternion().setFromAxisAngle(X, r))
  return [q.x, q.y, q.z, q.w]
}

type V3 = [number, number, number]
// Joint origins straight from arm.xacro (parent-relative xyz + rpy) + spin axis.
const J = {
  pan: { pos: [0, -0.079274, 0.06] as V3, quat: rpy(1.5708, 0, 3.1416), axis: Y.clone() },
  lift: { pos: [0, 0.0281, -0.00625] as V3, quat: rpy(3.1416, 0, 1.5708), axis: Y.clone() },
  elbow: { pos: [0.1378, 0, 0] as V3, quat: rpy(-3.1416, 0, 1.5708), axis: X.clone() },
  wrist: { pos: [-0.00014, 0.14265, 0] as V3, quat: rpy(3.1416, 0, 1.5708), axis: Y.clone() },
  grip: { pos: [0.0435, -0.012, 0.0045] as V3, quat: rpy(-1.5708, 0, 0), axis: new THREE.Vector3(0, -1, 0) },
  gripR: { pos: [0.0435, 0.012, 0.0045] as V3, quat: rpy(-1.5708, 0, -3.1416), axis: new THREE.Vector3(0, -1, 0) },
  linkR: { pos: [0.0605, 0.0205, -0.0055] as V3, quat: rpy(1.5708, 0, -1.3641), axis: new THREE.Vector3(0, -1, 0) },
  linkL: { pos: [0.0605, -0.0205, -0.0055] as V3, quat: rpy(1.5708, 0, -1.7775), axis: new THREE.Vector3(0, -1, 0) },
  fingerR: { pos: [0, 0.01, -0.037] as V3, quat: rpy(0, 0.20667, 3.1416) },
  fingerL: { pos: [0, 0, -0.037] as V3, quat: rpy(0, 0.20667, 0) },
}

// Mecanum wheel joint origins (frame_link-relative), wheel_radius 0.05, length 0.055.
const WHEELS: { pos: V3; quat: [number, number, number, number]; lo: number }[] = [
  { pos: [0.153, -0.085, -0.0455], quat: rpy(1.5708, 0, -1.5708), lo: 0.025 },
  { pos: [0.153, 0.085, -0.0455], quat: rpy(1.5708, 0, -1.5708), lo: 0.025 },
  { pos: [-0.098, -0.085, -0.0455], quat: rpy(1.5708, 0, -1.5708), lo: -0.025 },
  { pos: [-0.098, 0.085, -0.0455], quat: rpy(1.5708, 0, -1.5708), lo: -0.025 },
]

function jointVal(js: JointState | null, name: string): number {
  if (!js?.name) return 0
  const i = js.name.indexOf(name)
  return i >= 0 ? js.position[i] ?? 0 : 0
}
function yawOf(od: Odometry | null): number {
  if (!od?.pose?.pose) return 0
  const q = od.pose.pose.orientation
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
}

const ARM_COLOR = '#d4e7a6'
const ACCENT = '#aef359'

function ArmMaterial({ accent = false }: { accent?: boolean }) {
  return (
    <meshStandardMaterial
      color={accent ? '#bfe7f5' : ARM_COLOR}
      metalness={0.4}
      roughness={0.42}
      emissive={accent ? '#1d6f86' : '#3a541a'}
      emissiveIntensity={0.2}
    />
  )
}

function Rover({
  jointRef,
  poseRef,
  reduced,
}: {
  jointRef: React.MutableRefObject<JointState | null>
  poseRef: React.MutableRefObject<Odometry | null>
  reduced: boolean
}) {
  const geos = useLoader(STLLoader, URLS)
  const [g0, g1, g2, g3, g4, g5, g6, g7, g8, g9] = geos

  const spin = useRef<THREE.Group>(null)
  const pan = useRef<THREE.Group>(null)
  const lift = useRef<THREE.Group>(null)
  const elbow = useRef<THREE.Group>(null)
  const wrist = useRef<THREE.Group>(null)
  const grip = useRef<THREE.Group>(null)
  const gripR = useRef<THREE.Group>(null)
  const linkR = useRef<THREE.Group>(null)
  const linkL = useRef<THREE.Group>(null)
  const cur = useRef({ pan: 0, lift: 0, elbow: 0, wrist: 0, grip: 0, yaw: 0 })

  useFrame((state, dt) => {
    const js = jointRef.current
    const c = cur.current
    c.pan = damp(c.pan, jointVal(js, 'shoulder_pan_joint'), 8, dt)
    c.lift = damp(c.lift, jointVal(js, 'shoulder_lift_joint'), 8, dt)
    c.elbow = damp(c.elbow, jointVal(js, 'elbow_joint'), 8, dt)
    c.wrist = damp(c.wrist, jointVal(js, 'wrist_joint'), 8, dt)
    c.grip = damp(c.grip, jointVal(js, 'gripper_joint'), 10, dt)
    c.yaw = damp(c.yaw, yawOf(poseRef.current), 6, dt)

    pan.current?.setRotationFromAxisAngle(J.pan.axis, c.pan)
    lift.current?.setRotationFromAxisAngle(J.lift.axis, c.lift)
    elbow.current?.setRotationFromAxisAngle(J.elbow.axis, c.elbow)
    wrist.current?.setRotationFromAxisAngle(J.wrist.axis, c.wrist)
    grip.current?.setRotationFromAxisAngle(J.grip.axis, c.grip)
    gripR.current?.setRotationFromAxisAngle(J.gripR.axis, -c.grip)
    linkR.current?.setRotationFromAxisAngle(J.linkR.axis, 1.4 * c.grip)
    linkL.current?.setRotationFromAxisAngle(J.linkL.axis, -1.4 * c.grip)

    if (spin.current) {
      spin.current.rotation.y = c.yaw + (reduced ? 0 : state.clock.elapsedTime * 0.16)
    }
    if (!reduced) {
      state.camera.position.x = damp(state.camera.position.x, 0.34 + state.pointer.x * 0.22, 4, dt)
      state.camera.position.y = damp(state.camera.position.y, 0.34 - state.pointer.y * 0.12, 4, dt)
      state.camera.lookAt(0, 0.3, 0)
    }
  })

  return (
    <group ref={spin}>
      {/* frame_link frame is Z-up; rotate to three Y-up and rest wheels on y=0. */}
      <group rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.0955, 0]}>
        {/* ── chassis (URDF collision dims) ── */}
        <mesh position={[0, 0, -0.03]} castShadow receiveShadow>
          <boxGeometry args={[0.19, 0.28, 0.06]} />
          <meshStandardMaterial color="#10160f" metalness={0.5} roughness={0.5} emissive="#0b1207" emissiveIntensity={0.25} />
        </mesh>
        {/* raised electronics deck */}
        <mesh position={[0, 0.05, 0.045]} castShadow>
          <boxGeometry args={[0.18, 0.16, 0.05]} />
          <meshStandardMaterial color="#161d12" metalness={0.45} roughness={0.5} />
        </mesh>
        {/* chartreuse deck plate */}
        <mesh position={[0, 0.05, 0.072]}>
          <boxGeometry args={[0.15, 0.13, 0.004]} />
          <meshStandardMaterial color={ACCENT} emissive={ACCENT} emissiveIntensity={0.5} toneMapped={false} />
        </mesh>
        {/* depth camera (orbbec) on the front face */}
        <group position={[0.01, -0.1467, 0.0427]} quaternion={rpy(0, 0, -1.5708)}>
          <mesh>
            <boxGeometry args={[0.09, 0.025, 0.025]} />
            <meshStandardMaterial color="#0a0a0a" metalness={0.3} roughness={0.6} />
          </mesh>
          <mesh position={[0.028, 0, 0]}>
            <cylinderGeometry args={[0.006, 0.006, 0.026, 16]} />
            <meshStandardMaterial color="#1b6f86" emissive="#38bdf8" emissiveIntensity={0.6} toneMapped={false} />
          </mesh>
        </group>

        {/* ── mecanum wheels ── */}
        {WHEELS.map((w, i) => (
          <group key={i} position={w.pos} quaternion={w.quat}>
            <mesh position={[0, 0, w.lo]} rotation={[Math.PI / 2, 0, 0]} castShadow>
              <cylinderGeometry args={[0.05, 0.05, 0.055, 22]} />
              <meshStandardMaterial color="#0c0f0b" metalness={0.6} roughness={0.4} emissive="#16240e" emissiveIntensity={0.3} />
            </mesh>
          </group>
        ))}

        {/* ── arm: real meshes on the real chain ── */}
        <group position={J.pan.pos} quaternion={J.pan.quat}>
          <group ref={pan}>
            <mesh geometry={g0} castShadow>
              <ArmMaterial />
            </mesh>
            <group position={J.lift.pos} quaternion={J.lift.quat}>
              <group ref={lift}>
                <mesh geometry={g1} castShadow>
                  <ArmMaterial />
                </mesh>
                <group position={J.elbow.pos} quaternion={J.elbow.quat}>
                  <group ref={elbow}>
                    <mesh geometry={g2} castShadow>
                      <ArmMaterial />
                    </mesh>
                    <group position={J.wrist.pos} quaternion={J.wrist.quat}>
                      <group ref={wrist}>
                        <mesh geometry={g3} castShadow>
                          <ArmMaterial />
                        </mesh>
                        {/* gripper jaw */}
                        <group position={J.grip.pos} quaternion={J.grip.quat}>
                          <group ref={grip}>
                            <mesh geometry={g4} castShadow>
                              <ArmMaterial accent />
                            </mesh>
                          </group>
                        </group>
                        <group position={J.gripR.pos} quaternion={J.gripR.quat}>
                          <group ref={gripR}>
                            <mesh geometry={g5} castShadow>
                              <ArmMaterial accent />
                            </mesh>
                          </group>
                        </group>
                        {/* right finger linkage */}
                        <group position={J.linkR.pos} quaternion={J.linkR.quat}>
                          <group ref={linkR}>
                            <mesh geometry={g6} castShadow>
                              <ArmMaterial />
                            </mesh>
                            <group position={J.fingerR.pos} quaternion={J.fingerR.quat}>
                              <mesh geometry={g7} castShadow>
                                <ArmMaterial />
                              </mesh>
                            </group>
                          </group>
                        </group>
                        {/* left finger linkage */}
                        <group position={J.linkL.pos} quaternion={J.linkL.quat}>
                          <group ref={linkL}>
                            <mesh geometry={g8} castShadow>
                              <ArmMaterial />
                            </mesh>
                            <group position={J.fingerL.pos} quaternion={J.fingerL.quat}>
                              <mesh geometry={g9} castShadow>
                                <ArmMaterial />
                              </mesh>
                            </group>
                          </group>
                        </group>
                      </group>
                    </group>
                  </group>
                </group>
              </group>
            </group>
          </group>
        </group>
      </group>
    </group>
  )
}

function Scene({
  jointRef,
  poseRef,
  reduced,
}: {
  jointRef: React.MutableRefObject<JointState | null>
  poseRef: React.MutableRefObject<Odometry | null>
  reduced: boolean
}) {
  return (
    <>
      <ambientLight intensity={0.55} />
      <directionalLight position={[3, 5, 2]} intensity={1.5} castShadow shadow-mapSize={[1024, 1024]} />
      <pointLight position={[-2, 1.6, -1]} intensity={16} color="#38bdf8" />
      <pointLight position={[2, 1.3, 2]} intensity={12} color="#aef359" />
      <Suspense fallback={null}>
        <Rover jointRef={jointRef} poseRef={poseRef} reduced={reduced} />
      </Suspense>
      <ContactShadows position={[0, 0, 0]} opacity={0.5} scale={2.4} blur={2.6} far={1.5} color="#000000" />
      <gridHelper args={[4, 24, '#1c2a16', '#11170e']} position={[0, 0.001, 0]} />
    </>
  )
}

export function RobotTwin({ className }: { className?: string }) {
  const [{ jointStatesTopic, odomTopic }] = useSettings()
  const [reduced] = useState(prefersReducedMotion)
  const [hidden, setHidden] = useState(false)
  const [stale, setStale] = useState(false)
  const lastMsg = useRef(performance.now())
  const bump = () => {
    lastMsg.current = performance.now()
  }

  const jointRef = useTopic<JointState>(jointStatesTopic, ROS_TYPE.JointState, { onMessage: bump })
  const poseRef = useTopic<Odometry>(odomTopic, ROS_TYPE.Odometry, { onMessage: bump })

  useEffect(() => {
    const onVis = () => setHidden(document.hidden)
    document.addEventListener('visibilitychange', onVis)
    const id = setInterval(() => setStale(performance.now() - lastMsg.current > 3000), 1000)
    return () => {
      document.removeEventListener('visibilitychange', onVis)
      clearInterval(id)
    }
  }, [])

  return (
    <div className={cn('relative overflow-hidden', className)}>
      <Canvas
        dpr={[1, 1.5]}
        frameloop={hidden ? 'never' : 'always'}
        shadows
        camera={{ position: [0.34, 0.34, 0.46], fov: 40 }}
        gl={{ antialias: false, alpha: true, powerPreference: 'low-power' }}
      >
        <Scene jointRef={jointRef} poseRef={poseRef} reduced={reduced} />
      </Canvas>

      <div className="reticle pointer-events-none absolute inset-0">
        <span className="reticle-bl" aria-hidden />
        <span className="reticle-br" aria-hidden />
      </div>
      <div className="pointer-events-none absolute left-3 top-3 flex items-center gap-2">
        <span className="tag tag-strong">digital twin</span>
        <span className="tag tag-accent">MDL-MIRTE-01</span>
      </div>
      <div className="pointer-events-none absolute bottom-3 right-3">
        <span className={cn('tag', stale ? 'text-warning' : 'tag-accent')}>{stale ? '○ stale' : '● live'}</span>
      </div>
    </div>
  )
}
