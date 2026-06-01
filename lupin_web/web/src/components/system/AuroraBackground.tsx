/**
 * AuroraBackground — the living lightfield the whole console floats on.
 *
 * A single full-screen GLSL plane (domain-warped fbm) that drifts slowly and
 * blends botanical-ink → lichen-chartreuse → signal-cyan. Its tone eases
 * between three system moods:
 *   idle   calm, dim
 *   active brighter, faster drift  (link up / mission running / driving)
 *   alert  red shift               (e-stop engaged / fault)
 *
 * Guardrails:
 *   • DPR clamped (1–1.4, 1 on touch) — the shader is cheap but we never
 *     supersample it.
 *   • Frameloop pauses when the tab is hidden (zero GPU when backgrounded).
 *   • prefers-reduced-motion → no WebGL at all; a static CSS gradient instead.
 *   • Opaque output + grid/vignette overlay live here so stacking is explicit.
 */
import { PerformanceMonitor } from '@react-three/drei'
import { Canvas, useFrame, useThree } from '@react-three/fiber'
import { useEffect, useMemo, useRef, useState } from 'react'
import * as THREE from 'three'

import { prefersReducedMotion } from '@/lib/motion'

export type AuroraTone = 'idle' | 'active' | 'alert'

interface ToneSpec {
  base: THREE.Color
  warm: THREE.Color
  cool: THREE.Color
  intensity: number
  speed: number
}

// Colours are deliberately muted; the shader's blend factors keep the field
// dark with the occasional bright ribbon rather than a neon wash.
const TONES: Record<AuroraTone, ToneSpec> = {
  idle: {
    base: new THREE.Color('#080b08'),
    warm: new THREE.Color('#3f5e1c'),
    cool: new THREE.Color('#0f3b44'),
    intensity: 0.62,
    speed: 0.6,
  },
  active: {
    base: new THREE.Color('#080b08'),
    warm: new THREE.Color('#5f8e22'),
    cool: new THREE.Color('#13586a'),
    intensity: 1.0,
    speed: 1.0,
  },
  alert: {
    base: new THREE.Color('#0d0504'),
    warm: new THREE.Color('#7c1d12'),
    cool: new THREE.Color('#5a160f'),
    intensity: 1.1,
    speed: 1.35,
  },
}

const vertexShader = /* glsl */ `
  varying vec2 vUv;
  void main() {
    vUv = uv;
    gl_Position = vec4(position.xy, 0.0, 1.0);
  }
`

const fragmentShader = /* glsl */ `
  precision highp float;
  varying vec2 vUv;
  uniform float uTime;
  uniform vec2  uRes;
  uniform vec3  uBase;
  uniform vec3  uWarm;
  uniform vec3  uCool;
  uniform float uIntensity;
  uniform float uSpeed;

  // -- value noise + fbm ---------------------------------------------------
  float hash(vec2 p) {
    p = fract(p * vec2(123.34, 456.21));
    p += dot(p, p + 45.32);
    return fract(p.x * p.y);
  }
  float noise(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    float a = hash(i);
    float b = hash(i + vec2(1.0, 0.0));
    float c = hash(i + vec2(0.0, 1.0));
    float d = hash(i + vec2(1.0, 1.0));
    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
  }
  float fbm(vec2 p) {
    float v = 0.0;
    float a = 0.5;
    for (int i = 0; i < 5; i++) {
      v += a * noise(p);
      p *= 2.02;
      a *= 0.5;
    }
    return v;
  }

  void main() {
    vec2 uv = vUv;
    vec2 p = (uv - 0.5) * vec2(uRes.x / uRes.y, 1.0) * 2.6;
    float t = uTime * 0.05 * uSpeed;

    // Domain warp for the flowing ribbon look.
    float n1 = fbm(p * 1.1 + vec2(t, t * 0.6));
    float n2 = fbm(p * 1.9 - vec2(t * 0.7, t * 0.45) + n1 * 1.5);
    float bands = fbm(p + vec2(n1, n2) * 1.2);

    vec3 col = uBase;
    float warmGlow = smoothstep(0.30, 0.95, bands) * 0.75 * uIntensity;
    col = mix(col, uWarm, warmGlow);
    float coolGlow = smoothstep(0.45, 1.0, n2) * 0.5 * uIntensity;
    col = mix(col, uCool, coolGlow);

    // Corner blooms — chartreuse top-left, cyan bottom-right (match the brand).
    col += uWarm * 0.12 * uIntensity * smoothstep(0.75, 0.0, distance(uv, vec2(0.10, 0.92)));
    col += uCool * 0.09 * uIntensity * smoothstep(0.8, 0.0, distance(uv, vec2(0.95, 0.08)));

    // Vignette toward the floor.
    float vig = smoothstep(1.25, 0.25, distance(uv, vec2(0.5, 0.42)));
    col *= mix(0.62, 1.0, vig);

    gl_FragColor = vec4(col, 1.0);
  }
`

function AuroraPlane({ tone }: { tone: AuroraTone }) {
  const matRef = useRef<THREE.ShaderMaterial>(null)
  const { size } = useThree()

  // Live values eased toward the active tone each frame.
  const cur = useRef<ToneSpec>({
    base: TONES.idle.base.clone(),
    warm: TONES.idle.warm.clone(),
    cool: TONES.idle.cool.clone(),
    intensity: TONES.idle.intensity,
    speed: TONES.idle.speed,
  })

  const uniforms = useMemo(
    () => ({
      uTime: { value: 0 },
      uRes: { value: new THREE.Vector2(size.width, size.height) },
      uBase: { value: cur.current.base },
      uWarm: { value: cur.current.warm },
      uCool: { value: cur.current.cool },
      uIntensity: { value: cur.current.intensity },
      uSpeed: { value: cur.current.speed },
    }),
    // uniforms object is created once; size/colours are updated imperatively.
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [],
  )

  useEffect(() => {
    uniforms.uRes.value.set(size.width, size.height)
  }, [size, uniforms])

  useFrame((_, delta) => {
    const m = matRef.current
    if (!m) return
    const target = TONES[tone]
    const k = 1 - Math.exp(-delta / 0.5) // ~0.5s ease
    cur.current.base.lerp(target.base, k)
    cur.current.warm.lerp(target.warm, k)
    cur.current.cool.lerp(target.cool, k)
    cur.current.intensity += (target.intensity - cur.current.intensity) * k
    cur.current.speed += (target.speed - cur.current.speed) * k
    m.uniforms.uTime.value += delta
    m.uniforms.uIntensity.value = cur.current.intensity
    m.uniforms.uSpeed.value = cur.current.speed
  })

  return (
    <mesh frustumCulled={false}>
      <planeGeometry args={[2, 2]} />
      <shaderMaterial
        ref={matRef}
        uniforms={uniforms}
        vertexShader={vertexShader}
        fragmentShader={fragmentShader}
        depthTest={false}
        depthWrite={false}
      />
    </mesh>
  )
}

/** Cheap CSS field used for reduced-motion / no-WebGL. */
function CssAurora({ tone }: { tone: AuroraTone }) {
  const warm = tone === 'alert' ? 'hsla(8,80%,40%,0.22)' : 'hsla(78,80%,42%,0.16)'
  const cool = tone === 'alert' ? 'hsla(8,70%,35%,0.16)' : 'hsla(192,80%,45%,0.12)'
  return (
    <div
      className="absolute inset-0"
      style={{
        backgroundImage: `radial-gradient(1200px 800px at 8% 92%, ${warm}, transparent 60%), radial-gradient(1000px 700px at 96% 6%, ${cool}, transparent 60%)`,
        transition: 'background-image 600ms var(--ease-out-expo)',
      }}
    />
  )
}

export function AuroraBackground({ tone = 'idle' }: { tone?: AuroraTone }) {
  const [reduced] = useState(prefersReducedMotion)
  const [hidden, setHidden] = useState(false)
  const [failed, setFailed] = useState(false)
  const isTouch = useMemo(
    () => typeof window !== 'undefined' && window.matchMedia?.('(pointer: coarse)').matches,
    [],
  )
  // Degradation ladder — PerformanceMonitor drops us to dpr 1 on sustained
  // frame decline (modest laptop GPU is the constraint).
  const [maxDpr, setMaxDpr] = useState(isTouch ? 1 : 1.4)

  useEffect(() => {
    const onVis = () => setHidden(document.hidden)
    document.addEventListener('visibilitychange', onVis)
    return () => document.removeEventListener('visibilitychange', onVis)
  }, [])

  const useWebGL = !reduced && !failed

  return (
    <div
      aria-hidden
      className="pointer-events-none fixed inset-0 z-0 overflow-hidden noise"
    >
      {useWebGL ? (
        <Canvas
          className="!absolute inset-0"
          dpr={[1, maxDpr]}
          frameloop={hidden ? 'never' : 'always'}
          gl={{ antialias: false, alpha: false, powerPreference: 'low-power' }}
          onCreated={({ gl }) => gl.setClearColor('#080b08', 1)}
          // If WebGL context can't be created, drop to the CSS field.
          fallback={null}
        >
          <PerformanceMonitor
            onDecline={() => setMaxDpr(1)}
            flipflops={3}
            onFallback={() => setMaxDpr(1)}
          />
          <ErrorToCss onError={() => setFailed(true)}>
            <AuroraPlane tone={tone} />
          </ErrorToCss>
        </Canvas>
      ) : (
        <CssAurora tone={tone} />
      )}

      {/* Deterministic contrast floor: a fixed dark scrim so text legibility is
          computed against a KNOWN luminance, never the live shader's brightest
          pixel. Everything above this is guaranteed-dark base. */}
      <div className="absolute inset-0" style={{ backgroundColor: 'hsl(var(--ink-1) / 0.4)' }} />

      {/* Drifting hex grid + floor vignette over the colour field. */}
      <div
        className="absolute inset-0 bg-grid opacity-[0.5]"
        style={{
          maskImage: 'radial-gradient(ellipse 85% 65% at 50% 32%, #000 25%, transparent 82%)',
          WebkitMaskImage: 'radial-gradient(ellipse 85% 65% at 50% 32%, #000 25%, transparent 82%)',
        }}
      />
      <div
        className="absolute inset-0"
        style={{
          background:
            'radial-gradient(120% 90% at 50% -10%, transparent 55%, rgba(0,0,0,0.45) 100%)',
        }}
      />
    </div>
  )
}

/** Tiny boundary: if the GL subtree throws on init, switch to the CSS field. */
import { Component, type ReactNode } from 'react'
class ErrorToCss extends Component<{ children: ReactNode; onError: () => void }, { dead: boolean }> {
  state = { dead: false }
  static getDerivedStateFromError() {
    return { dead: true }
  }
  componentDidCatch() {
    this.props.onError()
  }
  render() {
    return this.state.dead ? null : this.props.children
  }
}
