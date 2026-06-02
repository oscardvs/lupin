import { AlertOctagon, Lightbulb, Loader2, Sparkles } from 'lucide-react'
import { useState } from 'react'

import { Button } from '@/components/ui/button'
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from '@/components/ui/card'
import { useService } from '@/lib/ros'
import { cn } from '@/lib/utils'
import {
  LED_SERVICE,
  LUPIN_SRV,
  MIRTE_SRV,
  type SetNeopixelRequest,
  type SetNeopixelResponse,
} from '@/types/ros'

interface LightControlProps {
  className?: string
}

interface Rgb {
  r: number
  g: number
  b: number
}

/**
 * Manual override for the status light strip. The robot normally drives the
 * strip from the mission state machine (lupin_hmi/light_strip_bridge); this
 * card lets the operator pin a colour or hand control back.
 *
 * The preset swatches mirror the bridge's auto palette so a manually-set
 * colour reads the same as the state it represents. Safety states (e-stop /
 * FAULT) still force red on the robot regardless of a manual hold — the bridge
 * enforces that, this UI just reflects the operator's intent.
 */
export function LightControl({ className }: LightControlProps) {
  const setManual = useService<SetNeopixelRequest, SetNeopixelResponse>(
    LED_SERVICE.setManual,
    MIRTE_SRV.SetNeopixel,
  )
  const setAuto = useService<Record<string, never>, { success: boolean; message: string }>(
    LED_SERVICE.setAuto,
    LUPIN_SRV.Trigger,
  )

  const [mode, setMode] = useState<'auto' | 'manual'>('auto')
  const [held, setHeld] = useState<Rgb | null>(null)
  const [custom, setCustom] = useState('#22d3ee')
  const [busy, setBusy] = useState<string | null>(null)
  const [feedback, setFeedback] = useState<{ tone: 'ok' | 'err'; msg: string } | null>(null)

  const applyManual = async (rgb: Rgb, label: string) => {
    if (busy) return
    setBusy(label)
    setFeedback(null)
    try {
      const res = await setManual({ color: rgb })
      const ok = res?.status ?? false
      if (ok) {
        setMode('manual')
        setHeld(rgb)
        setFeedback({ tone: 'ok', msg: `light → ${label.toLowerCase()}` })
      } else {
        setFeedback({ tone: 'err', msg: 'LED service rejected the colour' })
      }
    } catch (e) {
      setFeedback({ tone: 'err', msg: e instanceof Error ? e.message : String(e) })
    } finally {
      setBusy(null)
    }
  }

  const applyAuto = async () => {
    if (busy) return
    setBusy('auto')
    setFeedback(null)
    try {
      const res = await setAuto({})
      const ok = res?.success ?? false
      if (ok) {
        setMode('auto')
        setHeld(null)
        setFeedback({ tone: 'ok', msg: res?.message || 'following mission state' })
      } else {
        setFeedback({ tone: 'err', msg: res?.message || 'auto rejected' })
      }
    } catch (e) {
      setFeedback({ tone: 'err', msg: e instanceof Error ? e.message : String(e) })
    } finally {
      setBusy(null)
    }
  }

  return (
    <Card className={cn('flex flex-col', className)}>
      <CardHeader>
        <CardTitle>
          <Lightbulb className="h-4 w-4 text-primary" />
          Status Light
          <span className="tag tag-accent ml-auto">PNL-LED-01</span>
        </CardTitle>
        <CardDescription className="flex items-center gap-2 font-mono text-xs">
          {mode === 'auto' ? (
            <>
              <Sparkles className="h-3.5 w-3.5" />
              auto · following mission state
            </>
          ) : (
            <>
              <span
                aria-hidden
                className="h-3.5 w-3.5 rounded-full border border-hairline"
                style={{ background: held ? rgbToCss(held) : 'transparent' }}
              />
              manual · {held ? rgbToHex(held) : 'held'}
            </>
          )}
        </CardDescription>
      </CardHeader>

      <CardContent className="flex flex-col gap-3">
        <div className="grid grid-cols-6 gap-2">
          {PRESETS.map((p) => {
            const isHeld =
              mode === 'manual' &&
              held?.r === p.rgb.r &&
              held?.g === p.rgb.g &&
              held?.b === p.rgb.b
            return (
              <button
                key={p.label}
                type="button"
                title={`${p.label} (${rgbToHex(p.rgb)})`}
                aria-label={`Set light ${p.label}`}
                disabled={busy !== null}
                onClick={() => applyManual(p.rgb, p.label)}
                className={cn(
                  'group relative aspect-square rounded-sm border transition-all',
                  'border-hairline hover:scale-105 hover:border-primary/60',
                  'disabled:cursor-not-allowed disabled:opacity-50',
                  isHeld && 'ring-2 ring-primary ring-offset-1 ring-offset-background',
                )}
                style={{ background: rgbToCss(p.rgb) }}
              >
                {p.label === 'Off' && (
                  <span className="absolute inset-0 flex items-center justify-center text-[9px] font-semibold text-muted-foreground">
                    OFF
                  </span>
                )}
                {busy === p.label && (
                  <span className="absolute inset-0 flex items-center justify-center">
                    <Loader2 className="h-3.5 w-3.5 animate-spin text-white mix-blend-difference" />
                  </span>
                )}
              </button>
            )
          })}
        </div>

        <div className="flex items-center gap-2">
          <label className="flex items-center gap-2 text-[11px] text-muted-foreground">
            <span className="tag">custom</span>
            <input
              type="color"
              value={custom}
              onChange={(e) => setCustom(e.target.value)}
              aria-label="custom light colour"
              className="h-8 w-12 cursor-pointer rounded-sm border border-hairline bg-transparent p-0.5"
            />
            <span className="font-mono">{custom}</span>
          </label>
          <Button
            size="sm"
            variant="secondary"
            disabled={busy !== null}
            onClick={() => applyManual(hexToRgb(custom), 'Custom')}
            className="h-8"
          >
            {busy === 'Custom' ? <Loader2 className="h-4 w-4 animate-spin" /> : null}
            Set
          </Button>

          <Button
            size="sm"
            variant={mode === 'manual' ? 'default' : 'outline'}
            disabled={busy !== null || mode === 'auto'}
            onClick={applyAuto}
            className="ml-auto h-8"
            title="Hand the strip back to the mission state machine"
          >
            {busy === 'auto'
              ? <Loader2 className="h-4 w-4 animate-spin" />
              : <Sparkles className="h-4 w-4" />}
            Auto
          </Button>
        </div>

        {feedback && (
          <div
            aria-live="polite"
            className={cn(
              'flex items-start gap-2 rounded-sm border px-2.5 py-1.5 text-[11px]',
              feedback.tone === 'err'
                ? 'border-destructive/50 bg-destructive/10 text-destructive-foreground'
                : 'border-primary/30 bg-primary/10 text-foreground',
            )}
          >
            {feedback.tone === 'err' && <AlertOctagon className="mt-px h-3.5 w-3.5 shrink-0" />}
            <span className="font-mono">{feedback.msg}</span>
          </div>
        )}
      </CardContent>
    </Card>
  )
}

// Preset palette — mirrors light_strip_bridge's status colours so a manual
// pick reads the same as the mission state it stands for.
const PRESETS: { label: string; rgb: Rgb }[] = [
  { label: 'Red', rgb: { r: 255, g: 0, b: 0 } },
  { label: 'Orange', rgb: { r: 255, g: 128, b: 0 } },
  { label: 'Amber', rgb: { r: 255, g: 191, b: 0 } },
  { label: 'Yellow', rgb: { r: 255, g: 255, b: 0 } },
  { label: 'Green', rgb: { r: 0, g: 255, b: 0 } },
  { label: 'Spring', rgb: { r: 0, g: 255, b: 128 } },
  { label: 'Teal', rgb: { r: 0, g: 180, b: 140 } },
  { label: 'Cyan', rgb: { r: 0, g: 255, b: 255 } },
  { label: 'Blue', rgb: { r: 0, g: 0, b: 255 } },
  { label: 'Purple', rgb: { r: 180, g: 0, b: 255 } },
  { label: 'White', rgb: { r: 255, g: 255, b: 255 } },
  { label: 'Off', rgb: { r: 0, g: 0, b: 0 } },
]

function rgbToCss({ r, g, b }: Rgb): string {
  return `rgb(${r}, ${g}, ${b})`
}

function rgbToHex({ r, g, b }: Rgb): string {
  const h = (n: number) => n.toString(16).padStart(2, '0')
  return `#${h(r)}${h(g)}${h(b)}`
}

function hexToRgb(hex: string): Rgb {
  const m = /^#?([0-9a-f]{2})([0-9a-f]{2})([0-9a-f]{2})$/i.exec(hex.trim())
  if (!m) return { r: 0, g: 0, b: 0 }
  return {
    r: parseInt(m[1], 16),
    g: parseInt(m[2], 16),
    b: parseInt(m[3], 16),
  }
}
