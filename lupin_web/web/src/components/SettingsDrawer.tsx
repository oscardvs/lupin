import { Eye, EyeOff, Moon, RotateCcw, Sun } from 'lucide-react'
import { useState } from 'react'

import { Button } from '@/components/ui/button'
import { Input } from '@/components/ui/input'
import { Label } from '@/components/ui/label'
import { ScrollArea } from '@/components/ui/scroll-area'
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from '@/components/ui/select'
import { Separator } from '@/components/ui/separator'
import { Sheet, SheetContent, SheetDescription, SheetHeader, SheetTitle } from '@/components/ui/sheet'
import { Switch } from '@/components/ui/switch'
import { Textarea } from '@/components/ui/textarea'
import { useSettings, type VoiceNamedLocation } from '@/lib/settings'

interface SettingsDrawerProps {
  open: boolean
  onOpenChange: (open: boolean) => void
}

export function SettingsDrawer({ open, onOpenChange }: SettingsDrawerProps) {
  const [settings, update, reset] = useSettings()
  const [showKey, setShowKey] = useState(false)
  const [locationsDraft, setLocationsDraft] = useState<string>(() =>
    JSON.stringify(settings.voiceNamedLocations, null, 2),
  )
  const [locationsError, setLocationsError] = useState<string | null>(null)

  const commitLocations = (raw: string) => {
    setLocationsDraft(raw)
    try {
      const parsed = JSON.parse(raw) as Record<string, VoiceNamedLocation>
      // Lightweight shape check — anything else is the model's problem at runtime.
      for (const [name, p] of Object.entries(parsed)) {
        if (typeof p?.x !== 'number' || typeof p?.y !== 'number' || typeof p?.yaw !== 'number') {
          throw new Error(`"${name}" is missing numeric x/y/yaw`)
        }
      }
      update({ voiceNamedLocations: parsed })
      setLocationsError(null)
    } catch (e) {
      setLocationsError(e instanceof Error ? e.message : String(e))
    }
  }

  return (
    <Sheet open={open} onOpenChange={onOpenChange}>
      <SheetContent side="right" className="flex w-full flex-col gap-0 p-0 sm:max-w-md">
        <SheetHeader className="border-b p-4">
          <SheetTitle>Settings</SheetTitle>
          <SheetDescription>
            Persisted to localStorage. Reload to re-detect rosbridge URL from the page host.
          </SheetDescription>
        </SheetHeader>

        <ScrollArea className="flex-1">
          <div className="flex flex-col gap-6 p-4">
            <Section title="Connection">
              <Field label="rosbridge URL">
                <Input
                  value={settings.rosUrl}
                  onChange={(e) => update({ rosUrl: e.target.value })}
                  placeholder="ws(s)://host:8090/_ros or ws://host:9090"
                />
              </Field>
              <Field label="web_video_server URL">
                <Input
                  value={settings.webVideoServerUrl}
                  onChange={(e) => update({ webVideoServerUrl: e.target.value })}
                  placeholder="http(s)://host:8090/_video or http://host:8091"
                />
              </Field>
            </Section>

            <Section title="Drive">
              <Field label="cmd_vel topic">
                <Input
                  value={settings.cmdVelTopic}
                  onChange={(e) => update({ cmdVelTopic: e.target.value })}
                />
              </Field>
              <Field label="cmd_vel message type">
                <Select
                  value={settings.cmdVelType}
                  onValueChange={(v) => update({ cmdVelType: v as typeof settings.cmdVelType })}
                >
                  <SelectTrigger><SelectValue /></SelectTrigger>
                  <SelectContent>
                    <SelectItem value="geometry_msgs/msg/Twist">geometry_msgs/msg/Twist</SelectItem>
                    <SelectItem value="geometry_msgs/msg/TwistStamped">geometry_msgs/msg/TwistStamped</SelectItem>
                  </SelectContent>
                </Select>
              </Field>
            </Section>

            <Section title="Topics">
              <Field label="IMU"><Input value={settings.imuTopic} onChange={(e) => update({ imuTopic: e.target.value })} /></Field>
              <Field label="Lidar (LaserScan)"><Input value={settings.scanTopic} onChange={(e) => update({ scanTopic: e.target.value })} /></Field>
              <Field label="Odometry"><Input value={settings.odomTopic} onChange={(e) => update({ odomTopic: e.target.value })} /></Field>
              <Field label="Joint states"><Input value={settings.jointStatesTopic} onChange={(e) => update({ jointStatesTopic: e.target.value })} /></Field>
              <Field label="Battery"><Input value={settings.batteryTopic} onChange={(e) => update({ batteryTopic: e.target.value })} /></Field>
              <Field label="rosout"><Input value={settings.rosoutTopic} onChange={(e) => update({ rosoutTopic: e.target.value })} /></Field>
              <Field label="Camera"><Input value={settings.cameraTopic} onChange={(e) => update({ cameraTopic: e.target.value })} /></Field>
            </Section>

            <Section title="Arm">
              <Field label="Servo namespace">
                <Input
                  value={settings.armServoNamespace}
                  onChange={(e) => update({ armServoNamespace: e.target.value })}
                  placeholder="/io/servo/hiwonder"
                />
              </Field>
              <Field label="Default rate (deg/s)">
                <Input
                  type="number"
                  min={1}
                  max={360}
                  value={settings.armRateDegPerSec}
                  onChange={(e) => {
                    const n = Number(e.target.value)
                    if (Number.isFinite(n) && n > 0) update({ armRateDegPerSec: n })
                  }}
                />
              </Field>
            </Section>

            <Section title="Appearance">
              <Field label="Theme">
                <div className="flex gap-2">
                  <Button
                    variant={settings.theme === 'dark' ? 'default' : 'outline'}
                    size="sm"
                    onClick={() => update({ theme: 'dark' })}
                  >
                    <Moon className="mr-2 h-4 w-4" /> Dark
                  </Button>
                  <Button
                    variant={settings.theme === 'light' ? 'default' : 'outline'}
                    size="sm"
                    onClick={() => update({ theme: 'light' })}
                  >
                    <Sun className="mr-2 h-4 w-4" /> Light
                  </Button>
                </div>
              </Field>
            </Section>

            <Section title="Voice (Gemini Live)">
              <Field label="Google AI Studio API key">
                <div className="flex gap-2">
                  <Input
                    type={showKey ? 'text' : 'password'}
                    value={settings.geminiApiKey}
                    onChange={(e) => update({ geminiApiKey: e.target.value })}
                    placeholder="AIza…"
                    autoComplete="off"
                    spellCheck={false}
                  />
                  <Button
                    type="button"
                    variant="outline"
                    size="icon"
                    onClick={() => setShowKey((s) => !s)}
                    aria-label={showKey ? 'Hide key' : 'Show key'}
                  >
                    {showKey ? <EyeOff className="h-4 w-4" /> : <Eye className="h-4 w-4" />}
                  </Button>
                </div>
                <p className="text-[11px] text-muted-foreground">
                  Stored in localStorage on this device only. Never committed, never sent to the
                  robot. Get one at <span className="font-mono">aistudio.google.com</span>.
                </p>
              </Field>
              <Field label="Model">
                <Input
                  value={settings.geminiModel}
                  onChange={(e) => update({ geminiModel: e.target.value })}
                  placeholder="models/gemini-3.1-flash-live-preview"
                  spellCheck={false}
                />
              </Field>
              <Field label="Language (BCP-47)">
                <Input
                  value={settings.voiceLanguage}
                  onChange={(e) => update({ voiceLanguage: e.target.value })}
                  placeholder="en-US"
                  spellCheck={false}
                />
              </Field>
              <div className="flex items-center justify-between">
                <div>
                  <Label htmlFor="voicePushToTalk" className="text-sm">Tap-to-talk</Label>
                  <p className="text-[11px] text-muted-foreground">
                    On → tap the orb to open the mic; the speech endpointer closes it on
                    silence. Off → hands-free (mic stays open while the session is live).
                  </p>
                </div>
                <Switch
                  id="voicePushToTalk"
                  checked={settings.voicePushToTalk}
                  onCheckedChange={(v) => update({ voicePushToTalk: v })}
                />
              </div>
              <div className="flex items-center justify-between">
                <div>
                  <Label htmlFor="voiceVadEnabled" className="text-sm">
                    Auto-stop on silence
                  </Label>
                  <p className="text-[11px] text-muted-foreground">
                    Speech endpointer — closes the mic after a sustained pause so the agent
                    stops listening when you finish your sentence.
                  </p>
                </div>
                <Switch
                  id="voiceVadEnabled"
                  checked={settings.voiceVadEnabled}
                  onCheckedChange={(v) => update({ voiceVadEnabled: v })}
                />
              </div>
              <Field label="Silence hold (ms)">
                <Input
                  type="number"
                  step="50"
                  min="100"
                  max="5000"
                  value={settings.voiceVadEndHoldMs}
                  onChange={(e) =>
                    update({ voiceVadEndHoldMs: Math.max(100, Number(e.target.value) || 800) })
                  }
                />
                <p className="text-[11px] text-muted-foreground">
                  Continuous quiet duration that ends an utterance. Lower = snappier;
                  higher = more tolerant of mid-sentence pauses.
                </p>
              </Field>
              <Field label="Speech-start threshold (RMS)">
                <Input
                  type="number"
                  step="0.005"
                  min="0"
                  max="0.5"
                  value={settings.voiceVadStartThreshold}
                  onChange={(e) =>
                    update({
                      voiceVadStartThreshold: Math.max(0, Number(e.target.value) || 0.02),
                    })
                  }
                />
              </Field>
              <Field label="Speech-end threshold (RMS)">
                <Input
                  type="number"
                  step="0.005"
                  min="0"
                  max="0.5"
                  value={settings.voiceVadEndThreshold}
                  onChange={(e) =>
                    update({
                      voiceVadEndThreshold: Math.max(0, Number(e.target.value) || 0.012),
                    })
                  }
                />
                <p className="text-[11px] text-muted-foreground">
                  Set above your room noise floor. End threshold should sit a bit below
                  start to give clean hysteresis (typical: 0.02 / 0.012).
                </p>
              </Field>
              <Field label="Max utterance (ms)">
                <Input
                  type="number"
                  step="500"
                  min="1000"
                  max="60000"
                  value={settings.voiceVadMaxUtteranceMs}
                  onChange={(e) =>
                    update({
                      voiceVadMaxUtteranceMs: Math.max(1000, Number(e.target.value) || 15000),
                    })
                  }
                />
              </Field>
              <Field label="Max linear speed (m/s)">
                <Input
                  type="number"
                  step="0.05"
                  min="0"
                  value={settings.voiceMaxLinearMps}
                  onChange={(e) => update({ voiceMaxLinearMps: Number(e.target.value) || 0 })}
                />
              </Field>
              <Field label="Max angular speed (rad/s)">
                <Input
                  type="number"
                  step="0.1"
                  min="0"
                  value={settings.voiceMaxAngularRps}
                  onChange={(e) => update({ voiceMaxAngularRps: Number(e.target.value) || 0 })}
                />
              </Field>
              <Field label="System prompt">
                <Textarea
                  value={settings.voiceSystemPrompt}
                  onChange={(e) => update({ voiceSystemPrompt: e.target.value })}
                  rows={5}
                />
              </Field>
              <Field label="Named locations (JSON)">
                <Textarea
                  value={locationsDraft}
                  onChange={(e) => commitLocations(e.target.value)}
                  rows={6}
                  className="font-mono text-[12px]"
                  spellCheck={false}
                />
                {locationsError ? (
                  <p className="text-[11px] text-destructive">{locationsError}</p>
                ) : (
                  <p className="text-[11px] text-muted-foreground">
                    Map-frame poses the agent can navigate to via name.
                  </p>
                )}
              </Field>
            </Section>

            <Section title="Safety">
              <div className="flex items-center justify-between">
                <div>
                  <Label htmlFor="estopAutoOnFocusLoss" className="text-sm">
                    Auto E-stop on focus loss
                  </Label>
                  <p className="text-[11px] text-muted-foreground">
                    Trigger E-stop when the tab is hidden or the window loses focus. Page-unload and
                    rosbridge-disconnect always fire regardless. Disable during dev to stop alt-tab
                    from constantly tripping the stop.
                  </p>
                </div>
                <Switch
                  id="estopAutoOnFocusLoss"
                  checked={settings.estopAutoOnFocusLoss}
                  onCheckedChange={(v) => update({ estopAutoOnFocusLoss: v })}
                />
              </div>
            </Section>

            <Section title="Diagnostics">
              <div className="flex items-center justify-between">
                <div>
                  <Label htmlFor="debugPublish" className="text-sm">Log outgoing publishes</Label>
                  <p className="text-[11px] text-muted-foreground">
                    Stamps every Twist / publish call to the browser console with full JSON. Useful for
                    diagnosing rosbridge wire-format issues (e.g. unexpected sign on linear.x).
                  </p>
                </div>
                <Switch
                  id="debugPublish"
                  checked={settings.debugPublish}
                  onCheckedChange={(v) => update({ debugPublish: v })}
                />
              </div>
            </Section>

            <Separator />

            <Button variant="outline" onClick={reset} className="self-start">
              <RotateCcw className="mr-2 h-4 w-4" /> Reset all settings
            </Button>
          </div>
        </ScrollArea>
      </SheetContent>
    </Sheet>
  )
}

function Section({ title, children }: { title: string; children: React.ReactNode }) {
  return (
    <div className="flex flex-col gap-3">
      <div className="text-xs font-semibold uppercase tracking-wider text-muted-foreground">{title}</div>
      {children}
    </div>
  )
}

function Field({ label, children }: { label: string; children: React.ReactNode }) {
  return (
    <div className="flex flex-col gap-1.5">
      <Label className="text-xs">{label}</Label>
      {children}
    </div>
  )
}
