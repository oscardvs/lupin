import { Moon, RotateCcw, Sun } from 'lucide-react'

import { Button } from '@/components/ui/button'
import { Input } from '@/components/ui/input'
import { Label } from '@/components/ui/label'
import { ScrollArea } from '@/components/ui/scroll-area'
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from '@/components/ui/select'
import { Separator } from '@/components/ui/separator'
import { Sheet, SheetContent, SheetDescription, SheetHeader, SheetTitle } from '@/components/ui/sheet'
import { Switch } from '@/components/ui/switch'
import { useSettings } from '@/lib/settings'

interface SettingsDrawerProps {
  open: boolean
  onOpenChange: (open: boolean) => void
}

export function SettingsDrawer({ open, onOpenChange }: SettingsDrawerProps) {
  const [settings, update, reset] = useSettings()

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
                  placeholder="ws://host:9090"
                />
              </Field>
              <Field label="web_video_server URL">
                <Input
                  value={settings.webVideoServerUrl}
                  onChange={(e) => update({ webVideoServerUrl: e.target.value })}
                  placeholder="http://host:8080"
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
