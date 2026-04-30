import { CameraStream } from '@/components/widgets/CameraStream'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { useSettings } from '@/lib/settings'

const KNOWN_CAMERAS = [{ id: 'rgb', label: 'RGB', topicKey: 'cameraTopic' as const }]

export function CamerasView() {
  const [{ cameraTopic, webVideoServerUrl }] = useSettings()

  return (
    <div className="flex h-full min-h-0 w-full flex-col gap-3 p-3 sm:p-4">
      <Tabs defaultValue="rgb" className="flex flex-1 min-h-0 flex-col">
        <TabsList className="self-start">
          {KNOWN_CAMERAS.map((c) => (
            <TabsTrigger key={c.id} value={c.id}>
              {c.label}
            </TabsTrigger>
          ))}
        </TabsList>
        <TabsContent value="rgb" className="m-0 mt-2 flex flex-1 min-h-0">
          <CameraStream
            label="RGB"
            topic={cameraTopic}
            baseUrl={webVideoServerUrl}
            className="flex-1 min-h-[14rem]"
          />
        </TabsContent>
      </Tabs>
    </div>
  )
}
