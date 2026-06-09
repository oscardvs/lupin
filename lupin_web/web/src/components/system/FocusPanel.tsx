import * as Dialog from '@radix-ui/react-dialog'
import { X } from 'lucide-react'
import {
  createContext,
  useCallback,
  useContext,
  useMemo,
  useState,
  type ReactNode,
} from 'react'

/**
 * FocusPanel — one shared in-app "maximize" surface for the vision panels
 * (map, camera, twin, lidar). A Radix Dialog rendered as a 92vw×90vh
 * glass-strong stage that stays WITHIN the app shell, so E-stop / Take-Control
 * never disappear (unlike native fullscreen). Each panel supplies a render
 * function that mounts the SAME widget in an interactive/large variant; the
 * widget owns its own zoom/pan/orbit.
 */
interface FocusContent {
  title: string
  subtitle?: string
  render: (ctx: { close: () => void }) => ReactNode
  /** Optional controls rendered in the header rail (e.g. zoom −/＋/reset). */
  controls?: ReactNode
}

interface FocusApi {
  open: (content: FocusContent) => void
  close: () => void
}

const FocusCtx = createContext<FocusApi | null>(null)

export function useFocusPanel(): FocusApi {
  const api = useContext(FocusCtx)
  if (!api) throw new Error('useFocusPanel must be used within <FocusPanelProvider>')
  return api
}

export function FocusPanelProvider({ children }: { children: ReactNode }) {
  const [content, setContent] = useState<FocusContent | null>(null)
  const close = useCallback(() => setContent(null), [])
  const api = useMemo<FocusApi>(() => ({ open: setContent, close }), [close])

  return (
    <FocusCtx.Provider value={api}>
      {children}
      <Dialog.Root open={!!content} onOpenChange={(o) => { if (!o) close() }}>
        <Dialog.Portal>
          <Dialog.Overlay className="fixed inset-0 z-[80] bg-[hsl(var(--ink-0)/0.82)] backdrop-blur-sm data-[state=open]:animate-in data-[state=closed]:animate-out data-[state=closed]:fade-out-0 data-[state=open]:fade-in-0" />
          <Dialog.Content
            aria-describedby={undefined}
            className="glass-strong fixed left-1/2 top-1/2 z-[81] flex h-[90vh] w-[92vw] max-w-[1600px] -translate-x-1/2 -translate-y-1/2 flex-col overflow-hidden rounded-sm border border-hairline shadow-[0_24px_80px_-20px_hsl(var(--ink-0))] data-[state=open]:animate-in data-[state=closed]:animate-out data-[state=closed]:fade-out-0 data-[state=open]:fade-in-0 data-[state=open]:zoom-in-95"
          >
            <div className="flex shrink-0 items-center gap-3 border-b border-hairline px-4 py-2.5">
              <Dialog.Title className="font-display text-[18px] leading-none text-foreground">
                {content?.title}
              </Dialog.Title>
              {content?.subtitle ? <span className="tag tag-accent">{content.subtitle}</span> : null}
              <div className="ml-auto flex items-center gap-2">
                {content?.controls}
                <Dialog.Close
                  aria-label="Close"
                  className="inline-flex h-8 w-8 items-center justify-center rounded-sm border border-hairline text-muted-foreground transition-colors hover:bg-ink-3 hover:text-foreground"
                >
                  <X className="h-4 w-4" />
                </Dialog.Close>
              </div>
            </div>
            <div className="relative min-h-0 flex-1 bg-ink-0">
              {content ? content.render({ close }) : null}
            </div>
          </Dialog.Content>
        </Dialog.Portal>
      </Dialog.Root>
    </FocusCtx.Provider>
  )
}
