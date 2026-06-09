import * as React from 'react'
import { cn } from '@/lib/utils'

type CardProps = React.HTMLAttributes<HTMLDivElement> & {
  /** Apply the decorative `.edge-light` top edge (for hero / vision cards). */
  edge?: boolean
}

const Card = React.forwardRef<HTMLDivElement, CardProps>(
  ({ className, children, edge, ...props }, ref) => (
    <div
      ref={ref}
      className={cn(
        // Opaque base tile — glass is for floating overlays only, never base
        // tiles, so the aurora/grid never bleeds through. A firm hairline plus
        // a 1px inset top highlight give the panel a real elevation edge.
        'reticle relative flex flex-col rounded-sm border-hairline border bg-card text-card-foreground',
        'shadow-[inset_0_1px_0_0_hsl(var(--edge-tile)/0.05),inset_0_0_0_1px_hsl(var(--primary)/0.06),0_8px_24px_-12px_rgba(0,0,0,0.6)]',
        edge && 'edge-light',
        className,
      )}
      {...props}
    >
      <span className="reticle-bl" aria-hidden />
      <span className="reticle-br" aria-hidden />
      {children}
    </div>
  ),
)
Card.displayName = 'Card'

const CardHeader = React.forwardRef<HTMLDivElement, React.HTMLAttributes<HTMLDivElement>>(
  ({ className, ...props }, ref) => (
    <div
      ref={ref}
      className={cn(
        'flex flex-col gap-0.5 border-b border-hairline px-4 py-3',
        className,
      )}
      {...props}
    />
  ),
)
CardHeader.displayName = 'CardHeader'

const CardTitle = React.forwardRef<HTMLDivElement, React.HTMLAttributes<HTMLDivElement>>(
  ({ className, ...props }, ref) => (
    <div
      ref={ref}
      className={cn(
        'flex items-baseline gap-2 text-[15px] font-semibold leading-none tracking-tight',
        className,
      )}
      {...props}
    />
  ),
)
CardTitle.displayName = 'CardTitle'

const CardDescription = React.forwardRef<HTMLDivElement, React.HTMLAttributes<HTMLDivElement>>(
  ({ className, ...props }, ref) => (
    <div ref={ref} className={cn('tag', className)} {...props} />
  ),
)
CardDescription.displayName = 'CardDescription'

const CardContent = React.forwardRef<HTMLDivElement, React.HTMLAttributes<HTMLDivElement>>(
  ({ className, ...props }, ref) => (
    <div ref={ref} className={cn('px-4 py-3', className)} {...props} />
  ),
)
CardContent.displayName = 'CardContent'

const CardFooter = React.forwardRef<HTMLDivElement, React.HTMLAttributes<HTMLDivElement>>(
  ({ className, ...props }, ref) => (
    <div
      ref={ref}
      className={cn('flex items-center border-t border-hairline px-4 py-2.5', className)}
      {...props}
    />
  ),
)
CardFooter.displayName = 'CardFooter'

export { Card, CardHeader, CardFooter, CardTitle, CardDescription, CardContent }
