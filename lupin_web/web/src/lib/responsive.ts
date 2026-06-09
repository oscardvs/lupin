import { useEffect, useState } from 'react'

/**
 * useMediaQuery — re-renders on a CSS media query toggle. SSR-safe (returns
 * `false` on the server / during the first render before hydration).
 */
export function useMediaQuery(query: string): boolean {
  const [matches, setMatches] = useState<boolean>(() => {
    if (typeof window === 'undefined') return false
    return window.matchMedia(query).matches
  })

  useEffect(() => {
    if (typeof window === 'undefined') return
    const mql = window.matchMedia(query)
    const handler = (e: MediaQueryListEvent) => setMatches(e.matches)
    setMatches(mql.matches)
    mql.addEventListener('change', handler)
    return () => mql.removeEventListener('change', handler)
  }, [query])

  return matches
}

/** Tailwind sm: breakpoint — landscape phone and larger. */
export const useIsSm = () => useMediaQuery('(min-width: 640px)')
/** Tailwind md: breakpoint. */
export const useIsMd = () => useMediaQuery('(min-width: 768px)')
/** Tailwind lg: breakpoint. */
export const useIsLg = () => useMediaQuery('(min-width: 1024px)')
/** Tailwind xl: breakpoint — large desktop; drives density up-scaling. */
export const useIsXl = () => useMediaQuery('(min-width: 1280px)')
/** Phone-portrait tier — below Tailwind sm (e.g. iPhone 12 Pro Max = 428w). */
export const useIsPhone = () => useMediaQuery('(max-width: 639px)')
/** Coarse pointer (touch). */
export const useIsTouch = () => useMediaQuery('(pointer: coarse)')
