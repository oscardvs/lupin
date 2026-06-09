import { defineConfig } from 'vitest/config'

// Kept separate from vite.config.ts so the production build (`tsc -b && vite
// build`) is unaffected by test tooling — tsconfig.node.json only compiles
// vite.config.ts, so this file is never type-checked into the app bundle.
// Default environment is 'node' (fast); the settings migration test opts into
// jsdom for localStorage via a `// @vitest-environment jsdom` docblock.
export default defineConfig({
  test: {
    environment: 'node',
    include: ['src/**/*.test.ts'],
  },
})
