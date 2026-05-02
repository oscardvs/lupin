import path from 'node:path'
import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig({
  plugins: [react()],
  resolve: {
    alias: {
      '@': path.resolve(__dirname, './src'),
    },
  },
  // roslib 1.4 opens its source with `var ROSLIB = this.ROSLIB || {...}`,
  // expecting CJS top-level `this` to be `module.exports`. In an ES-module
  // bundle (what Vite emits) top-level `this` is `undefined`, so the
  // property access throws "Cannot read properties of undefined (reading
  // 'ROSLIB')" before the `||` fallback can fire. Substitute the literal
  // `this.ROSLIB` at build time so the fallback kicks in cleanly.
  define: {
    'this.ROSLIB': 'undefined',
  },
  server: {
    host: true,
    port: 8090,
    strictPort: true,
  },
  preview: {
    host: true,
    port: 8090,
    strictPort: true,
  },
})
