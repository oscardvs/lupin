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
    // web_video_server (8091) doesn't return CORS headers, so a cross-origin
    // fetch from the UI to scrape its topic-list HTML is blocked by the
    // browser. <img src=…> is fine cross-origin, but discovery isn't.
    // Proxy it through the UI's own origin so the fetch is same-origin.
    proxy: {
      '/_video': {
        target: 'http://localhost:8091',
        changeOrigin: true,
        rewrite: (p) => p.replace(/^\/_video/, ''),
      },
    },
  },
  preview: {
    host: true,
    port: 8090,
    strictPort: true,
    proxy: {
      '/_video': {
        target: 'http://localhost:8091',
        changeOrigin: true,
        rewrite: (p) => p.replace(/^\/_video/, ''),
      },
    },
  },
})
