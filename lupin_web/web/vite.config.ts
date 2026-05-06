import path from 'node:path'
import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import basicSsl from '@vitejs/plugin-basic-ssl'

// LUPIN_TLS=1 turns on a self-signed cert (via @vitejs/plugin-basic-ssl) so the
// HMI can be served over HTTPS. Required on the robot because browsers gate
// navigator.mediaDevices.getUserMedia (mic for voice mode) to secure contexts —
// localhost is exempt, but http://<robot-ip>:8090 is not. Sim/dev keep plain
// HTTP because they're hit at http://localhost.
const TLS_ENABLED = process.env.LUPIN_TLS === '1'

// Same-origin reverse proxies. Both dev and preview share these so the in-app
// defaults can point at /_ros and /_video regardless of which mode is running.
//   /_ros   → rosbridge_websocket on :9090 (with WS upgrade)
//   /_video → web_video_server on :8091 (HTTP + MJPEG long-lived streams)
// When TLS is on, the browser sees wss://<host>:8090/_ros and
// https://<host>:8090/_video; Vite terminates TLS and forwards plain ws/http
// to localhost. Keeping rosbridge / web_video_server unencrypted on loopback
// avoids touching vendor service configs.
const proxy = {
  '/_ros': {
    target: 'ws://localhost:9090',
    ws: true,
    changeOrigin: true,
    rewrite: (p: string) => p.replace(/^\/_ros/, ''),
  },
  '/_video': {
    target: 'http://localhost:8091',
    changeOrigin: true,
    rewrite: (p: string) => p.replace(/^\/_video/, ''),
  },
}

export default defineConfig({
  plugins: [react(), ...(TLS_ENABLED ? [basicSsl()] : [])],
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
    proxy,
  },
  preview: {
    host: true,
    port: 8090,
    strictPort: true,
    proxy,
  },
})
