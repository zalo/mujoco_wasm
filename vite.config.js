export default {
  server: {
    fs: {
      allow: ['..']
    }
  },
  assetsInclude: ['**/*.onnx', '**/*.wasm'],
  optimizeDeps: {
    exclude: ['onnxruntime-web']
  },
  build: {
    commonjsOptions: {
      include: [/onnxruntime-web/, /node_modules/]
    }
  },
  // Configure headers for .wasm files
  headers: {
    '**/*.wasm': {
      'Content-Type': 'application/wasm'
    }
  }
} 