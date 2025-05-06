# Quick Start

```bash
python examples/scenes/generate_index.py

npm install
npm run dev
```


# Features

- [x] ONNX Model Inference
- [x] Policy Switching
- [x] Composable Observation


# Known Issues

- The `unitree_g1` model loading is not yet supported.
- The `await new Promise(resolve => setTimeout(resolve, 10));` is used to wait align the simulation speed with wall clock time, which is not precise.
- The `reloadPolicy` function use wait until the inference is finished, should replace with thread lock.
