import * as ort from 'onnxruntime-web';

export class ONNXModule {
  constructor(modelPath) {
    this.modelPath = modelPath;
    this.metaPath = modelPath.replace('.onnx', '.json');
  }

  async init() {
    // Load the ONNX model
    const modelResponse = await fetch(this.modelPath);
    const modelArrayBuffer = await modelResponse.arrayBuffer();
    const meta = await (fetch(this.metaPath).then(response => response.json()));

    this.inKeys = meta["in_keys"];
    this.outKeys = meta["out_keys"];

    // Create session from the array buffer
    this.session = await ort.InferenceSession.create(modelArrayBuffer, {
      executionProviders: ['wasm'],
      graphOptimizationLevel: 'all'
    });

    console.log('ONNX model loaded successfully');
    console.log("inKeys", this.inKeys);
    console.log("outKeys", this.outKeys);

    console.log("inputNames", this.session.inputNames);
    console.log("outputNames", this.session.outputNames);
  }

  async runInference(input) {
    // construct input
    let onnxInput = {};
    for (let i = 0; i < this.inKeys.length; i++) {
      onnxInput[this.session.inputNames[i]] = input[this.inKeys[i]];
    }
    // run inference
    const onnxOutput = await this.session.run(onnxInput);
    // construct output
    let result = {};
    for (let i = 0; i < this.outKeys.length; i++) {
      result[this.outKeys[i]] = onnxOutput[this.session.outputNames[i]].data;
    }
    return result;
  }
}
