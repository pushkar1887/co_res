// batch_worker.js: runs one experiment (or the verification suite) off the UI thread.
// Message in: {id}. Messages out: {type:'progress', done, total} and {type:'done', outputs:[{folder, files}]}.
import { runExperiment } from './experiments.js';
import { runVerification } from './verify.js';

self.onmessage = (e) => {
  const { id } = e.data, progress = (done, total) => self.postMessage({ type: 'progress', done, total });
  try {
    const outputs = id === 'V' ? Object.values(runVerification(progress)) : [runExperiment(id, progress)];
    self.postMessage({ type: 'done', outputs });
  } catch (err) {
    self.postMessage({ type: 'error', message: String(err && err.stack || err) });
  }
};
