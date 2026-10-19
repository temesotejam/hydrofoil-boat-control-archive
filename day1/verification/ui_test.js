'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const page = fs.readFileSync(path.join(__dirname,
  '../communication/include/production_page_ja.h'), 'utf8');
const script = page.split('<script>')[1].split('</script>')[0];
new vm.Script(script); // Parse the complete UI, including handlers not exercised below.

const defaults = {
  'kp-pitch': .8, 'kd-pitch': .1, 'kp-roll': 1.25, 'kd-roll': .22,
  'kp-height': .75, 'target-height': .45, 'kp-yaw': .9, 'kd-yaw': .12,
  'max-duty': .5, 'vesc-ramp-rise': 2,
  'left-up': 105, 'left-middle': 90, 'left-down': 75,
  'right-up': 75, 'right-middle': 90, 'right-down': 105,
  'rear-left': 75, 'rear-middle': 90, 'rear-right': 105,
};
const fields = Object.fromEntries(Object.entries(defaults)
  .map(([id, value]) => [id, {value: String(value)}]));
const context = vm.createContext({get: id => fields[id]});
for (const prefix of ['const tuningIds=', 'function validTuningClient()', 'function tuningQuery()']) {
  const declaration = script.split('\n').find(line => line.startsWith(prefix));
  assert.ok(declaration, 'Missing real UI declaration: ' + prefix);
  vm.runInContext(declaration, context);
}
const valid = () => vm.runInContext('validTuningClient()', context);
assert.equal(valid(), true, 'All nineteen valid tuning values must be accepted');
const query = new URLSearchParams(vm.runInContext('tuningQuery()', context));
assert.equal(query.size, 19);
for (const name of ['pitch_cutoff_hz', 'roll_cutoff_hz', 'height_cutoff_hz', 'yaw_cutoff_hz']) {
  assert.equal(query.has(name), false, 'Removed LPF parameter: ' + name);
}
for (const [id, bad] of [
  ['target-height', .09], ['target-height', 2.01], ['kd-yaw', 5.01],
  ['max-duty', 1.01], ['vesc-ramp-rise', .09],
  ['left-up', 181], ['right-up', -1], ['rear-right', 181],
]) {
  fields[id].value = String(bad);
  assert.equal(valid(), false, 'Invalid value accepted for ' + id);
  fields[id].value = String(defaults[id]);
}
assert.equal(valid(), true);
console.log('UI: nineteen tuning values accepted, LPF parameters absent, invalid boundaries rejected');
