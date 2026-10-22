// Execute the unchanged, byte-pinned published source reference.
import fs from 'node:fs';
import {referenceDerivatives} from './source-reference.mjs';
const {times, parameters} = JSON.parse(fs.readFileSync(0, 'utf8'));
process.stdout.write(JSON.stringify(times.map(time => referenceDerivatives(time, parameters))));
