// server/src/collagen/collagenService.ts
import { telemetryEmitter } from '../utils/eventEmitter';
import { SmokerTelemetryData } from '../types';

const SAMPLE_INTERVAL_MS = 15 * 60 * 1000; // 15 minutes

// Collagen rendering rates from https://smoketrailsbbq.com/brisket-holding-masterclass-and-tenderness-model/
const RATE_TABLE_F: { tempF: number; ratePerHour: number }[] = [
  { tempF: 140, ratePerHour: 1 },
  { tempF: 150, ratePerHour: 2 },
  { tempF: 160, ratePerHour: 3 },
  { tempF: 170, ratePerHour: 5 },
  { tempF: 180, ratePerHour: 9 },
  { tempF: 190, ratePerHour: 18 },
  { tempF: 195, ratePerHour: 25 },
  { tempF: 200, ratePerHour: 35 },
  { tempF: 205, ratePerHour: 55 },
  { tempF: 210, ratePerHour: 75 },
];

interface CollagenState {
  cumulativePercent: number;
  lastSampleTimestamp: number;
  lastCookTimerValue: number;
}

interface FitResult {
  lnA: number;
  b: number;
}

function fitExponentialCurve(table: { tempF: number; ratePerHour: number }[]): FitResult {
  const n = table.length;
  let sumT = 0;
  let sumLnRate = 0;
  let sumTLnRate = 0;
  let sumT2 = 0;

  for (const { tempF, ratePerHour } of table) {
    const lnRate = Math.log(ratePerHour);
    sumT += tempF;
    sumLnRate += lnRate;
    sumTLnRate += tempF * lnRate;
    sumT2 += tempF * tempF;
  }

  const b = (n * sumTLnRate - sumT * sumLnRate) / (n * sumT2 - sumT * sumT);
  const lnA = (sumLnRate - b * sumT) / n;

  return { lnA, b };
}

const collagenStates = new Map<string, CollagenState>();
const { lnA, b } = fitExponentialCurve(RATE_TABLE_F);
const a = Math.exp(lnA);

function renderRatePerHour(tempF: number): number {
  if (tempF < 140) return 0;
  return a * Math.exp(b * tempF);
}

export class CollagenCalculator {
  recordSample(data: SmokerTelemetryData): void {
    if (data.probe1Temperature === undefined) {
      return;
    }

    const state = collagenStates.get(data.smokerId) || {
      cumulativePercent: 0,
      lastSampleTimestamp: data.timestamp,
      lastCookTimerValue: data.cookTimer || 0,
    };

    // Cook-reset heuristic: cookTimer transition from <= 0 to > 0 signals new cook
    if (state.lastCookTimerValue <= 0 && (data.cookTimer || 0) > 0) {
      console.log(`[Collagen] smokerId=${data.smokerId} COOK START DETECTED - resetting cumulative to 0%`);
      state.cumulativePercent = 0;
      state.lastSampleTimestamp = data.timestamp;
      state.lastCookTimerValue = data.cookTimer || 0;
      collagenStates.set(data.smokerId, state);
      return;
    }

    // First sample for this smoker: just initialize, no integration yet
    if (!collagenStates.has(data.smokerId)) {
      collagenStates.set(data.smokerId, state);
      return;
    }

    // Check if enough time has elapsed for a sample
    const elapsedMs = data.timestamp - state.lastSampleTimestamp;
    const rate = renderRatePerHour(data.probe1Temperature);

    if (elapsedMs >= SAMPLE_INTERVAL_MS) {
      const elapsedHours = elapsedMs / (1000 * 60 * 60);
      const percentageGained = rate * elapsedHours;
      state.cumulativePercent += percentageGained;
      state.lastSampleTimestamp = data.timestamp;
      console.log(`[Collagen] smokerId=${data.smokerId} temp=${data.probe1Temperature}°F rate=${rate.toFixed(2)}%/hr elapsedHours=${elapsedHours.toFixed(3)} percentageGained=${percentageGained.toFixed(3)}% cumulative=${state.cumulativePercent.toFixed(2)}%`);
    }

    state.lastCookTimerValue = data.cookTimer || 0;
    collagenStates.set(data.smokerId, state);
  }

  getCollagenPercentage(smokerId: string): number | undefined {
    return collagenStates.get(smokerId)?.cumulativePercent;
  }
}

export const collagenCalculator = new CollagenCalculator();

export function initCollagenService(): void {
  telemetryEmitter.on('telemetry', (data: SmokerTelemetryData) => {
    collagenCalculator.recordSample(data);
  });
}
