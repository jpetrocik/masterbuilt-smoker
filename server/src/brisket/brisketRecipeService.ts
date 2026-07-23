// server/src/brisket/brisketRecipeService.ts
import { telemetryEmitter } from '../utils/eventEmitter';
import { publishCommand } from '../mqtt/mqttService';
import { SmokerTelemetryData } from '../types';

const SAMPLE_INTERVAL_MS = 15 * 60 * 1000; // 15 minutes
const COLLAGEN_COMPLETION_THRESHOLD = 90; // percent
const BRISKET_FINISHED_TEMP_F = 200; // probe target when resting

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

export interface BrisketRecipeState {
  active: boolean;
  phase: 'cooking' | 'wrapped' | 'resting' | 'finished'; // descriptive label only, not a precondition
  cumulativeCollagenPercent: number;
  lastSampleTimestamp: number;
  wrappedNotified: boolean;
  restingNotified: boolean;
  finishedNotified: boolean;
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

const recipeStates = new Map<string, BrisketRecipeState>();
const { lnA, b } = fitExponentialCurve(RATE_TABLE_F);
const a = Math.exp(lnA);

function renderRatePerHour(tempF: number): number {
  if (tempF < 140) return 0;
  return a * Math.exp(b * tempF);
}

export function startBrisketRecipe(smokerId: string): void {
  console.log(`[BrisketRecipe] Starting recipe for smokerId=${smokerId}`);

  const state: BrisketRecipeState = {
    active: true,
    phase: 'cooking',
    cumulativeCollagenPercent: 0,
    lastSampleTimestamp: Date.now(),
    wrappedNotified: false,
    restingNotified: false,
    finishedNotified: false,
  };

  recipeStates.set(smokerId, state);

  // Publish initial commands
  publishCommand(smokerId, 'setTemp=225');
  publishCommand(smokerId, 'setProbe1Target=200');

  console.log(`[BrisketRecipe] smokerId=${smokerId} initialized to 225°F pit, 200°F probe target`);
}

export function stopBrisketRecipe(smokerId: string): void {
  if (recipeStates.has(smokerId)) {
    recipeStates.delete(smokerId);
    console.log(`[BrisketRecipe] Stopped recipe for smokerId=${smokerId}`);
  }
}

export function getBrisketRecipeState(smokerId: string): BrisketRecipeState | undefined {
  return recipeStates.get(smokerId);
}

export function initBrisketRecipeService(): void {
  telemetryEmitter.on('telemetry', (data: SmokerTelemetryData) => {
    const state = recipeStates.get(data.smokerId);

    // Skip if no active recipe for this smoker
    if (!state || !state.active) {
      return;
    }

    // Skip if no probe1 temperature reading
    if (data.probe1Temperature === undefined) {
      return;
    }

    // Check if enough time has elapsed for a sample
    const elapsedMs = data.timestamp - state.lastSampleTimestamp;
    const rate = renderRatePerHour(data.probe1Temperature);

    if (elapsedMs >= SAMPLE_INTERVAL_MS) {
      const elapsedHours = elapsedMs / (1000 * 60 * 60);
      const percentageGained = rate * elapsedHours;
      state.cumulativeCollagenPercent += percentageGained;
      state.lastSampleTimestamp = data.timestamp;

      console.log(`[BrisketRecipe] smokerId=${data.smokerId} phase=${state.phase} temp=${data.probe1Temperature}°F rate=${rate.toFixed(2)}%/hr collagen=${state.cumulativeCollagenPercent.toFixed(2)}%`);

      // Check if resting should start: collagen >= 90% OR probe already reached target temp
      if (!state.restingNotified && (state.cumulativeCollagenPercent >= COLLAGEN_COMPLETION_THRESHOLD || data.probe1Temperature >= BRISKET_FINISHED_TEMP_F)) {
        state.restingNotified = true;
        state.phase = 'resting';
        publishCommand(data.smokerId, 'setTemp=150');
        console.log(`[BrisketRecipe] smokerId=${data.smokerId} resting phase started (collagen=${state.cumulativeCollagenPercent.toFixed(2)}%, temp=${data.probe1Temperature}°F) — lowering temp to 150°F`);
        telemetryEmitter.emit('sendNotification', data.smokerId, 'Brisket Resting', 'Collagen rendering is complete — your brisket is resting.');
      }

      // Check if brisket has finished (reached target temp)
      if (!state.finishedNotified && data.probe1Temperature >= BRISKET_FINISHED_TEMP_F) {
        state.finishedNotified = true;
        state.phase = 'finished';
        console.log(`[BrisketRecipe] smokerId=${data.smokerId} brisket finished — probe reached ${data.probe1Temperature}°F`);
        telemetryEmitter.emit('sendNotification', data.smokerId, 'Brisket Finished', 'Your brisket has finished cooking and is ready to serve.');
      }
    }
  });

  telemetryEmitter.on('stallDetected', (smokerId: string, probeIndex: number, probeTemp: number) => {
    const state = recipeStates.get(smokerId);

    // Skip if no active recipe, not probe 1, or already wrapped
    if (!state || !state.active || probeIndex !== 1 || state.wrappedNotified) {
      return;
    }

    // Transition to wrapped phase
    state.wrappedNotified = true;
    state.phase = 'wrapped';
    publishCommand(smokerId, 'setTemp=250');
    console.log(`[BrisketRecipe] smokerId=${smokerId} stall detected at ${probeTemp}°F — transitioning to wrapped phase, raising temp to 250°F`);
    telemetryEmitter.emit('sendNotification', smokerId, 'Wrap Your Brisket!', `Stall detected at ${probeTemp}°F — time to wrap.`);
  });
}
