import { create } from 'zustand';
import { type TelemetryData } from '../lib/api';

export type { TelemetryData };

interface ProbeState {
  temperature: number | null;
  target: number;
  alarm: boolean;
}

interface SmokerState {
  isOnline: boolean;
  isHeatOn: boolean;
  smokerTemperature: number | null;
  smokerTarget: number;
  cookTime: number;
  cookTimer: number;
  dutyCycle: number;
  probe1: ProbeState;
  probe2: ProbeState;
  probe3: ProbeState;
  probe4: ProbeState;
  historicalData: TelemetryData[];
  nextThreshold: number; // Timestamp for the next 1-minute threshold during downsampling
  isConnecting: boolean;
  updateFromTelemetry: (data: TelemetryData) => void;
  initHistoricalData: (data: TelemetryData[]) => void;
  setSmokerTarget: (target: number) => void;
  setProbeTarget: (probeNumber: 1 | 2 | 3 | 4, target: number) => void;
  setCookTimer: (totalSeconds: number) => void;
  setIsOnline: (status: boolean) => void;
  setIsConnecting: (status: boolean) => void;
  _updateSmokerStateWithTelemetry: (data: TelemetryData) => void;
}

export const useSmokerStore = create<SmokerState>((set, get) => ({
  isOnline: false,
  isHeatOn: false,
  smokerTemperature: null,
  smokerTarget: 0,
  cookTime: 0,
  cookTimer: 0,
  dutyCycle: 0,
  probe1: { temperature: null, target: 0, alarm: false },
  probe2: { temperature: null, target: 0, alarm: false },
  probe3: { temperature: null, target: 0, alarm: false },
  probe4: { temperature: null, target: 0, alarm: false },
  historicalData: [],
  nextThreshold: 0,
  isConnecting: true,

  _updateSmokerStateWithTelemetry: (data: TelemetryData) => {
    const MIN_TEMP = 37.0;
    const isHeatOn = data.smokerTarget > MIN_TEMP;

    set({
      isHeatOn,
      smokerTemperature: data.smokerTemperature,
      smokerTarget: data.smokerTarget,
      cookTime: data.cookTime || 0,
      cookTimer: data.cookTimer || 0,
      dutyCycle: data.dutyCycle || 0,
      probe1: {
        temperature: data.probe1Temperature ?? null,
        target: data.probe1Target || 0,
        alarm: data.probe1Alarm || false,
      },
      probe2: {
        temperature: data.probe2Temperature ?? null,
        target: data.probe2Target || 0,
        alarm: data.probe2Alarm || false,
      },
      probe3: {
        temperature: data.probe3Temperature ?? null,
        target: data.probe3Target || 0,
        alarm: data.probe3Alarm || false,
      },
      probe4: {
        temperature: data.probe4Temperature ?? null,
        target: data.probe4Target || 0,
        alarm: data.probe4Alarm || false,
      },
    });
  },

  updateFromTelemetry: (data: TelemetryData) => {
    if (!data.timestamp) {
      console.warn('Received telemetry data without timestamp, ignoring:', data);
      return;
    }

    const currentState = get();
    let updatedHistoricalData = currentState.historicalData;
    let updatedNextThreshold = currentState.nextThreshold;

    // The Downsampling Check
    if (data.timestamp >= currentState.nextThreshold) {
      updatedHistoricalData = [...currentState.historicalData, data].slice(-360);
      updatedNextThreshold = data.timestamp - (data.timestamp % 60000) + 60000;
    }

    // Update the fast-changing properties directly
    get()._updateSmokerStateWithTelemetry(data);

    // These only change once a minute to keep Recharts performing smoothly
    set({
      historicalData: updatedHistoricalData,
      nextThreshold: updatedNextThreshold,
    });
  },

  initHistoricalData: (data: TelemetryData[]) => {
    // 1. Filter out any corrupt data without a timestamp and sort chronologically
    const processedData = data
      .filter(item => item.timestamp)
      .sort((a, b) => a.timestamp! - b.timestamp!);

    // Update UI with the last historical record first for responsiveness
    if (processedData.length > 0) {
      get()._updateSmokerStateWithTelemetry(processedData[processedData.length - 1]);
    }

    // 2. The 1-Minute Downsampling Algorithm
    const downsampled: TelemetryData[] = [];
    let nextThreshold = processedData.length > 0 ? processedData[0].timestamp! - (processedData[0].timestamp! % 60000) : 0;

    for (const item of processedData) {
      if (item.timestamp! >= nextThreshold) {
        // We crossed the threshold, so keep this record!
        downsampled.push(item);
        
        // Calculate the next threshold: 
        // Snap the current timestamp to the :00 second mark of its minute, 
        // then add exactly 1 minute (60000ms) to set the next tripwire.
        nextThreshold = item.timestamp! - (item.timestamp! % 60000) + 60000;
      }
    }

    // 3. Save the clean, lightweight array to state
    set({ nextThreshold: nextThreshold, historicalData: downsampled });
  },

  setIsOnline(status: boolean) {
    set({ isOnline: status });
  },

  setSmokerTarget: (target: number) => set({ smokerTarget: target }),

  setProbeTarget: (probeNumber: 1 | 2 | 3 | 4, target: number) => {
    set(state => {
      switch(probeNumber) {
        case 1:
          return { ...state, probe1: { ...state.probe1, target } };
        case 2:
          return { ...state, probe2: { ...state.probe2, target } };
        case 3:
          return { ...state, probe3: { ...state.probe3, target } };
        case 4:
          return { ...state, probe4: { ...state.probe4, target } };
        default:
          return state;
      }
    });
  },

  setCookTimer: (totalSeconds: number) => set({ cookTimer: totalSeconds }),

  setIsConnecting: (status: boolean) => set({ isConnecting: status }),
}));
