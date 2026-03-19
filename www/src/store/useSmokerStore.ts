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
  updateFromTelemetry: (data: TelemetryData) => void;
  initHistoricalData: (data: TelemetryData[]) => void;
  setSmokerTarget: (target: number) => void;
  setProbeTarget: (probeNumber: 1 | 2 | 3 | 4, target: number) => void;
  setCookTimer: (totalSeconds: number) => void;
  isConnecting: boolean;
  setIsConnecting: (status: boolean) => void;
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

updateFromTelemetry: (data: TelemetryData) => {

  if (!data.timestamp) {
    console.warn('Received telemetry data without timestamp, ignoring:', data);
    return;
  }

    const MIN_TEMP = 37.0;
    const isHeatOn = data.smokerTarget > MIN_TEMP;
    
    // 2. Default to keeping the chart exactly as it is
    const currentState = get();
    let updatedHistoricalData = currentState.historicalData;
    let updatedNextThreshold = currentState.nextThreshold;

    // 3. The Downsampling Check
    if (data.timestamp >= currentState.nextThreshold) {
      // The minute has rolled over! Append the new dot and enforce the 6-hour memory limit (360 points)
      updatedHistoricalData = [...currentState.historicalData, data].slice(-360);
      
      // Calculate the next 1-minute tripwire
      updatedNextThreshold = data.timestamp - (data.timestamp % 60000) + 60000;
    }

    // 4. Update the Store
    set({
      // These fast-changing properties update every 5 seconds for your UI dials
      isOnline: true,
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
      
      // These only change once a minute to keep Recharts performing smoothly
      historicalData: updatedHistoricalData,
      nextThreshold: updatedNextThreshold,
    });
  },

  initHistoricalData: (data: TelemetryData[]) => {
    // 1. Filter out any corrupt data without a timestamp and sort chronologically
    const processedData = data
      .filter(item => item.timestamp)
      .sort((a, b) => a.timestamp! - b.timestamp!);

    // 2. The 1-Minute Downsampling Algorithm
    const downsampled: TelemetryData[] = [];
    let nextThreshold = get().nextThreshold || 0;

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

  isConnecting: true, // Default to true on initial load
  setIsConnecting: (status: boolean) => set({ isConnecting: status }),
}));
