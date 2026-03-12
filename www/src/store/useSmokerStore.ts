import { create } from 'zustand';
import type { TelemetryData } from '../lib/api';

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
  updateFromTelemetry: (data: TelemetryData) => void;
  addHistoricalData: (data: TelemetryData[]) => void;
  appendLiveToHistory: (data: TelemetryData) => void;
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
  
  updateFromTelemetry: (data: TelemetryData) => {
    const MIN_TEMP = 37.0;
    const isHeatOn = data.smokerTarget > MIN_TEMP;
    
    // Add timestamp for chart plotting
    const dataWithTimestamp = {
      ...data,
      timestamp: Date.now(),
    };
    
    // Update current state
    set({
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
    });
    
    // Append to historical data for chart
    get().appendLiveToHistory(dataWithTimestamp);
  },
  
  addHistoricalData: (data: TelemetryData[]) => {
    // Merge and sort by timestamp
    const merged = [...get().historicalData, ...data];
    const sorted = merged.sort((a, b) => {
      const aTime = a.timestamp || 0;
      const bTime = b.timestamp || 0;
      return aTime - bTime;
    });
    // Remove duplicates (keep last occurrence)
    const unique = sorted.filter((item, index, self) =>
      index === self.findIndex((t) => t.timestamp === item.timestamp)
    );
    set({ historicalData: unique });
  },
  
  appendLiveToHistory: (data: TelemetryData) => {
    // Add live data point to historical data for chart
    // Limit history to prevent memory issues (keep last 100 points)
    const history = [...get().historicalData, data];
    const maxHistory = 100;
    const trimmedHistory = history.length > maxHistory 
      ? history.slice(-maxHistory) 
      : history;
    
    set({ historicalData: trimmedHistory });
  },
}));
