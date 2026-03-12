import { create } from 'zustand';
import type { TelemetryData } from '../lib/api';

interface ProbeState {
  temperature: number | null;
  target: number;
  alarm: boolean;
}

interface SmokerState {
  selectedSmokerId: string | null;
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
  setSelectedSmokerId: (id: string | null) => void;
  updateFromTelemetry: (data: TelemetryData) => void;
  addHistoricalData: (data: TelemetryData[]) => void;
}

export const useSmokerStore = create<SmokerState>((set, get) => ({
  selectedSmokerId: null,
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
  
  setSelectedSmokerId: (id: string | null) => set({ selectedSmokerId: id }),
  
  updateFromTelemetry: (data: TelemetryData) => {
    const MIN_TEMP = 37.0;
    const isHeatOn = data.smokerTarget > MIN_TEMP;
    
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
  },
  
  addHistoricalData: (data: TelemetryData[]) => {
    set({ historicalData: [...get().historicalData, ...data] });
  },
}));
