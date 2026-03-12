import { create } from 'zustand';

interface ProbeState {
  temperature: number;
  target: number;
  alarm: boolean;
}

interface SmokerState {
  isOnline: boolean;
  isHeatOn: boolean;
  smokerTemperature: number;
  smokerTarget: number;
  cookTime: string;
  cookTimer: string;
  dutyCycle: number;
  probe1: ProbeState;
  probe2: ProbeState;
  probe3: ProbeState;
  probe4: ProbeState;
  toggleOnline: () => void;
  toggleHeat: () => void;
}

export const useSmokerStore = create<SmokerState>((set) => ({
  isOnline: true,
  isHeatOn: false,
  smokerTemperature: 224,
  smokerTarget: 225,
  cookTime: "06:35",
  cookTimer: "04:34",
  dutyCycle: 0.75,
  probe1: { temperature: 145, target: 165, alarm: false },
  probe2: { temperature: 151, target: 160, alarm: false },
  probe3: { temperature: 148, target: 155, alarm: false },
  probe4: { temperature: 153, target: 170, alarm: false },
  toggleOnline: () => set((state) => ({ isOnline: !state.isOnline })),
  toggleHeat: () => set((state) => ({ isHeatOn: !state.isHeatOn })),
}));
