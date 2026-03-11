import { create } from 'zustand';

interface ProbeState {
  current: number;
  target: number;
}

interface SmokerState {
  isOnline: boolean;
  isHeatOn: boolean;
  smokerTemp: number;
  smokerTarget: number;
  elapsedCookTime: string;
  cookTimer: string;
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
  smokerTemp: 224,
  smokerTarget: 225,
  elapsedCookTime: "06:35",
  cookTimer: "04:34",
  probe1: { current: 145, target: 165 },
  probe2: { current: 151, target: 160 },
  probe3: { current: 148, target: 155 },
  probe4: { current: 153, target: 170 },
  toggleOnline: () => set((state) => ({ isOnline: !state.isOnline })),
  toggleHeat: () => set((state) => ({ isHeatOn: !state.isHeatOn })),
}));
