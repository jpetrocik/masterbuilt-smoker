import { create } from 'zustand';

interface SmokerState {
  isOnline: boolean;
  isHeatOn: boolean;
  toggleOnline: () => void;
  toggleHeat: () => void;
}

export const useSmokerStore = create<SmokerState>((set) => ({
  isOnline: true,
  isHeatOn: false,
  toggleOnline: () => set((state) => ({ isOnline: !state.isOnline })),
  toggleHeat: () => set((state) => ({ isHeatOn: !state.isHeatOn })),
}));
