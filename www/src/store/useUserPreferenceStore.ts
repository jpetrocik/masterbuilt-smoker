import { create } from 'zustand';
import { persist } from 'zustand/middleware';

interface UserPreferenceState {
  carouselIndex: number;
  setCarouselIndex: (index: number) => void;
}

export const useUserPreferenceStore = create<UserPreferenceState>()(
  persist(
    (set) => ({
      carouselIndex: 0,
      setCarouselIndex: (index: number) => set({ carouselIndex: index }),
    }),
    {
      name: 'user-preferences',
    }
  )
);
