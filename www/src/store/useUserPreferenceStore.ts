import { create } from 'zustand';
import { persist } from 'zustand/middleware';

interface UserPreferenceState {
  carouselIndex: number;
  selectedSmokerId: string | null;
  setCarouselIndex: (index: number) => void;
  setSelectedSmokerId: (id: string | null) => void;
}

export const useUserPreferenceStore = create<UserPreferenceState>()(
  persist(
    (set) => ({
      carouselIndex: 0,
      selectedSmokerId: null,
      setCarouselIndex: (index: number) => set({ carouselIndex: index }),
      setSelectedSmokerId: (id: string | null) => set({ selectedSmokerId: id }),
    }),
    {
      name: 'user-preferences',
    }
  )
);
