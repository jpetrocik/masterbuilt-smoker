import { useState, useRef, useMemo } from 'react';
import { useLongPress } from '../lib/useLongPress';

interface TimerSetModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSave: (totalSeconds: number) => void;
  title: string;
  currentValueInSeconds: number;
}

// Helper to format total seconds into HH:MM
const formatTime = (totalSeconds: number) => {
  const hours = Math.floor(totalSeconds / 3600);
  const minutes = Math.floor((totalSeconds % 3600) / 60);
  return {
    hours: hours.toString().padStart(2, '0'),
    minutes: minutes.toString().padStart(2, '0'),
  };
};

export const TimerSetModal = ({
  isOpen,
  onClose,
  onSave,
  title,
  currentValueInSeconds
}: TimerSetModalProps) => {
  const [totalSeconds, setTotalSeconds] = useState(currentValueInSeconds);
  const decreaseRef = useRef<HTMLButtonElement>(null);
  const increaseRef = useRef<HTMLButtonElement>(null);

  const decreaseHandlers = useLongPress(decreaseRef, {
    onClick: () => setTotalSeconds(prev => Math.max(0, prev - 60)), // -1 min
    onLongPress: () => setTotalSeconds(prev => Math.max(0, prev - 900)), // -5 mins
  });

  const increaseHandlers = useLongPress(increaseRef, {
    onClick: () => setTotalSeconds(prev => prev + 60), // +1 min
    onLongPress: () => setTotalSeconds(prev => prev + 900), // +5 mins
  });

  const { hours, minutes } = useMemo(() => formatTime(totalSeconds), [totalSeconds]);

  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/70 backdrop-blur-sm p-4">
      <div className="relative w-full max-w-sm rounded-2xl bg-gray-800 p-6 shadow-2xl border border-gray-700">
        
        <h3 className="text-2xl font-bold text-white text-center mb-6">
          {title}
        </h3>

        <div className="flex items-center justify-center gap-4 mb-8">
          <button 
            ref={decreaseRef}
            {...decreaseHandlers}
            className="h-16 w-16 rounded-full bg-gray-700 text-3xl font-bold text-white active:bg-gray-600 transition-colors select-none"
          >
            -
          </button>

          <div className="flex w-48 items-center justify-center rounded-xl border-2 border-gray-700 bg-gray-900 px-4 py-4">
            <span className="text-5xl font-black text-orange-500 font-mono">{hours}</span>
            {/* Added mx-1 here to give the colon a little breathing room */}
            <span className="mx-1 text-5xl font-black text-orange-500 font-mono animate-pulse">:</span>
            <span className="text-5xl font-black text-orange-500 font-mono">{minutes}</span>
          </div>

          <button 
            ref={increaseRef}
            {...increaseHandlers}
            className="h-16 w-16 rounded-full bg-gray-700 text-3xl font-bold text-white active:bg-gray-600 transition-colors select-none"
          >
            +
          </button>
        </div>

        <div className="flex gap-3">
          <button 
            onClick={onClose}
            className="flex-1 rounded-xl bg-gray-700 py-3 text-lg font-bold text-white active:bg-gray-600"
          >
            Cancel
          </button>
          <button 
            onClick={() => {
              onSave(totalSeconds);
              onClose();
            }}
            className="flex-1 rounded-xl bg-orange-600 py-3 text-lg font-bold text-white active:bg-orange-700"
          >
            Ok
          </button>
        </div>
      </div>
    </div>
  );
};
