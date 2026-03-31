import { useState, useRef } from 'react';
import { useLongPress } from '../lib/useLongPress';

interface TargetTemperatureModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSave: (newTarget: number) => void;
  title: string;
  currentTarget: number;
}

export const TargetTemperatureModal = ({
  isOpen,
  onClose,
  onSave,
  title,
  currentTarget
}: TargetTemperatureModalProps) => {
  const [tempVal, setTempState] = useState(currentTarget);
  const decreaseRef = useRef<HTMLButtonElement>(null);
  const increaseRef = useRef<HTMLButtonElement>(null);

  const setTemp = (newTemp: number) => {
    setTempState(Math.max(0, Math.min(300, newTemp)));
  }

  const decreaseHandlers = useLongPress(decreaseRef, {
    onClick: () => setTemp(tempVal - 1),
    onLongPress: () => setTemp(tempVal - 5),
  });

  const increaseHandlers = useLongPress(increaseRef, {
    onClick: () => setTemp(tempVal + 1),
    onLongPress: () => setTemp(tempVal + 5),
  });

  if (!isOpen) return null;


  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/60 backdrop-blur-sm p-4">
      <div className="relative w-full max-w-sm rounded-2xl bg-gray-800 p-6 shadow-2xl border border-gray-700">
        
        <h3 className="text-2xl font-bold text-white text-center mb-6">
          {title}
        </h3>

        <div className="flex items-center justify-center gap-4 mb-8">
          {/* Quick Decrease */}
          <button 
            ref={decreaseRef}
            {...decreaseHandlers}
            className="h-16 w-16 rounded-full bg-gray-700 text-2xl font-bold text-white active:bg-gray-600 transition-colors select-none"
          >
            -
          </button>

          {/* MAIN INPUT */}
          <div className="flex w-40 items-center rounded-xl border-2 border-gray-700 bg-gray-900 pr-4 focus-within:border-orange-500">
            <input 
              type="number" 
              inputMode="numeric"
              value={tempVal}
              onChange={(e) => setTemp(Number(e.target.value))}
              // The crazy bracket classes at the end hide the default browser spinner arrows
              className="w-full bg-transparent py-4 text-right text-5xl font-black text-orange-500 focus:outline-none [appearance:textfield] [&::-webkit-inner-spin-button]:appearance-none [&::-webkit-outer-spin-button]:appearance-none"
            />
            <span className="ml-2 text-2xl font-bold text-gray-400">
              °F
            </span>
          </div>

          {/* Quick Increase */}
          <button 
            ref={increaseRef}
            {...increaseHandlers}
            className="h-16 w-16 rounded-full bg-gray-700 text-2xl font-bold text-white active:bg-gray-600 transition-colors select-none"
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
              onSave(tempVal);
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