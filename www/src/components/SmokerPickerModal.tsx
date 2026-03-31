import type { Smoker } from '../lib/api';

interface SmokerPickerModalProps {
  isOpen: boolean;
  smokers: Smoker[];
  onClose: () => void;
  onSelect: (smokerId: string) => void;
}

export const SmokerPickerModal = ({ isOpen, smokers, onClose, onSelect }: SmokerPickerModalProps) => {
  if (!isOpen) {
    return null;
  }

  return (
    <div className="fixed inset-0 bg-black/60 flex items-center justify-center z-50">
      <div className="bg-zinc-900 rounded-xl p-6 max-w-sm w-full mx-4 border border-zinc-800">
        <h2 className="text-xl font-bold text-orange-500 mb-4">Select Smoker</h2>
        <div className="space-y-2">
          {smokers.map((smoker) => (
            <button
              key={smoker.id}
              onClick={() => {
                onSelect(smoker.id);
                onClose();
              }}
              className="w-full text-left p-3 rounded-lg bg-zinc-800 hover:bg-zinc-700 transition-colors"
            >
              <div className="font-medium">{smoker.id}</div>
              <div className="text-sm text-zinc-400">
                {smoker.status === 'online' ? 'Online' : 'Offline'}
              </div>
            </button>
          ))}
        </div>
        <button
          onClick={onClose}
          className="w-full mt-4 text-center p-3 rounded-lg bg-gray-700 hover:bg-gray-600 transition-colors"
        >
          Cancel
        </button>
      </div>
    </div>
  );
};
