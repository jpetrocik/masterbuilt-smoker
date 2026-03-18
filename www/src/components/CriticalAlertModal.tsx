// src/components/CriticalAlertModal.tsx

export const CriticalAlertModal = ({ 
  title = "MEAT READY!", 
  body = "Temperature threshold reached.", 
  onDismiss 
}: { 
  title?: string; 
  body?: string; 
  onDismiss: () => void;
}) => {
  return (
    <div className="fixed inset-0 z-[9999] flex items-center justify-center bg-black/80 backdrop-blur-sm p-4">
      <div className="relative w-full max-w-md rounded-2xl bg-gradient-to-b from-red-600 to-red-800 p-8 text-center shadow-[0_0_60px_rgba(220,38,38,0.6)] border border-red-400">
        
        <div className="mx-auto mb-6 flex h-24 w-24 items-center justify-center rounded-full bg-red-500/50 shadow-[0_0_30px_rgba(255,255,255,0.3)] animate-pulse">
          <svg className="h-14 w-14 text-white" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
            <path strokeLinecap="round" strokeLinejoin="round" d="M17.657 18.657A8 8 0 016.343 7.343S7 9 9 10c0-2 .5-5 2.986-7C14 5 16.09 5.777 17.656 7.343A7.975 7.975 0 0120 13a7.975 7.975 0 01-2.343 5.657z" />
            <path strokeLinecap="round" strokeLinejoin="round" d="M9.879 16.121A3 3 0 1012.015 11L11 14H9c0 .768.293 1.536.879 2.121z" />
          </svg>
        </div>

        <h2 className="mb-2 text-4xl font-black tracking-tight text-white uppercase">
          {title}
        </h2>
        <p className="mb-8 text-xl font-bold text-red-100 whitespace-pre-line">
          {body}
        </p>

        <div className="flex flex-col gap-3">
          <button 
            onClick={onDismiss}
            className="w-full rounded-xl bg-white px-6 py-4 text-xl font-bold text-red-700 shadow-lg transition-transform hover:scale-105 active:scale-95"
          >
            ACKNOWLEDGE
          </button>
        </div>
      </div>
    </div>
  );
};