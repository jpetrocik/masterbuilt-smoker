
export const LostConnectionModal = () => {
  return (
    <div className="fixed inset-0 z-[100] flex items-center justify-center bg-black/60 backdrop-blur-md p-4">
      <div className="flex flex-col items-center justify-center w-full max-w-sm rounded-2xl bg-gray-800 p-8 shadow-2xl border border-orange-500/30">
        
        {/* ANIMATED ICON CONTAINER */}
        <div className="relative flex items-center justify-center w-24 h-24 mb-6">
          
          {/* 1. Outer Rotating Ring (Spins continuously) */}
          <svg 
            className="absolute inset-0 w-full h-full text-orange-600 animate-spin" 
            viewBox="0 0 24 24" 
            fill="none" 
            stroke="currentColor" 
            strokeWidth="1.5" 
            strokeLinecap="round" 
            strokeLinejoin="round"
            // Slow down the standard Tailwind spin slightly for a calmer effect
            style={{ animationDuration: '2s' }}
          >
            {/* Top arc with arrow */}
            <path d="M 3 12 a 9 9 0 0 1 9 -9 a 9.75 9.75 0 0 1 6.74 2.74 L 21 8" />
            <path d="M 21 3 v 5 h -5" />
            {/* Bottom arc with arrow */}
            <path d="M 21 12 a 9 9 0 0 1 -9 9 a 9.75 9.75 0 0 1 -6.74 -2.74 L 3 16" />
            <path d="M 3 21 v -5 h 5" />
          </svg>

          {/* 2. Inner Pulsing Signal (Wifi waves - Mathematically centered) */}
          <svg 
            className="absolute w-10 h-10 text-orange-400" 
            viewBox="0 0 24 24" 
            fill="none" 
            stroke="currentColor" 
            strokeWidth="2.5" 
            strokeLinecap="round" 
            strokeLinejoin="round"
          >
            {/* Inner Dot (Always visible, shifted up from 20 to 16) */}
            <path d="M12 16h.01" />
            
            {/* Middle Wave (Pulses first, shifted up from 16.5 to 12.5) */}
            <path 
              d="M8.5 12.5a5 5 0 0 1 7 0" 
              className="animate-pulse" 
              style={{ animationDuration: '1.5s', animationDelay: '0ms' }} 
            />
            
            {/* Outer Wave (Pulses second, shifted up from 13 to 9) */}
            <path 
              d="M5 9a10 10 0 0 1 14 0" 
              className="animate-pulse" 
              style={{ animationDuration: '1.5s', animationDelay: '300ms' }} 
            />
          </svg>

        </div>

        {/* TEXT CONTENT */}
        <h3 className="text-xl font-black text-orange-500 tracking-widest animate-pulse mb-3" style={{ animationDuration: '2s' }}>
          RECONNECTING...
        </h3>
        <p className="text-gray-400 text-center text-sm font-medium leading-relaxed">
          Connection Lost.<br/>
          Automatically attempting to restore.<br/>
          Please wait.
        </p>

      </div>
    </div>
  );
};