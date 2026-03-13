import { useState } from 'react';
import { SmokerDashboard } from './components/SmokerDashboard';
import { isIOS, isStandalone } from './lib/firebase';
import './App.css';

function App() {
  const [showIOSBanner, setShowIOSBanner] = useState(() => {
    // Initialize state based on the check, runs only once
    return isIOS() && !isStandalone();
  });

  return (
    <div className="dark">
      {/* iOS Installation Banner */}
      {showIOSBanner && (
        <div className="bg-orange-600 text-white p-3 text-center text-sm">
          <p>Install this app on your iPhone: tap Share</p>
          <p className="text-orange-200">then "Add to Home Screen"</p>
          <button 
            onClick={() => setShowIOSBanner(false)}
            className="absolute top-2 right-2 text-orange-200 hover:text-white"
          >
            ✕
          </button>
        </div>
      )}
      
      <SmokerDashboard />
    </div>
  );
}

export default App;
