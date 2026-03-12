import { useState, useEffect } from 'react';
import { SmokerDashboard } from './components/SmokerDashboard';
import { isIOS, isStandalone, getFCMToken } from './lib/firebase';
import { registerFcmToken } from './lib/api';
import './App.css';

function App() {
  const [showIOSBanner, setShowIOSBanner] = useState(false);
  const [smokerId, setSmokerId] = useState<string | null>(null);

  useEffect(() => {
    // Check if iOS and not in standalone mode
    if (isIOS() && !isStandalone()) {
      setShowIOSBanner(true);
    }

    // Get last selected smoker for FCM registration
    const lastSelected = localStorage.getItem('lastSelectedSmoker');
    if (lastSelected) {
      setSmokerId(lastSelected);
    }
  }, []);

  // Register FCM token when smoker is selected
  useEffect(() => {
    const registerToken = async () => {
      if (smokerId) {
        try {
          const token = await getFCMToken();
          if (token) {
            await registerFcmToken(smokerId, token);
            console.log('FCM token registered successfully');
          }
        } catch (err) {
          console.error('Failed to register FCM token:', err);
        }
      }
    };

    registerToken();
  }, [smokerId]);

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
