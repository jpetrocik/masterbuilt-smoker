import { useState, useEffect } from 'react';
import { SmokerDashboard } from './components/SmokerDashboard';
import { isIOS, isStandalone, getFCMToken, initFirebase } from './lib/firebase';
import { registerFcmToken } from './lib/api';
import { useUserPreferenceStore } from './store/useUserPreferenceStore';
import './App.css';
import { onMessage } from 'firebase/messaging';
import { CriticalAlertModal } from './components/CriticalAlertModal';

function App() {
  const [criticalAlert, setCriticalAlert] = useState<{ title: string; body: string } | null>(null);
  const [showIOSBanner, setShowIOSBanner] = useState(() => {
    // Initialize state based on the check, runs only once
    return isIOS() && !isStandalone();
  });
  const selectedSmokerId = useUserPreferenceStore((state) => state.selectedSmokerId);

  // Request notification permissions and register FCM token
  useEffect(() => {
    const registerToken = async () => {
      if (selectedSmokerId) {
        try {
          const token = await getFCMToken();
          if (token) {
            await registerFcmToken(selectedSmokerId, token);
            console.log('FCM token registered successfully');
          }
        } catch (err) {
          console.error('Failed to register FCM token:', err);
        }
      }
    };

    registerToken();
  }, [selectedSmokerId]);

  // Handle foreground messages
  useEffect(() => {
    // Ensure Firebase is initialized before setting up the listener
    initFirebase().then(messaging => {
      if (messaging) {
        const unsubscribe = onMessage(messaging, (payload) => {
          console.log('Foreground message received.', payload);
          const title = payload.data?.title || 'Alert';
          const body = payload.data?.body || '';
          
          setCriticalAlert({ title, body });
        });

        return () => {
          unsubscribe();
        };
      }
    });
  }, []);

  // Handle background messages via BroadcastChannel
  useEffect(() => {
    const channel = new BroadcastChannel('fcm-alerts');

    const handleMessage = (event: MessageEvent) => {
      console.log('Broadcast message received.', event.data);
      const title = event.data?.title || 'Alert';
      const body = event.data?.body || '';
      
      setCriticalAlert({ title, body });
    };

    channel.addEventListener('message', handleMessage);

    return () => {
      channel.removeEventListener('message', handleMessage);
      channel.close();
    };
  }, []);

  return (
    <div className="dark">
      {criticalAlert && (
        <CriticalAlertModal
          title={criticalAlert.title}
          body={criticalAlert.body}
          onDismiss={() => setCriticalAlert(null)}
        />
      )}

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
