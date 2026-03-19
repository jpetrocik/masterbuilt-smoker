import React, { useState, useEffect } from 'react';
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ResponsiveContainer,
} from 'recharts';
import { Wifi, WifiOff, Flame } from 'lucide-react';
import { cn } from '../lib/utils';
import { useSmokerStore } from '../store/useSmokerStore';
import { useUserPreferenceStore } from '../store/useUserPreferenceStore';
import { useTelemetryWebSocket, sendCommand } from '../lib/useTelemetryWebSocket';
import type { Smoker } from '../lib/api';
import { getSmokers } from '../lib/api';
import { TargetTemperatureModal } from './TargetTemperatureModal';
import { TimerSetModal } from './TimerSetModal';
import { LostConnectionModal } from './LostConnectionModal';

export const SmokerDashboard: React.FC = () => {
  const { 
    isOnline, 
    isHeatOn, 
    smokerTemperature, 
    smokerTarget, 
    cookTime, 
    cookTimer, 
    probe1,
    probe2,
    probe3,
    probe4,
    historicalData,
    isConnecting,
  } = useSmokerStore();
  
  const { 
    carouselIndex,
    setCarouselIndex,
    selectedSmokerId,
    setSelectedSmokerId,
  } = useUserPreferenceStore();
  const [smokers, setSmokers] = useState<Smoker[]>([]);
  const [showPicker, setShowPicker] = useState(false);
  const carouselRef = React.useRef<HTMLDivElement>(null);
  const [modalState, setModalState] = useState({
    isOpen: false,
    targetType: '',
    title: '',
    currentTarget: 0,
  });
  const [isTimerModalOpen, setIsTimerModalOpen] = useState(false);

  // --- Modal Handlers ---
  const handleOpenModal = (targetType: string, title: string, currentTarget: number) => {
    setModalState({
      isOpen: true,
      targetType,
      title,
      currentTarget: currentTarget || 0,
    });
  };

  const handleCloseModal = () => {
    setModalState({ isOpen: false, targetType: '', title: '', currentTarget: 0 });
  };

  const handleSetTarget = (newTemperature: number) => {
    const { targetType } = modalState;
    if (!targetType) return;

    let commandKey = '';
    
    // Optimistically update the UI
    if (targetType === 'smoker') {
      commandKey = 'setTemp';
      useSmokerStore.getState().setSmokerTarget(newTemperature);
    } else if (/^probe[1-4]$/.test(targetType)) {
      const probeNumber = parseInt(targetType.replace('probe', ''), 10) as 1 | 2 | 3 | 4;
      commandKey = `setProbe${probeNumber}Target`;
      useSmokerStore.getState().setProbeTarget(probeNumber, newTemperature);
    }
    
    // Dispatch command
    if (commandKey) {
      sendCommand({
        action: 'command',
        commandKey,
        commandValue: newTemperature.toString(),
      });
    }
  };

  const handleSetTimer = (totalSeconds: number) => {
    // Optimistically update the UI
    useSmokerStore.getState().setCookTimer(totalSeconds);

    const totalMinutes = Math.ceil(totalSeconds / 60);

    // Dispatch command
    sendCommand({
      action: 'command',
      commandKey: 'setCookTime',
      commandValue: totalMinutes.toString(),
    });
  };


  // Load smokers on mount
  useEffect(() => {
    const loadSmokers = async () => {
      try {
        const data = await getSmokers('online');
        setSmokers(data);
        
        // Auto-select logic based on persisted selectedSmokerId
        const availableIds = data.map(s => s.id);
        
        if (selectedSmokerId && availableIds.includes(selectedSmokerId)) {
          // Selected smoker is available, no action needed
        } else if (data.length === 1) {
          setSelectedSmokerId(data[0].id);
        } else if (data.length > 1) {
          setShowPicker(true);
        }
      } catch (err) {
        console.error('Failed to load smokers:', err);
      }
    };
    
    loadSmokers();
  }, [setSelectedSmokerId, selectedSmokerId]);

  // Connect to WebSocket (handles messages directly in store)
  useTelemetryWebSocket(selectedSmokerId);

  // Set initial scroll position of the carousel
  useEffect(() => {
    if (carouselRef.current) {
      const slideWidth = carouselRef.current.clientWidth;
      carouselRef.current.scrollLeft = carouselIndex * slideWidth;
    }
  }, [carouselIndex]);

  // Set isOnline status when selectedSmokerId or smokers list changes
  useEffect(() => {
    if (selectedSmokerId && smokers.length > 0) {
      const selectedSmoker = smokers.find(s => s.id === selectedSmokerId);
      if (selectedSmoker) {
        useSmokerStore.getState().setIsOnline(selectedSmoker.status === 'online');
      }
    }
  }, [selectedSmokerId, smokers]);

  // Update carousel index based on scroll position
  const handleScroll = (e: React.UIEvent<HTMLDivElement>) => {
    const target = e.currentTarget;
    const scrollLeft = target.scrollLeft;
    const slideWidth = target.clientWidth;
    const currentIndex = Math.round(scrollLeft / slideWidth);
    
    if (currentIndex !== carouselIndex) {
      setCarouselIndex(currentIndex);
    }
  };

  // Format time for display
  const formatTimeDisplay = (seconds: number): string => {
    if (!seconds || seconds <= 0) return '--:--';
    const hrs = Math.floor(seconds / 3600);
    const mins = Math.ceil((seconds % 3600) / 60);
    return `${hrs.toString().padStart(2, '0')}:${mins.toString().padStart(2, '0')}`;
  };

  // Format temperature for display
  const formatTemp = (temp: number | null | undefined): string => {
    if (temp === null || temp === undefined || temp === 0) return '--';
    return Math.round(temp).toString();
  };

  const isProbePresent = (temp: number | null | undefined): boolean => {
    return !(temp === null || temp === undefined || temp === 0);
  };

  // Prepare chart data from historical and live data
  const chartData = React.useMemo(() => {
    return historicalData.map((item) => ({
      // 1. Keep the raw numerical timestamp. This becomes our true X-axis value.
      timestamp: item.timestamp, 
      
      // 2. Keep a formatted string just for the hover Tooltip to use
      time: item.timestamp ? new Date(item.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }) : '',
      
      // 3. Your temperature mappings must handle undefined values
      probe1: item.probe1Temperature ?? null,
      probe2: item.probe2Temperature ?? null,
      probe3: item.probe3Temperature ?? null,
      probe4: item.probe4Temperature ?? null,
    }));
  }, [historicalData]);

  // Calculate hourly ticks for the X-axis
  const hourlyTicks = React.useMemo(() => {
    if (historicalData.length < 2) {
      return [];
    }

    const getHourlyTicks = (data: typeof historicalData) => {
      const timestamps = data.map(d => d.timestamp).filter(t => t) as number[];
      if (timestamps.length < 2) {
        return [];
      }

      const minTime = Math.min(...timestamps);
      const maxTime = Math.max(...timestamps);

      const start = new Date(minTime);
      start.setMinutes(0, 0, 0);
      start.setHours(start.getHours() + 1); 

      const end = new Date(maxTime);
      end.setMinutes(0, 0, 0);

      const ticks = [];
      let current = start.getTime();

      while (current <= end.getTime()) {
        ticks.push(current);
        current += 3600000; // Add one hour
      }
      return ticks;
    };

    return getHourlyTicks(historicalData);
  }, [historicalData]);

  const formatXAxisTick = (unixTime: number): string => {
    const date = new Date(unixTime);
    let hours = date.getHours();
    const ampm = hours >= 12 ? 'pm' : 'am';
    hours %= 12;
    hours = hours || 12; // the hour '0' should be '12'
    return `${hours.toString().padStart(2, '0')}${ampm}`;
  };

  return (
    <div className="min-h-screen bg-zinc-950 text-white p-4 md:p-6 pb-20 safe-area-inset-bottom">
      {isConnecting && <LostConnectionModal />}
      <TargetTemperatureModal
        key={modalState.targetType}
        isOpen={modalState.isOpen}
        onClose={handleCloseModal}
        onSave={handleSetTarget}
        title={modalState.title}
        currentTarget={modalState.currentTarget}
      />
      <TimerSetModal
        isOpen={isTimerModalOpen}
        onClose={() => setIsTimerModalOpen(false)}
        onSave={handleSetTimer}
        title="Set Cook Timer"
        currentValueInSeconds={cookTimer}
      />
      {/* Smoker Picker Dialog */}
      {showPicker && (
        <div className="fixed inset-0 bg-black/60 flex items-center justify-center z-50">
          <div className="bg-zinc-900 rounded-xl p-6 max-w-sm w-full mx-4 border border-zinc-800">
            <h2 className="text-xl font-bold text-orange-500 mb-4">Select Smoker</h2>
            <div className="space-y-2">
              {smokers.map((smoker) => (
                <button
                  key={smoker.id}
                  onClick={() => {
                    setSelectedSmokerId(smoker.id);
                    setShowPicker(false);
                  }}
                  className="w-full text-left p-3 rounded-lg bg-zinc-800 hover:bg-zinc-700 transition-colors"
                >
                  <div className="font-medium">{smoker.id}</div>
                  <div className="text-sm text-zinc-400">
                    Last seen: {new Date(smoker.lastSeen).toLocaleString()}
                  </div>
                </button>
              ))}
            </div>
          </div>
        </div>
      )}

      {/* Header */}
      <header className="mb-6 flex items-center justify-between">
        <button 
          onClick={() => setShowPicker(true)}
          className="text-2xl font-bold text-orange-500 text-left bg-transparent disabled:cursor-not-allowed hover:opacity-80 transition-opacity"
        >
          Smoker {selectedSmokerId ? `#${selectedSmokerId}` : ''}
        </button>
        <div className="flex items-center">
          {isOnline ? (
            <Wifi className="text-orange-500" size={24} />
          ) : (
            <WifiOff className="text-zinc-400" size={24} />
          )}
        </div>
      </header>

      {/* Temperature Grid */}
      <div className="grid grid-cols-2 gap-4 mb-8">
        {/* Hero Card: Carousel */}
        <div className="col-span-2 bg-zinc-900 border border-zinc-800 rounded-xl p-6 relative overflow-hidden">
          {/* Carousel Header */}
          <div className="absolute top-4 left-4 text-zinc-300 text-sm font-semibold z-10">
            {carouselIndex === 0 ? "Smoker Temp" : carouselIndex === 1 ? "Cook Timer" : "Elapsed Time"}
          </div>

           {/* Swipeable Container */}
           <div 
            ref={carouselRef}
            className="mt-4 flex overflow-x-auto snap-x snap-mandatory snap-start h-36 items-center scrollbar-hide"
            onScroll={handleScroll}
          >
            {/* Slide 1: Temp */}
            <div className="snap-start shrink-0 w-full flex flex-col items-center justify-center">
              <button onClick={() => handleOpenModal('smoker', 'Smoker Temperature', smokerTarget || 150)}>
                <div className="flex items-center justify-center gap-2">
                  <Flame 
                    className={cn(
                      "w-12 h-12",
                      isHeatOn ? "text-orange-500" : "text-zinc-400"
                    )} 
                  />
                  <div className={cn(
                    "text-7xl font-bold tracking-tighter",
                    isHeatOn ? "text-orange-500 [text-shadow:0_0_20px_rgba(249,115,22,0.6)]" : "text-zinc-400"
                  )}>
                    {formatTemp(smokerTemperature)}°
                  </div>
                </div>
                <div className="text-zinc-500 text-xl mt-1">
                  {formatTemp(smokerTarget)}°
                </div>
              </button>
            </div>

            {/* Slide 2: Timer */}
            <div className="snap-start shrink-0 w-full flex flex-col items-center justify-center">
              <button onClick={() => setIsTimerModalOpen(true)}>
                <div className="flex items-center justify-center gap-2">
                  <Flame 
                    className={cn(
                      "w-12 h-12",
                      isHeatOn ? "text-orange-500" : "text-zinc-400"
                    )} 
                  />
                  <div className={cn(
                    "text-7xl font-bold tracking-widest font-mono",
                    isHeatOn ? "text-orange-500" : "text-zinc-400"
                  )}>
                    {formatTimeDisplay(cookTimer)}
                  </div>
                </div>
                <div className="text-zinc-500 text-xl mt-1">
                  Remaining
                </div>
              </button>
            </div>

            {/* Slide 3: Elapsed Time */}
            <div className="snap-start shrink-0 w-full flex flex-col items-center justify-center">
              <div className="flex items-center justify-center gap-2">
                <Flame 
                  className={cn(
                    "w-12 h-12",
                    isHeatOn ? "text-orange-500" : "text-zinc-400"
                  )} 
                />
                <div className={cn(
                  "text-7xl font-bold tracking-widest font-mono",
                  isHeatOn ? "text-orange-500" : "text-zinc-400"
                )}>
                  {formatTimeDisplay(cookTime)}
                </div>
              </div>
              <div className="text-zinc-500 text-xl mt-1">
                Elapsed
              </div>
            </div>
          </div>

          {/* Carousel Indicators */}
          <div className="absolute bottom-4 left-0 right-0 flex justify-center gap-2">
            <div className={cn("w-2 h-2 rounded-full", carouselIndex === 0 ? "bg-orange-500" : "bg-zinc-600")} />
            <div className={cn("w-2 h-2 rounded-full", carouselIndex === 1 ? "bg-orange-500" : "bg-zinc-600")} />
            <div className={cn("w-2 h-2 rounded-full", carouselIndex === 2 ? "bg-orange-500" : "bg-zinc-600")} />
          </div>
        </div>

        {/* Probe Cards */}
        <button onClick={() => handleOpenModal('probe1', 'Probe 1 Alarm', probe1.target || 150)} className="bg-zinc-900 border border-zinc-800 rounded-xl p-4 flex flex-col items-center justify-center text-left w-full">
          <div className="text-zinc-300 text-sm font-semibold mb-2 self-start">
            Probe 1
          </div>
          <div className={cn("text-5xl font-bold", isProbePresent(probe1.temperature) ? "text-orange-500 [text-shadow:0_0_20px_rgba(249,115,22,0.6)]" : "text-zinc-500")}>
            {formatTemp(probe1.temperature)}°
          </div>
          <div className="text-zinc-500 text-lg mt-1">
            {formatTemp(probe1.target)}°
          </div>
        </button>
        <button onClick={() => handleOpenModal('probe2', 'Probe 2 Alarm', probe2.target || 150)} className="bg-zinc-900 border border-zinc-800 rounded-xl p-4 flex flex-col items-center justify-center text-left w-full">
          <div className="text-zinc-300 text-sm font-semibold mb-2 self-start">
            Probe 2
          </div>
          <div className={cn("text-5xl font-bold", isProbePresent(probe2.temperature) ? "text-orange-500 [text-shadow:0_0_20px_rgba(249,115,22,0.6)]" : "text-zinc-500")}>
            {formatTemp(probe2.temperature)}°
          </div>
          <div className="text-zinc-500 text-lg mt-1">
            {formatTemp(probe2.target)}°
          </div>
        </button>
        <button onClick={() => handleOpenModal('probe3', 'Probe 3 Alarm', probe3.target || 150)} className="bg-zinc-900 border border-zinc-800 rounded-xl p-4 flex flex-col items-center justify-center text-left w-full">
          <div className="text-zinc-300 text-sm font-semibold mb-2 self-start">
            Probe 3
          </div>
          <div className={cn("text-5xl font-bold", isProbePresent(probe3.temperature) ? "text-orange-500 [text-shadow:0_0_20px_rgba(249,115,22,0.6)]" : "text-zinc-500")}>
            {formatTemp(probe3.temperature)}°
          </div>
          <div className="text-zinc-500 text-lg mt-1">
            {formatTemp(probe3.target)}°
          </div>
        </button>
        <button onClick={() => handleOpenModal('probe4', 'Probe 4 Alarm', probe4.target || 150)} className="bg-zinc-900 border border-zinc-800 rounded-xl p-4 flex flex-col items-center justify-center text-left w-full">
          <div className="text-zinc-300 text-sm font-semibold mb-2 self-start">
            Probe 4
          </div>
          <div className={cn("text-5xl font-bold", isProbePresent(probe4.temperature) ? "text-orange-500 [text-shadow:0_0_20px_rgba(249,115,22,0.6)]" : "text-zinc-500")}>
            {formatTemp(probe4.temperature)}°
          </div>
          <div className="text-zinc-500 text-lg mt-1">
            {formatTemp(probe4.target)}°
          </div>
        </button>
      </div>

      {/* Historical Chart Section */}
      <div className="bg-zinc-900 border border-zinc-800 rounded-xl p-4 h-80 flex flex-col">
        <div className="text-zinc-300 font-semibold mb-4 text-sm tracking-wider">
          Probe Temperature History
        </div>
        <div className="flex-1 w-full min-h-0">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart
              data={chartData}
              margin={{ top: 5, right: 20, left: -10, bottom: 5 }}
            >
              <CartesianGrid strokeDasharray="3 3" stroke="#3f3f46" />
                <XAxis 
                  dataKey="timestamp"
                  type="number"
                  scale="time"
                  domain={['dataMin', 'dataMax']}
                  stroke="#71717a" 
                  fontSize={12} 
                  tickLine={false}
                  tickFormatter={formatXAxisTick}
                  ticks={hourlyTicks}
                  label={{ value: 'Hour', position: 'insideBottom', offset: -2 }}
                />
              <YAxis 
                stroke="#71717a" 
                fontSize={12} 
                tickLine={false}
                allowDecimals={false}
                tickFormatter={(value) => `${value}°`}
                domain={[50, 350]}
              />              
              <Tooltip
                contentStyle={{
                  backgroundColor: '#18181b',
                  borderColor: '#27272a',
                  color: '#fff',
                }}
                formatter={(value, name) => [`${Math.round(Number(value))}°F`, name]}
                labelFormatter={(label) => {
                  return new Date(label).toLocaleTimeString([], { 
                    hour: '2-digit', 
                    minute: '2-digit' 
                  });
                }}
              />
              <Line 
                type="monotone" 
                dataKey="probe1" 
                stroke="#fdba74" 
                strokeWidth={1.5} 
                dot={false}
                name="Probe 1"
              />
              <Line 
                type="monotone" 
                dataKey="probe2" 
                stroke="#a1a1aa" 
                strokeWidth={1.5} 
                dot={false}
                name="Probe 2"
              />
              <Line 
                type="monotone" 
                dataKey="probe3" 
                stroke="#fb923c" 
                strokeWidth={1.5} 
                dot={false}
                name="Probe 3"
              />
              <Line 
                type="monotone" 
                dataKey="probe4" 
                stroke="#78716c" 
                strokeWidth={1.5} 
                dot={false}
                name="Probe 4"
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
      </div>
    </div>
  );
};
