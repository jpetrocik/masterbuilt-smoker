import React, { useLayoutEffect } from 'react';
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

// Mock Data Generation
const generateMockData = () => {
  const data = [];
  for (let i = 0; i <= 12; i++) {
    data.push({
      hour: `${i}h`,
      probe1: Math.round(140 + Math.random() * 20), // 140-160
      probe2: Math.round(150 + Math.random() * 15), // 150-165
      probe3: Math.round(145 + Math.random() * 10),
      probe4: Math.round(155 + Math.random() * 12),
    });
  }
  return data;
};

const mockData = generateMockData();

export const SmokerDashboard: React.FC = () => {
  const { 
    isOnline, 
    isHeatOn, 
    smokerTemperature, 
    smokerTarget, 
    cookTime, 
    cookTimer, 
    dutyCycle,
    probe1,
    probe2,
    probe3,
    probe4
  } = useSmokerStore();
  const { carouselIndex, setCarouselIndex } = useUserPreferenceStore();
  const [touchStart, setTouchStart] = React.useState<number | null>(null);
  const [touchEnd, setTouchEnd] = React.useState<number | null>(null);
  const carouselRef = React.useRef<HTMLDivElement>(null);

  const minSwipeDistance = 50;

  // Programmatic scroll when carouselIndex changes
  useLayoutEffect(() => {
    if (carouselRef.current) {
      const scrollAmount = carouselRef.current.clientWidth * carouselIndex;
      carouselRef.current.scrollTo({ left: scrollAmount, behavior: 'smooth' });
    }
  }, [carouselIndex]);

  const onTouchStart = (e: React.TouchEvent) => {
    setTouchEnd(null);
    setTouchStart(e.targetTouches[0].clientX);
  };

  const onTouchMove = (e: React.TouchEvent) => setTouchEnd(e.targetTouches[0].clientX);

  const onTouchEnd = () => {
    if (!touchStart || !touchEnd) return;
    const distance = touchStart - touchEnd;
    const isLeftSwipe = distance > minSwipeDistance;
    const isRightSwipe = distance < -minSwipeDistance;

    if (isLeftSwipe && carouselIndex < 2) {
      setCarouselIndex(carouselIndex + 1);
    } else if (isRightSwipe && carouselIndex > 0) {
      setCarouselIndex(carouselIndex - 1);
    }
  };

  return (
    <div className="min-h-screen bg-zinc-950 text-white p-4 md:p-6 pb-[env(safe-area-inset-bottom)]">
      {/* Header */}
      <header className="mb-6 flex items-center justify-between">
        <h1 className="text-2xl font-bold text-orange-500">Smoker Controller</h1>
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
            onTouchStart={onTouchStart}
            onTouchMove={onTouchMove}
            onTouchEnd={onTouchEnd}
          >
            {/* Slide 1: Temp */}
            <div className="snap-start shrink-0 w-full flex flex-col items-center justify-center">
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
                  {smokerTemperature}°
                </div>
              </div>
              <div className="text-zinc-500 text-xl mt-1">
                {smokerTarget}°
              </div>
            </div>

            {/* Slide 2: Timer */}
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
                  {cookTimer}
                </div>
              </div>
              <div className="text-zinc-500 text-xl mt-1">
                Remaining
              </div>
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
                  {cookTime}
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
        <div
          className="bg-zinc-900 border border-zinc-800 rounded-xl p-4 flex flex-col items-center justify-center"
        >
          <div className="text-zinc-300 text-sm font-semibold mb-2 self-start">
            Probe 1
          </div>
          <div className="text-5xl font-bold text-orange-500 [text-shadow:0_0_20px_rgba(249,115,22,0.6)]">
            {probe1.temperature}°
          </div>
          <div className="text-zinc-500 text-lg mt-1">
            {probe1.target}°
          </div>
        </div>
        <div
          className="bg-zinc-900 border border-zinc-800 rounded-xl p-4 flex flex-col items-center justify-center"
        >
          <div className="text-zinc-300 text-sm font-semibold mb-2 self-start">
            Probe 2
          </div>
          <div className="text-5xl font-bold text-orange-500 [text-shadow:0_0_20px_rgba(249,115,22,0.6)]">
            {probe2.temperature}°
          </div>
          <div className="text-zinc-500 text-lg mt-1">
            {probe2.target}°
          </div>
        </div>
        <div
          className="bg-zinc-900 border border-zinc-800 rounded-xl p-4 flex flex-col items-center justify-center"
        >
          <div className="text-zinc-300 text-sm font-semibold mb-2 self-start">
            Probe 3
          </div>
          <div className="text-5xl font-bold text-orange-500 [text-shadow:0_0_20px_rgba(249,115,22,0.6)]">
            {probe3.temperature}°
          </div>
          <div className="text-zinc-500 text-lg mt-1">
            {probe3.target}°
          </div>
        </div>
        <div
          className="bg-zinc-900 border border-zinc-800 rounded-xl p-4 flex flex-col items-center justify-center"
        >
          <div className="text-zinc-300 text-sm font-semibold mb-2 self-start">
            Probe 4
          </div>
          <div className="text-5xl font-bold text-orange-500 [text-shadow:0_0_20px_rgba(249,115,22,0.6)]">
            {probe4.temperature}°
          </div>
          <div className="text-zinc-500 text-lg mt-1">
            {probe4.target}°
          </div>
        </div>
      </div>

      {/* Historical Chart Section */}
      <div className="mt-8 bg-zinc-900 border border-zinc-800 rounded-xl p-4 h-80 flex flex-col">
        <div className="text-zinc-300 font-semibold mb-4 text-sm tracking-wider">
          Probe Temperature History
        </div>
        <div className="flex-1 w-full min-h-0">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart
              data={mockData}
              margin={{ top: 5, right: 20, left: -10, bottom: 5 }}
            >
              <CartesianGrid strokeDasharray="3 3" stroke="#3f3f46" />
              <XAxis
                dataKey="hour"
                stroke="#71717a"
                fontSize={12}
                tickLine={false}
              />
              <YAxis
                stroke="#71717a"
                fontSize={12}
                tickLine={false}
                domain={['dataMin - 5', 'dataMax + 5']}
              />
              <Tooltip
                contentStyle={{
                  backgroundColor: '#18181b',
                  borderColor: '#27272a',
                  color: '#fff',
                }}
              />
              {/* Probe Lines */}
              <Line
                type="monotone"
                dataKey="probe1"
                stroke="#fdba74" // Orange-300
                strokeWidth={1.5}
                dot={false}
                name="Probe 1"
              />
              <Line
                type="monotone"
                dataKey="probe2"
                stroke="#a1a1aa" // Zinc-400
                strokeWidth={1.5}
                dot={false}
                name="Probe 2"
              />
              <Line
                type="monotone"
                dataKey="probe3"
                stroke="#fb923c" // Orange-400
                strokeWidth={1.5}
                dot={false}
                name="Probe 3"
              />
              <Line
                type="monotone"
                dataKey="probe4"
                stroke="#78716c" // Stone-500
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
