import { useRef, useCallback, useEffect, type RefObject } from 'react';

interface LongPressOptions {
  onClick: () => void;
  onLongPress: () => void;
  delay?: number;
  repeatInterval?: number;
}

export const useLongPress = <T extends HTMLElement | null>(
  ref: RefObject<T>,
  {
    onClick,
    onLongPress,
    delay = 500,
    repeatInterval = 250,
  }: LongPressOptions
) => {
  const timeoutId = useRef<number | null>(null);
  const intervalId = useRef<number | null>(null);
  const isLongPressTriggered = useRef(false);

  const startLongPress = useCallback(() => {
    isLongPressTriggered.current = true;
    onLongPress(); // Fire immediately
    intervalId.current = window.setInterval(onLongPress, repeatInterval);
  }, [onLongPress, repeatInterval]);

  const handlePressStart = useCallback(() => {
    isLongPressTriggered.current = false;
    timeoutId.current = window.setTimeout(startLongPress, delay);
  }, [delay, startLongPress]);

  const handlePressEnd = useCallback(() => {
    if (timeoutId.current) {
      window.clearTimeout(timeoutId.current);
    }
    if (intervalId.current) {
      window.clearInterval(intervalId.current);
    }
    if (!isLongPressTriggered.current) {
      onClick();
    }
  }, [onClick]);

  // Attach touch event listeners manually to control passive option
  useEffect(() => {
    const element = ref.current;
    if (!element) return;

    const handleTouchStart = (e: TouchEvent) => {
      e.preventDefault();
      handlePressStart();
    };

    const handleTouchEnd = (e: TouchEvent) => {
      e.preventDefault();
      handlePressEnd();
    };

    element.addEventListener('touchstart', handleTouchStart, { passive: false });
    element.addEventListener('touchend', handleTouchEnd, { passive: false });

    return () => {
      element.removeEventListener('touchstart', handleTouchStart);
      element.removeEventListener('touchend', handleTouchEnd);
    };
  }, [ref, handlePressStart, handlePressEnd]);

  return {
    onMouseDown: handlePressStart,
    onMouseUp: handlePressEnd,
    onMouseLeave: handlePressEnd,
    // onTouchStart and onTouchEnd are now handled by useEffect
  };
};
