# Following the Smoke Trails Collagen Rendering Formula


(https://smoketrailsbbq.com/brisket-holding-masterclass-and-tenderness-model/)

### Background

The smoker is a custom, ESP32-controlled electric smoker. The smoker's temperature is PID-controlled. Data from four brisket probes and the smoker's internal temperature sensor are published every five seconds to an MQTT topic. Software on a local server monitors this temperature data to calculate the percentage of collagen rendered.

### Cook Notes

1. The smoker was initially set to 250°F.
2. The initial brisket temperature was 49°F.
3. The brisket was wrapped when it reached 172°F.
4. When the brisket hit 195°F, the smoker temperature was reduced to 150°F.
5. At 195°F, the collagen was 72.7% rendered.
6. The brisket's internal temperature continued to rise, peaking at 199°F.
7. The smoker took 23 minutes to drop to 150°F.
8. After two hours, the internal brisket temperature had only dropped by 10°F.

> **Note:** I removed the brisket from the smoker after holding it for only two hours because the cooldown was extremely slow. At this point, the collagen rendering had reached 127%.

### Log of Collagen Rendering

**Note:** I am missing the logs between 140°F and 156°F. I would like to blame the software, but it was operator error. I retroactively calculated the collagen rendering for the lost data from the published mqtt data at 1.67%.

    Jul 27 19:54:20 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=157°F rate=2.61%/hr collagen=0.66%
    Jul 27 20:09:22 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=159°F rate=2.95%/hr collagen=1.39%
    Jul 27 20:24:24 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=160°F rate=3.14%/hr collagen=2.18%
    Jul 27 20:39:26 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=161°F rate=3.33%/hr collagen=3.02%
    Jul 27 20:54:27 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=162°F rate=3.54%/hr collagen=3.90%
    Jul 27 21:09:29 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=163°F rate=3.76%/hr collagen=4.85%
    Jul 27 21:24:31 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=168°F rate=5.10%/hr collagen=6.12%
    Jul 27 21:39:33 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=167°F rate=4.80%/hr collagen=7.33%
    Jul 27 21:54:35 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=168°F rate=5.10%/hr collagen=8.60%
    Jul 27 22:09:36 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=169°F rate=5.42%/hr collagen=9.96%
    Jul 27 22:24:38 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=170°F rate=5.76%/hr collagen=11.40%
    Jul 27 22:39:40 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=172°F rate=6.50%/hr collagen=13.03%
    Jul 27 22:54:42 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=173°F rate=6.91%/hr collagen=14.76%
    Jul 27 23:09:44 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=174°F rate=7.35%/hr collagen=16.60%
    Jul 27 23:24:45 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=175°F rate=7.81%/hr collagen=18.56%
    Jul 27 23:39:47 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=176°F rate=8.29%/hr collagen=20.64%
    Jul 27 23:54:49 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=177°F rate=8.81%/hr collagen=22.85%
    Jul 28 00:09:51 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=178°F rate=9.37%/hr collagen=25.19%
    Jul 28 00:24:53 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=179°F rate=9.95%/hr collagen=27.69%
    Jul 28 00:39:54 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=180°F rate=10.58%/hr collagen=30.34%
    Jul 28 00:54:56 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=181°F rate=11.24%/hr collagen=33.15%
    Jul 28 01:09:58 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=182°F rate=11.95%/hr collagen=36.14%
    Jul 28 01:25:00 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=182°F rate=11.95%/hr collagen=39.14%
    Jul 28 01:40:02 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=184°F rate=13.49%/hr collagen=42.51%
    Jul 28 01:55:03 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=185°F rate=14.34%/hr collagen=46.11%
    Jul 28 02:10:05 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=187°F rate=16.19%/hr collagen=50.16%
    Jul 28 02:25:07 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=189°F rate=18.28%/hr collagen=54.74%
    Jul 28 02:40:09 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=191°F rate=20.64%/hr collagen=59.91%
    Jul 28 02:55:11 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=194°F rate=24.77%/hr collagen=66.12%
    Jul 28 03:10:12 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=cooking temp=195°F rate=26.33%/hr collagen=72.71%
    Jul 28 03:10:12 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 resting phase started (collagen=72.71%, temp=195°F) — lowering temp to 150°F
    Jul 28 03:25:14 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=resting temp=197°F rate=29.73%/hr collagen=80.16%
    Jul 28 03:40:16 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=resting temp=199°F rate=33.57%/hr collagen=88.57%
    Jul 28 03:55:18 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=resting temp=199°F rate=33.57%/hr collagen=96.98%
    Jul 28 04:10:20 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=resting temp=197°F rate=29.73%/hr collagen=104.43%
    Jul 28 04:10:20 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 brisket finished (collagen=104.43%, temp=197°F) — cook complete
    Jul 28 04:25:21 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=finished temp=196°F rate=27.98%/hr collagen=111.44%
    Jul 28 04:40:23 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=finished temp=193°F rate=23.31%/hr collagen=117.28%
    Jul 28 04:55:25 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=finished temp=191°F rate=20.64%/hr collagen=122.45%
    Jul 28 05:10:27 hermes smoker[860809]: [BrisketRecipe] smokerId=11100 phase=finished temp=190°F rate=19.43%/hr collagen=127.32%

