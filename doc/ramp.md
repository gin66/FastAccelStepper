32000 steps in 10s

T=Tr+Tc+Tr = 10s

v = Tr * a

steps = 0.5 * a * Tr² + v * Tc + 0.5 * a * Tr²
      = a * Tr² + v * Tc
      = a * Tr² + (a * Tr) * Tc
      = a * Tr * (Tr + Tc)
      = a * Tr * (T - Tr)

      steps
a = -------------
    Tr * (T - Tr)


Tr = 1s => a = 32000/9  steps/s² = 3556
           v = 3556 => 281us
Tr = 2s => a = 32000/16 steps/s² = 2000
           v = 4000 => 250us
Tr = 3s => a = 32000/21 steps/s² = 15248
           v = 4571 => 218us
Tr = 4s => a = 32000/24 steps/s² = 1333
           v = 5333 => 188us
Tr = 5s => a = 32000/25 steps/s² = 1280
           v = 6400 => 156us


