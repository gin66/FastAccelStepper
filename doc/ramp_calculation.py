#!/usr/bin/env python3

import matplotlib.pyplot as plt

v_1 = 3000.1
v_2 = 1000.1
a = 100.0

T = abs(v_2 - v_1)/a
S = (v_1 + v_2) * T / 2
F = 2 * v_1 / (v_1 + v_2)
print("T = %.3f  S = %.3f  F = %.3f  calc(v1) = %.3f" % (T,S,F,S/T*F))


t = 0
dt = 0.1
Tr = T
Sr = S

tx = []
sx = []
vx = []
ax = []

f = F
v = v_1
while t < T:
    tx.append(t)
    sx.append(Sr)
    vx.append(v)
    v = f * Sr / Tr
    if v_2 > v_1:
        if v_2 < v:
            v = v_2
    else:
        if v_2 > v:
            v = v_2
    ds = v * dt
    f = 2 * v / (v + v_2)
    Tr -= dt
    Sr -= ds
    t += dt
    ax.append((v - vx[-1]) / dt)
    print("Tr = %.3f  Sr = %.3f  F = %.3f  v=%.3f  dt=%.3f  ds=%.3f" % (Tr,Sr,F,v,dt,ds))

fig, (ax1,ax2) = plt.subplots(2,1)
ax1.set_title("acceleration over time")
ax2.set_title("speed over time")
ax1.plot(tx,ax)
ax2.plot(tx,vx)
plt.show(block=True)
