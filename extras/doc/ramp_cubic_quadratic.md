# Ramp generator concept for cubic/quadratic ramp combination

Piecewise ramp:
- ramp up with s = 1/6 * j * t^3 and v = 1/2 * j * t^2
  so t = sqrt(2*v/j) and s = 1/6 * j * (2*v/j)^(3/2)
  and (6*s/j)^(2/3) = 2*v / j
  and v = j/2 * (6/j)^(2/3) * s^(2/3) = 6^(2/3)/2 * j^(1/3) * s^(2/3)
- define hand over point s_h
- ramp up to max speed with s = 1/2 * a * t'^2 = 1/2 * v^2 / a

Conditions at hand over point s_h:
-    s_h = 1/6 * j * t_h^3 = 1/2 * a * t'_h^2 + ds
-    v_h = 1/2 * j * t_h^2 =       a * t'_h
-    a_h =       j * t_h   =       a

Replacing j = a / t_h:
-    s_h = 1/6 * a * t_h^2 = 1/2 * a * t'_h^2 + ds
-    v_h = 1/2 * a * t_h   =       a * t'_h

With t'_h = t_h - dt:
-    s_h = 1/6 * a * t_h^2 = 1/2 * a * (t_h^2 - 2*t_h*dt + dt^2) + ds
-    v_h = 1/2 * a * t_h   =       a * t_h    - a * dt

Consequently from v_h:
	 dt = th/2

And for s:
-    s_h = 1/6 * a * t_h^2 = 1/8 * a * t_h^2 + ds

So ds:
	ds = (8-6)/48 * a * t_h^2 = 1/24 * a * t_h^2 = s_h/4

So we have clear relation from s to the ramp speed:

	if s <  s_h, then cubic ramp:      v(s) = 6^(2/3)/2 * j^(1/3) * s^(2/3)
	if s >= s_h, then quadratic ramp:  v(s) = sqrt(2 * a * (s-s_h/4))

Still need to reduce j and s_h to one parameter of freedom.
If choose s_h as user defined parameter, then:

	t_h = sqrt(6 * s_h / a)

and so:

	j = a / t_h = a / sqrt(6 * s_h / a) = sqrt(a^3 / (6 * s_h))

For the speed calculation cubic ramp, this yields:

	v(s) = [6^(2/3)/2 * (a^3/6/s_h)^(1/6)] * s^(2/3)
	     = [6^(4/6)/2 * (a^3)^(1/6) * 6^(-1/6) * s_h^(-1/6)] * s^(2/3)
	     = [6^(3/6)/2 * sqrt(a)                * s_h^(-1/6)] * s^(2/3)
	     = [sqrt(6)/2 * sqrt(a)                * s_h^(-1/6)] * s^(2/3)
	     = [sqrt(3/2) * sqrt(a) / s_h^(1/6)                ] * s^(2/3)

Check cubic:
    v(s_h) = [sqrt(3/2) * sqrt(a) / s_h^(1/6)                ] * s_h^(2/3)
           = [sqrt(3/2) * sqrt(a) ] * s_h^(1/2)

and quadratic:
	v(s_h) = sqrt(2 * a * s_h * 3 / 4) = sqrt(3/2 * a * s_h) equals cubic

With s_h = 0 there will be no cubic ramp start.

