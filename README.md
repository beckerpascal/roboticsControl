# roboticsControl

Repository for the class 'Software Technology Course with a Varying Content' and the topic 'Introduction in Artificial Intelligence'.

Some useful links:

- [V-Rep Doku](http://coppeliarobotics.com/helpFiles/index.html)


## PID tunings

A few different tunings found out to work more or less well.

### Bullet engine

#### |a|:
- 50ms - 13.7 0.199 1286
- 10ms - 56.25, 2.736, 1389

#### a**2 + v**2:
- 50ms - 13.76 0.1990 1609

#### a**2 + |x|
- 50ms - 13.7 0.2197 1194

#### a**2 + x**2
- 50ms - 36.72, 0.5333, 2036 (bad)
- 10ms - ?

#### a**2 + x**2 / t**2
- 10ms - 26.11, 1.971, 2925

#### |a| + |x| + |v| / t^2
- 10ms - 12.73 0.3582 3042

#### Pitch-delta
- Doomed to drift...
- 50ms - 0.5, 0.005, 12.5

#### Oldies but goldies
- 50ms: 21.000000 9.009500 16.550000
- 10ms: 79.0 19.9 41.4

### ODE engine

#### a**2 + x**2 / t**2
- 10ms - 324 3.24 4500

#### |a| + |x| + |v| / t^2
- 10ms - 80.77 0.5011 330.0

