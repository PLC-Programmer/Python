2025-02-25: work in progress

The German term "PT2" (https://de.wikipedia.org/wiki/PT2-Glied) is equivalent to a "2nd order lag" process.

\
In this directory is refer to this old but excellent book from 1999 by Ernst-Guenther Feindt: **"Computersimulation von Regelungen - 
Modellbildung und Softwareentwicklung"** (https://www.degruyter.com/document/doi/10.1515/9783486799002/html) and there specifically to chapter "3.5 Simulation einer kontinuierlichen Regelung, Kontrolle der Simulationsgenauigkeit, Parameterbestimmung mit Simulation einer selbstoptimierenden Regelung" ("Simulation of continuous control, control of simulation accuracy, parameter determination with simulation of self-optimizing control") under section "3. Simulation der kontinuierlichen Regelungen" ("Simulation of continuous controls").

Actually, this book is the source of my motivation for the whole "PID control" directory: https://github.com/PLC-Programmer/Python/tree/master/systems%2Cfilters_and_feedback-controls/PID-control

Ironically this book does not feature a PID controller, all examples are only done with PI controllers.

<br/>
The featured process can be modeled with transfer function **G(s) =  = b0 / (s² + a1·s + a0)**

with these parameters:

* b0 = 1.3
* a0 = 1.0
* a1 = 1.6

which leads to a naturally undamped process with parameters:

* ω0 = omega0 = np.sqrt(a0) (natural angular frequency) => ω0 = 1.0/sec
* a1 = 2·D·ω0
* D = a1 / 2.0 / omega0 = 0.8, which makes this process underdamped (D < 1.0)














