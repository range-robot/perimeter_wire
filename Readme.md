

= Perimeter Wire =


== Resources == 

 * https://www.goetting-agv.com/components/inductive
 * http://wiki.ardumower.de/index.php?title=Perimeter_wire
 * https://www.robotshop.com/de/de/robotshop-perimeter-wire-generator-sensor-soldering-kit.html

== Observations ==
 * Robot-Shop and Götting use simple sinus signals. Ardumower uses a code and matching algorithm.
 * Robot-Shop uses 1mH, Arudmower 100mH coil.


 == Theory ==

Magnetic field strength for z coil:
k: konstant
x: distance orthogonal to wire
d: distance above ground (to wire)

B=k * x / (d^2 + x^2)
https://www.wolframalpha.com/input/?i=sin(arctan(x%2Fd))+*+cos(arctan(x%2Fd))%2Fd