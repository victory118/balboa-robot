# Balboa Robot Notes

## To do

* Modify pendulum simulation for Balboa robot
  * Add simulation for 
* 

## Balboa specifications

### Inertial Sensors (Accelerometer and Gyro)

Source: https://github.com/pololu/balboa-32u4-arduino-library/blob/master/examples/InertialSensors/InertialSensors.ino

```
// This example reads the raw values from the LSM6DS33
// accelerometer and gyro and and the LIS3MDL magnetometer on the
// Balboa 32U4, and prints those raw values to the serial
// monitor.
//
// The accelerometer readings can be converted to units of g
// using the conversion factors specified in the "Mechanical
// characteristics" table in the LSM6DS33 datasheet.  We use a
// full scale (FS) setting of +/- 16 g, so the conversion factor
// is 0.488 mg/LSB (least-significant bit).  A raw reading of
// 2048 would correspond to 1 g.
//
// The gyro readings can be converted to degrees per second (dps)
// using the "Mechanical characteristics" table in the LSM6DS33
// datasheet.  We use a full scale (FS) of +/- 1000 dps so the
// conversion factor is 35 mdps/LSB.  A raw reading of 2571
// would correspond to 90 dps.
//
// The magnetometer readings are more difficult to interpret and
// will usually require calibration.
//
// To run this sketch, you will need to install the LSM6 and
// LIS3MDL libraries:
//
// https://github.com/pololu/lsm6-arduino
// https://github.com/pololu/lis3mdl-arduino
```

### Component Masses

The stability conversion kit shown below weighs 33 g. I do not know for sure if this includes the weight of the ball caster.

<img src="balboa-robot-notes.assets/0J8766.600x480.jpg" alt="img" style="zoom:25%;" />

* https://www.pololu.com/product/3572/specs

The weight of the assembled robot with motors, 80x10 mm wheels, and bumper cage shown below is 200 g (not including batteries).

<img src="balboa-robot-notes.assets/0J7789.600x480.jpg" alt="img" style="zoom:25%;" />

* https://www.pololu.com/product/3575/specs

The weight of each rechargeable battery is 0.952 oz (27.0 g)

* http://www.panasonicbatteryproducts.com/eneloop_rechargeable_batteries/eneloop_rechargeable_batteries-aa_4-pack/
* https://www.batteryjunction.com/eneloop-hr-3uwxa8a-8aa.html

The total weight of the robot base including batteries (but not including wheels) is $m_b = 200 \text{ g} + 6\cdot 27 \text{ g} = 362 \text{ g}$.

The center of mass location of the robot base including batteries (but not including wheels) can be found by analyzing the dimensions of the Balboa 32U4 Balancing Robot with 80x10mm wheels as shown below:

<img src="balboa-robot-notes.assets/0J7596.1200.png" alt="img" style="zoom:50%;" />

The length of the robot base is 118.3 - 12 = 106.3 mm. Approximate the length as 106 mm. The distance from the base bottom to the motor axis is 40 - 12 = 28 mm. Assuming the weight is equally distributed across the base, the center of mass is approximately 106/2 = 53 mm from the base bottom. Thus the center of mass is located $d_b = 53 - 28 = 25 \text{ mm}$ above the motor axis. The motor and gearbox add significant weight near the motor axis. Each motor weighs 9.5 g, so two motors and two gearboxes weighs at least as much as one battery (27 g). Accordingly, I will adjust the center of mass location of the robot base to be located $L_b = 22 \text{ m}$ above the motor axis. The motor and gearbox are located at the motor axis so they do not contribute to the moment of inertia about the motor axis .

* Motor specs: https://www.pololu.com/product/3073/specs

### Body Moment of Inertia

The mass moment of inertia of a rectangular prism about the X axis is given by $I_X = \frac{m(a^2+b^2)}{12}$.

![img](balboa-robot-notes.assets/in_box.gif)

Here $a = 106 \text{ mm}$ corresponds to the length of the robot base, $b=22 \text{ mm}$ corresponds to the depth, and $m = m_b = 362 \text{ g}$. We are interested in calculating the mass moment of inertia of the robot base about the motor axis so we must use the parallel axis theorem:

$I_m = I_X + m_b d_b^2 = 5.7980\cdot 10^{-4} \text{ kg}\cdot\text{m}^2$

```matlab
>> a = .106;
>> b = 0.022;
>> mb = .362;
>> db = 0.025;
>> Im = mb*(a^2 + b^2)/12 + mb*db^2
Im =
   5.7980e-04
```

> Note that I use the distance $d_b$ instead of the effective center of mass offset $L_b$ because $L_b$ accounts for the extra mass of the motors and gearbox, but they do not contribute to moment of inertia about the motor axis.

### Wheel Moment of Inertia

The wheels are 80 mm (3.15") in diameter and have a thickness of 10 mm. The weight is 0.7 oz (0.02835 kg) per wheel, including the tire.

An image of the wheel is shown below:

<img src="balboa-robot-notes.assets/0J8859.1200.jpg" alt="img" style="zoom:10%;" />

The moment of inertia of common objects is given below: 

![Question #f7941 | Socratic](balboa-robot-notes.assets/Z4Px0NkERHnhvNUjiqzw_moment_of_inertia_chart.jpg)

To simplify the moment of inertia calculation, I assume the geometry of the wheel to be a thin hoop of radius $R_w$ for the rim with six long uniform rods of length $R_w$ as the spokes. I also assume the wheel has a uniform mass per unit length $\rho_2=m_w/L_w$, where $m_w$ is the total mass of the wheel and $L_w = 2\pi R_w + 6R_w$ is the total length of the wheel geometry.

> Note: This is not truly accurate because rubber and plastic have different densities. Also, the rim and spokes are not infinitely thin.

Then the moment inertia can be calculated by:
$$
\begin{align}
I_w &= I_{rim} + 6I_{spoke}\\
&= m_{rim}R_w^2 + 6(\frac{1}{3}m_{spoke}R_w^2) \\
&= \rho_w L_{rim}R_w^2 + 6(\frac{1}{3}\rho_w L_{spoke}R_w^2)\\
&= \frac{m_w}{(2\pi + 6)R_w}(2\pi R_w)R_w^2 + 6(\frac{1}{3}\frac{m_w}{(2\pi + 6)R_w}( R_w)R_w^2)\\
&= \frac{2\pi+2}{2\pi+6} m_w R_w^2 \\
&\approx 0.6744 m_w R_w^2
\end{align}
$$
The value of $0.6744m_wR_w^2$ makes sense because this is in between $\frac{1}{2}m_w R_w^2$ for a solid cylinder and $m_wR_w^2$ for a thin hoop. Remember to double the mass when calculating the inertia because there are two wheels. Substituting values for $2m_w$ and $R_w$, we get $I_w=6.1177\times10^{-5} \:\text{kg}\cdot\text{m}^2$.

```matlab
>> mw = 0.02835;
>> Rw = 0.08/2;
>> Iw = (2*pi+2)/(2*pi+6)*2*mw*Rw^2
Iw =
   6.1177e-05
```

### Other Moment of Inertia

For now, I assume that the moment of inertias for other rotating parts such as gears and motors are negligible.

**References**

* https://www.pololu.com/product/1434/specs#note1
* https://socratic.org/questions/586e548711ef6b71cc1f7941

## Links

### Software

In 

