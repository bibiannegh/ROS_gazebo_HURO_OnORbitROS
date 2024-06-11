# Experiments to validate OnOrbitROS

Some experiments have been conducted to validate the framework. 

---

## OnOrbitROS Dynamics: ETS VII simulation

The dynamics of the ETS VII robotic experiments are simulated and compared to the actual flight data obtained from the mission. 

The results obtained are presented in: 
<span style="color:gray">
    Ramón, J. L., Pomares, J., & Felicetti, L. (2023). Task space control for on-orbit space robotics using a new ROS-based framework. Simulation Modelling Practice and Theory, 127(102790), 102790. [https://doi.org/10.1016/j.simpat.2023.102790](https://doi.org/10.1016/j.simpat.2023.102790)
</span>

And compared to the real mission from: 
<span style="color:gray">
    S. Abiko, K. Yoshida, Post flight analysis of ETS-VII space robotic experiments, in: Proceedings of the 6th International Symposium on Artificial Intelligence and Robotics & Automation in Space: I-SAIRAS, St-Hubert, Quebec, Canada, 2001.
</span>
and 
<span style="color:gray">
    K. Yoshida, Engineering test satellite VII flight experiments for space robot dynamics and control: theories on laboratory test beds ten years ago, now in orbit, Int. J. Rob. Res. 22 (2003) 321–335, https://doi.org/10.1177/0278364903022005003.
</span>

---

## OnOrbitROS propagator

Two orbits, a circular an elliptic, have been simulated and compared to GMAT's RungeKutta89 propagator. 
The satellite used is a sphere with 1m of diameter orbiting around Earth in LEO. 

The selected orbit has been taken from the Cubesat XI-V orbit on 17th April 2024: 

```yaml
    1 28895U 05042F 24108.35401664 .00006071 00000-0 10830-2 0 9996
    2 28895  98.1916 244.1444 0015992 185.6352 174.4674 14.67269849985536
```

>> ### Circular Orbit

The orbit has been taken from the previous TLE set (in km and º) and the eccentricity set to 0:

    eccentricity: 0.0
    semi_major_axis: 7047.5
    inclination: 98.1916
    rate_of_right_ascension: 244.1444  
    right_ascension_ini: 244.1444
    argument_of_perigee_ini: 185.6352
    rate_argument_Of_perigee: 185.6352
    mean_anomaly_ini: 174.4674

    time_pass_perigee:
    sec : 0
    min: 0
    hour: 0
    mday: 1 
    mon: 1
    year: 2000

    time_start:
    sec : 0
    min: 0
    hour: 0
    mday: 1 
    mon: 1
    year: 2000



In the graphs below, the orbital position relative to the ECI (Earth MJ2000) frame is shown for the first 15,000 seconds of the propagated orbit. The left panels display results from OnOrbitROS, while the right one show results from GMAT. The `x`, `y`, and `z` axes are represented in red, green, and blue respectively. The satellite's altitude is depicted in orange.

The table following the graphs presents the numerical data for the maximum values of the orbital position and the mean altitude. The orbital shapes and maximum values obtained using OnOrbitROS are consistent with those from GMAT, with relative errors under 1%, using GMAT as the reference standard.

<img src="/images/circularOnOrbitROS.png" alt="OnOrbitROS" style="width: 45%;" /> <img src="/images/circularGMAT.jpg" alt="OnOrbitROS" style="width: 53%;" />

<img src="/images/circularResults.jpeg" alt="OnOrbitROS" style="width: 50%;" />


>> ### Elliptic Orbit

The same orbit has been evaluated in this case with an eccentricity value of 0.01. 

    eccentricity: 0.01

The following graphs show the results for the elliptic orbit, following the same format as used for the circular orbit. The shapes of the orbits in both cases are consistent, with a maximum relative error of 1.64%.

<img src="/images/elipticOnOrbitROS.png" alt="OnOrbitROS" style="width: 45%;" /> <img src="/images/ellipticGMAT.jpg" alt="OnOrbitROS" style="width: 53%;" />

<img src="/images/ellipticResults.jpeg" alt="OnOrbitROS" style="width: 50%;" />

---

## Other validations


!!! Warning
    More validations on orbital mechanics and perturbation modelling to come.