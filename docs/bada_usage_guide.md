# BADA Usage Guide

[Back to Landing Page](/README.md)

This Sample Algorithm code uses [EUROCONTROL's BADA](https://eurocontrol.int/services/bada) for aircraft performance data and some calculations. But, the use of BADA data is limited and you may not need to use the same approach that we did. As a minimum operational requirement, your algorithm implementation must limit the IM Speeds according to ownship's flight envelope. 

For discussions of IM Speed Limiting, see also DO-361A Section 2.2.4.5.3, Appendix C.4.7, and Appendix K. 

**NOTE**: The kinematic trajectory prediction algorithms (e.g. [KinematicDescent4DPredictor.cpp](https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/KinematicDescent4DPredictor.cpp)) do _not use BADA_ at all.

## Code Overview of BADA Usage

In our implementation, we chose to use BADA's flight envelope as a way to meet the miniumum requirement and ensure that the IM Speeds produced by the algorithm were not outside of ownship's speed bounds. 

The significant portion of this implementation is found in [IMAlgorithm::LimitImSpeedCommand()](https://github.com/mitre/im_sample_algorithm/blob/968030837f662a197c0e2755280756b18ce9f5b6/IntervalManagement/IMAlgorithm.cpp#L397):

```c++
Units::Speed IMAlgorithm::LimitImSpeedCommand(
    const Units::Speed im_speed_command_ias,
    const double reference_velocity_mps,
    const Units::Length distance_to_go_to_abp,
    const BadaWithCalc &bada_with_calc,
    const Units::Length ownship_altitude,
    const int flap_configuration,
    const Units::Speed rf_upper_limit) {

...
   // flight envelope protection
   if (limitedspeed > bada_with_calc.flight_envelope.V_mo) {
      limitedspeed = bada_with_calc.flight_envelope.V_mo;
      if (m_quantize_flag) {
         int ilimitspeed = Units::KnotsSpeed(limitedspeed).value();
         int iqthreshold = Units::KnotsSpeed(qThreshold).value();
         limitedspeed = Units::KnotsSpeed(ilimitspeed - (ilimitspeed % iqthreshold));
      }
      m_active_filter_flag |= 64;
   }

...
   // flaps deployed protection
   if (flap_configuration > 0) {
      if (flap_configuration == 1 && limitedspeed > bada_with_calc.mFlapSpeeds.VappMax) {
         limitedspeed = bada_with_calc.mFlapSpeeds.VappMax;
         m_active_filter_flag |= 512;
      } else if (flap_configuration == 2 && limitedspeed > bada_with_calc.mFlapSpeeds.VlndMax) {
         limitedspeed = bada_with_calc.mFlapSpeeds.VlndMax;
         m_active_filter_flag |= 512;
      } else if (flap_configuration == 3 && limitedspeed > bada_with_calc.mFlapSpeeds.VgearMax) {
         limitedspeed = bada_with_calc.mFlapSpeeds.VgearMax;
         m_active_filter_flag |= 512;
      }
   }

...

    }
```

## Using this Code without BADA

External users of this code may prefer not to use the BADA model for aircraft performance data. It is reasonably possible to entirely remove BADA from this code base and still get desired performance. To do so, supply an alternate data source for ownship's flight envelope (e.g. min/max speeds under various operating conditions) and provide those instead. Pay particular attention to [IMAlgorithm::LimitImSpeedCommand()](https://github.com/mitre/im_sample_algorithm/blob/968030837f662a197c0e2755280756b18ce9f5b6/IntervalManagement/IMAlgorithm.cpp#L397). 