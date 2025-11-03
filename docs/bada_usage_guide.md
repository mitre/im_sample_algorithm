# BADA Usage Guide

[Back to Landing Page](/README.md)

This Sample Algorithm code benefits from [EUROCONTROL's BADA](https://eurocontrol.int/services/bada) model for aircraft performance data and some calculations.
But the use of BADA data is intentionally limited in scope and you may not need to use the same approach that we did.
Note that the code compiles and passes tests without any BADA code/library provided.

For formal documentation of IM Speed Limiting, see also DO-361A Section 2.2.4.5.3, Appendix C.4.7, and Appendix K.

**NOTE**: The kinematic trajectory prediction algorithms (e.g. [KinematicDescent4DPredictor.cpp](https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/KinematicDescent4DPredictor.cpp)) do _not use BADA_ at all.

## MITRE's BADA Usage

As a minimum operational requirement, a FIM algorithm implementation must limit the IM Speeds according to ownship's flight envelope. In our implementation, we chose to use BADA's flight envelope as a way to meet the miniumum requirements. It provided the data to help us ensure that the IM Speeds produced by the algorithm were within ownship's speed bounds.

The significant portion of this implementation is found in `FIMSpeedLimiter`.

## Using this Code without BADA

External users of this code may prefer not to rely upon the BADA model for aircraft performance data.
As an alternative to BADA, one may consider using the open-source [WRAP aviation data](https://github.com/junzis/wrap) product.
No attempt has been made by MITRE to use that project.
If you try it, let us know how it goes! :four_leaf_clover: