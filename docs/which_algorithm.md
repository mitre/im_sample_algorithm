# Achieve or Maintain?

[Back to Landing Page](/README.md)

The IM Sample Algorithm is a control law with two stages: Acheive, Maintain. These two terms are related to the [operational clearances](im_clearance_types.md), but are not exactly the same and should not be confused as having a one-to-one mapping. Furthermore, each stage can be viewed as its own individual control law. For a detailed discussion of the inner workings of these algorithm stages, see DO-361A Appendix C.

There are times when it is helpful to know which stage the algorithm is in. These stages are defined in [IMAlgorithm.h](https://github.com/mitre/im_sample_algorithm/blob/master/include/imalgs/IMAlgorithm.h). 

First the code defines the `FightStage` enumeration which looks like this:

```c++
enum FlightStage
   {
      UNSET = -1,
      NONE = 0,
      ACHIEVE = 1,
      MAINTAIN = 2
   };
```

And the class also provides a getter to access the current stage, which is defined as:

```c++
const FlightStage GetFlightStage() const;
```

The stage itself is managed internally by each `IMAlgorithm` concrete class. For instance, in [IMTimeBasedAchieve.cpp](https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/IMTimeBasedAchieve.cpp) you will find this code in the `HandleAchieveStage` method:

```c++
Guidance IMTimeBasedAchieve::HandleAchieveStage(
    const AircraftState &current_ownship_state,
    const AircraftState &current_target_state,
    const vector<AircraftState> &target_adsb_history,
    const DynamicsState &three_dof_dynamics_state,
    Guidance &guidance_out) {

   m_stage_of_im_operation = ACHIEVE;

...

    }
```