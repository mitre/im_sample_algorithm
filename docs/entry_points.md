# Entry Points

[Back to Landing Page](/README.md)

_Note: this code will not compile as-provided. If you're trying to call it, we'll assume you've modified it to make it compile._

From a state-machine perspective, the code follows a simple process path.

* Instantiate a concrete `IMAlgorithm` class
* Call its `Initialize` method to establish its internal states
* Repeatedly call its `Update` method, providing new inputs as the simulation clock advances.

Like this:

![state_sequence](images/state_sequence.png)

## Initialize the Algorithm

You'll need a concrete `IMAlgorithm` class for this step and that choice is based on what you are trying to do. (Unsure? [Consider this.](im_clearance_types.md)) But, in general, you'll probably want to instantiate an [`IMTimeBasedAchieve`](https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/IMTimeBasedAchieve.cpp) or [`IMDistBasedAchieve`](https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/IMDistBasedAchieve.cpp) algorithm. Once that is built, you need to initialize the internal states. You'll find this declared in the parent class, [`IMKinematicAchieve.h`](https://github.com/mitre/im_sample_algorithm/blob/master/include/imalgs/IMKinematicAchieve.h).

```c++
virtual void Initialize(const KineticTrajectoryPredictor &ownship_kinetic_trajectory_predictor,
                        const KineticTrajectoryPredictor &target_kinetic_trajectory_predictor,
                        std::shared_ptr<TangentPlaneSequence> tangent_plane_sequence,
                        AircraftIntent &target_aircraft_intent,
                        const IMClearance &im_clearance,
                        const std::string &achieve_by_point,
                        WeatherPrediction &weather_prediction);
```

## Call the Algorithm

The code provides a `virtual` declaration for the `Update` method in [IMAlgorithm.h](https://github.com/mitre/im_sample_algorithm/blob/master/include/imalgs/IMAlgorithm.h#L80).

```c++
virtual Guidance Update(const Guidance &prevguidance,
                        const DynamicsState &dynamicsstate,
                        const AircraftState &owntruthstate,
                        const AircraftState &targettruthstate,
                        const vector<AircraftState> &targethistory);
```
Each time this method is called, it is assumed that the simulation clock has advanced and new data is being provided to the algorithm. You should be providing:

* the `Guidance` object previously returned, but update with the aircraft's current Selected Speed (analagous to the speed value on the Mode Control Panel)
* a `DynamicsState` struct that indicates the current configuration of the aircraft (e.g. flaps settings, etc)
* an `AircraftState` that represents ownship's truth state (or sensed state, depending on the fidelity of your simulation)
* an `AircraftState` that represents traffic's most recent ADS-B state
* a vector of the traffic's ADS-B states relevant to the IM operation

The return is a `Guidance` object which contains a new IM Speed in the `m_ias_command` class member.

## Data Returned by the Algorithm

Each time this method is called, it returns a `Guidance` object. That object has [many class members](https://github.com/mitre/FMACM/blob/master/include/public/Guidance.h), but it operates as a `struct` by holding the latest value of any given member; most members are `public`. :unamused: There are only two output members of importance (from an IM Algorithm perspective) in the Guidance object:

* `m_ias_command`: the latest speed calculated (unitized for disambiguation)
* `m_valid`: a boolean indicating if the object contains valid data

_The only members of the Guidance class that you should pay attention to are these two members. All others are unused by the IMAlgorithm classes._

This output can also be accessed via a simple getter exposed by [`IMAlgorithm`](https://github.com/mitre/im_sample_algorithm/blob/master/include/imalgs/IMKinematicAchieve.h#L40):

```c++
   virtual const Units::Speed GetImSpeedCommandIas() const;
```

We've noticed that the `Units::Speed` object sometimes truncates bits when converting from meters-per-second to knots. The result is that the above call to GetImSpeedCommandIas() can sometimes give a value very near to an integer IAS value, but not quite (e.g. 269.9999999 rather than 270.0). This can look odd on a human-interface display. There are multiple work-arounds to this problem, but ours has been to also provide an integer accessor for the IM Speed. It's in the [Guidance](https://github.com/mitre/FMACM/blob/d8156ca28e9f9073c933ea5776d9bde91f003b0c/include/public/Guidance.h#L38) object and looks like this:

```c++
int GetIasCommandIntegerKnots() const;
```

This call is implemented such that it is guaranteed to return a clean value appropriate for human-interface displays.