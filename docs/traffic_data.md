# State Data for Ownship & Traffic

[Back to Landing Page](/README.md)

As discussed in the [entry points](entry_points.md) document, the code provides an `Update` method in [IMAlgorithm.h](https://github.com/mitre/im_sample_algorithm/blob/master/include/imalgs/IMAlgorithm.h#L80) that is used to call the algorithm repeatedly.

```c++
virtual Guidance Update(const Guidance &prevguidance,
                        const DynamicsState &dynamicsstate,
                        const AircraftState &owntruthstate,
                        const AircraftState &targettruthstate,
                        const vector<AircraftState> &targethistory);
```

## Ownship State Definition

The ownship data (`owntruthstate` in the `Update` method above) is not expected to conform to ADS-B source definitions. The code provides a container class in [AircraftState](https://github.com/mitre/FMACM/blob/master/include/public/AircraftState.h) that defines the data elements needed for the IM Sample Algorithm to work correctly. The data elements of significance are:

* Ownship ID (an integer uniquely identifying ownship's aircraft source)
* Simulation Time for this state (time of applicability)
* Position (3 dimensional: x, y, z)
* Ground Velocity (3 dimensional: x, y, z)
* Sensed Wind Velocity (2 dimensional: x, y)
* Sensed Outside Air Temperature

## Traffic State Definition

The IM Sample Algorithm must be provided with traffic data at each update step (`targettruthstate` in the `Update` method above). The traffic data is not defined according to any "raw" definition of ADS-B data. However, it is reasonably derived from the information contained in an ADS-B traffic message. The code provides a container class in [AircraftState](https://github.com/mitre/FMACM/blob/master/include/public/AircraftState.h) that defines the data elements needed for the IM Sample Algorithm to work. The data elements of significance are:

* Traffic ID (an integer uniquely identifing this state's source aircraft)
* Simulation Time (for this state, analagous to time of applicability)
* ENU Position (3 dimensional: x, y, z)
* ENU Ground Velocity (3 dimensional: xd, yd, zd)

## IM Sample Algorithm Data Frequency

The IM Sample Algorithm is a control law designed to run at a nominal frequency of 1 Hz. However, that is not a strict requirement. The MITRE Corporation's [own research activities](coming_soon.md) have successfully used this algorithm in a variable-frequency environment with no observed ill behavior. However, deviation from a nominal 1 Hz update rate is not recommended.

## Using ADS-B Source Data for Traffic

There is no code provided to handle binary ADS-B source data. Use of such data will require custom development to translate that data souce into the state data definition described above for Traffic.




