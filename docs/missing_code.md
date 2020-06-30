# Missing Code

Okay, we know this repository is incomplete from a code perspective. And we recognize that's not ideal for interested parties that would like to see a working sample algorithm. While we can't make this all better, we can do our best to help the traveller find their way. Here are some thoughts and responses to code questions we've received.

| Missing Implementation | Our Response | 
| --- | --- |
| aaesim/ads_types.h | There are no algorithms here -- just simple structs. We recommend that you can reverse engineer what's missing to fit your needs. |
| aaesim/KineticDescent4DPredictor.h | You don’t really need this. We recommend removing all references and filling in the few compile gaps that result with your own algorithms. You do have KinematicDescent4DPredictor and that's all you really need for predictions. Implement DO-361A Appendix C.2.1. |
| aaesim/KineticTrajectoryPredictor.h | Implement DO-361A Appendix C.2.1 yourself for this functionality. |
| aaesim/SimpleAircraft.h | This is not needed. You can safely remove all references. We'll attempt to remove this from the includes in a future release. |
| aaesim/TrajectoryPredictor.h | You need to implement DO-361A Appendix C.2. |
| aaesim/TrueDistances.h | This is only a container of post-processed results for our own convenience. You can safely remove it without impacting any IM algorithm. We'll consider removing this in a future release. |
| aaesim/VerticalPredictor.h | You need to implement DO-361A Appendix C.2.2. See also [KinematicDescent4DPredictor](https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/KinematicDescent4DPredictor.cpp). |
| aaesim/Bada.h | Sorry, we can't help this. See our [note in the project README](./README.md). |
| aaesim/PredictedWindEvaluator | This is missing in 3.4.1, but will be provided in the following [release](https://github.com/mitre/im_sample_algorithm/tags). |

Not seeing what you need to know about? [Post an issue](https://github.com/mitre/im_sample_algorithm/issues). We gladly prioritize specific requests. Thanks!
