# Interval Management Clearance Types

[RTCA's DO-361A](https://my.rtca.org/nc__store?search=do-361) defines multiple clearances that will (in the future) be used by Air Traffic Controllers to space aircraft. Here's a quick summary. Further details can be obtained from RTCA.

Clearance Type | Description | Relevant Class
--- | --- | ---
Achieve-by Then Maintain (time) | The given time-based spacing goal is to be achieved by a specific downstream waypoint | [IMTimeBasedAchieve](https://github.com/mitre/im_sample_algorithm/blob/master/include/imalgs/IMTimeBasedAchieve.h)
Achieve-by Then Maintain (dist) | The given distance-based spacing goal is to be achieved by a specific downstream waypoint | [IMDistBasedAchieve](https://github.com/mitre/im_sample_algorithm/blob/master/include/imalgs/IMDistBasedAchieve.h)
Capture Then Maintain (time) | The given time-base spacing goal is to be immediately captured and maintained | [IMKinematicTimeBasedMaintain](https://github.com/mitre/im_sample_algorithm/blob/master/include/imalgs/IMKinematicTimeBasedMaintain.h)
Capture Then Maintain (dist) | The given distance-base spacing goal is to be immediately captured and maintained | [IMKinematicDistBasedMaintain](https://github.com/mitre/im_sample_algorithm/blob/master/include/imalgs/IMKinematicDistBasedMaintain.h)
Final Approach Spacing | Achieve-by Then Maintain for final approach situations | see above
IM Turn | A variant of Achieve-by Then Maintain that shortens the path to the Achieve-by Point | Not currently available [read Where is IM Turn?](where_is_im_turn.md)
Maintain Current Spacing (time) | Like Capture Then Maintain (time), but the current spacing is the spacing goal | see above
Maintain Current Spacing (dist) | Like Capture Then Maintain (dist), but the current spacing is the spacing goal | see above