# Interval Management Clearance Types

[RTCA's DO-328B](https://my.rtca.org/nc__store?search=do-328) defines multiple clearance types that may (in the future) be used by Air Traffic Controllers to space aircraft. Here's a quick summary. Further details can be obtained from RTCA.

Clearance Type | Relevant Class
--- | ---
Achieve-by Then Maintain (time) | [IMTimeBasedAchieve](https://github.com/mitre/im_sample_algorithm/blob/master/include/imalgs/IMTimeBasedAchieve.h)
Achieve-by Then Maintain (dist) | [IMDistBasedAchieve](https://github.com/mitre/im_sample_algorithm/blob/master/include/imalgs/IMDistBasedAchieve.h)
Capture Then Maintain (time) | [IMKinematicTimeBasedMaintain](https://github.com/mitre/im_sample_algorithm/blob/master/include/imalgs/IMKinematicTimeBasedMaintain.h)
Capture Then Maintain (dist) | [IMKinematicDistBasedMaintain](https://github.com/mitre/im_sample_algorithm/blob/master/include/imalgs/IMKinematicDistBasedMaintain.h)
Final Approach Spacing | [IMKinematicAchieve](https://github.com/mitre/im_sample_algorithm/blob/master/include/imalgs/IMKinematicAchieve.h)
IM Turn | :disappointed: Not currently available (read [Where is IM Turn?](where_is_im_turn.md))
Maintain Current Spacing (time) | [IMKinematicTimeBasedMaintain](https://github.com/mitre/im_sample_algorithm/blob/master/include/imalgs/IMKinematicTimeBasedMaintain.h)
Maintain Current Spacing (dist) | [IMKinematicDistBasedMaintain](https://github.com/mitre/im_sample_algorithm/blob/master/include/imalgs/IMKinematicDistBasedMaintain.h)
