<meta http-equiv="Content-Type" content="text/html; charset=utf-8" />

- [MITRE's Interval Management Sample Algorithm](#mitre-s-interval-management-sample-algorithm)
  * [TL;DR](#tl-dr)
  * [The Legal Stuff (yay!)](#the-legal-stuff--yay--)
    + [Notice](#notice)
    + [Licensing](#licensing)
  * [Published Documentation](#published-documentation)
  * [Living Documentation](#living-documentation)
  * [EUROCONTROL BADA Development Necessary](#eurocontrol-bada-development-necessary)
  * [Related Project](#related-project)

# MITRE's Interval Management Sample Algorithm

## TL;DR

You need [RTCA's DO-361A, Appendix C](https://my.rtca.org/nc__store?search=do-361) to understand any of this.

The code is in [this git repo](https://github.com/mitre/im_sample_algorithm). It doesn't compile; that's intentional. We are considering providing some unit tests.

MITRE is happy to answer questions; the email is opensource@mitre.org.

Good luck. :crossed_fingers: :four_leaf_clover:

## The Legal Stuff (yay!)

### Notice

This is the copyright work of The MITRE Corporation, and was produced
for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
is subject to Federal Aviation Administration Acquisition Management
System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
(Oct. 1996).  No other use other than that granted to the U. S.
Government, or to those acting on behalf of the U. S. Government,
under that Clause is authorized without the express written
permission of The MITRE Corporation. For further information, please
contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
McLean, VA  22102-7539, (703) 983-6000.

Copyright 2020 The MITRE Corporation. All Rights Reserved.
Approved for Public Release; Distribution Unlimited. 15-1482

This project contains content developed by The MITRE Corporation. If this code is used in a deployment or embedded within another project, it is requested that you send an email to opensource@mitre.org in order to let us know where this software is being used.

### Licensing

[Apache 2.0](https://github.com/mitre/im_sample_algorithm/blob/master/LICENSE)

Any questions related to MITRE Open Source technologies may be emailed to opensource@mitre.org

## Published Documentation

Official algorithm descriptions are available via [RTCA's DO-361A](https://my.rtca.org/nc__store?search=do-361) documentation (expected publication date Fall 2020). Please contact RTCA for more information.

## Living Documentation

Living, developer-level documentation is provided on this GitHub site. Here the goal is to provide deeper detail regarding how our code works and how others might use this code to inform their own implementations of DO-361A. All topics assume the reader has access to DO-361A, specifically Appendix C.

* **Big Picture** of this codebase -- read [the context](context.md) overview

* **Developer Talk**: [Does it compile & pass tests?](dev_talk.md) -- hint: nope

* **Modeling Topics**
    * Mapping DO-361A [Appendix C to published code](appendix_url_mapping.md)
    * Interval Management [clearance types](im_clearance_types.md)
    * [Navigation Database/ARINC-424](navdb.md) discussion
    * [Runtime Frequency](traffic_data.md) considerations
    * How does the [kinematic trajectory prediction](kinematic_prediction.md) work? -- [Coming Soon](coming_soon.md)
    * [Coordinate systems](coordinate_systems.md) used in the code -- [Coming Soon](coming_soon.md)
    * EUROCONTROL BADA v3.7. Do you need it? -- [Coming Soon](coming_soon.md)
    
* **Code Topics**
    * Who you gonna call? Read [code entry points](entry_points.md)
    * How does an algorithm receive IM Clearance details? Read [code entry points](entry_points.md)
    * How does an algorithm receive a new aircraft state? Read [code entry points](entry_points.md) and the [traffic data discussion](traffic_data.md)
    * Find your own (shorter) path: [IM Turn Implementation](imturn.md)
    * What are the ADS-B Data expectations? -- Reveiw the [traffic data discussion](traffic_data.md)
    * Where is the [IFPI defined](ifpi.md)? -- [Coming Soon](coming_soon.md)
    * Uh...Which algorithm is running? -- Review [the algorithm stages](which_algorithm.md) discussion
    * Are we missing some code? Review the [missing code table](missing_code.md)
    
Not seeing what you need to know about? [Post an issue](https://github.com/mitre/im_sample_algorithm/issues). We gladly prioritize specific requests. Thanks!

## EUROCONTROL BADA Development Necessary

This code uses [EUROCONTROL's BADA](https://eurocontrol.int/services/bada) for aircraft performance data. However, BADA functionality and code cannot be provided due to licensing restrictions imposed by EUROCONTROL. Please complete your own implementation of these concepts before using the code. See our related project, the [FIM MOPS Aircraft Dynamics & Control Model](https://mitre.github.io/FMACM), for a guide of what is needed.

But, do you need BADA implemented to use this code base? **Nope.** 

Do you need BADA implemented for your project? Take a look at our guide  -- [Coming Soon](coming_soon.md)

## Related Project

Directly related to this sample algorithm implementation is the [FIM MOPS Aircraft Dynamics & Control Model](https://mitre.github.io/FMACM). That code is a fully operational aircraft simulation the comes with compile instructions and passing unit tests. It is the core simulation that MITRE used to implement and test the Sample Algorithm control law discussed here.

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>(Table of Contents generated with markdown-toc)</a></i></small>
