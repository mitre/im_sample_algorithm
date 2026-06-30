# MITRE's Interval Management Sample Algorithm

- [MITRE's Interval Management Sample Algorithm](#mitres-interval-management-sample-algorithm)
  - [TL;DR](#tldr)
  - [The Legal Stuff](#the-legal-stuff)
    - [Notice](#notice)
    - [Licensing](#licensing)
  - [Published Documentation](#published-documentation)
  - [Living Documentation](#living-documentation)
  - [EUROCONTROL BADA Development -- Necessary?](#eurocontrol-bada-development----necessary)
  - [Related Project](#related-project)

## TL;DR

![Apache 2.0](https://img.shields.io/badge/license-Apache%20License%202.0-blue?style=for-the-badge)

![CMake](https://img.shields.io/badge/CMake-%23008FBA.svg?style=for-the-badge&logo=cmake&logoColor=white)
![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)

![Rocky Linux](https://img.shields.io/badge/-Rocky%20Linux-%2310B981?style=for-the-badge&logo=rockylinux&logoColor=white)
![macOS](https://img.shields.io/badge/mac%20os-000000?style=for-the-badge&logo=macos&logoColor=F0F0F0)

This [repository](https://github.com/mitre/im_sample_algorithm) contains a C++ library with some tests.
_It is not an application._
GitHub [CI](../.github/workflows/ci.yml) is used to build & run tests.

You should be familiar with the [FAA's Flight-deck Interval Management](https://www.faa.gov/about/office_org/headquarters_offices/ang/offices/tc/library/storyboard/detailedwebpages/im.html) concept. To understand any of this content at a technical level, you need to also have [RTCA's DO-361A, Appendix C](https://my.rtca.org/).

For an application that can use this library, see the [FIM MOPS Aircraft & Control Model](https://mitre.github.io/FMACM/) (FMACM).

MITRE is happy to answer questions; please [post your questions publicly](https://github.com/mitre/im_sample_algorithm/issues) and we'll do our best to respond.

Good luck. :crossed_fingers: :four_leaf_clover:

## The Legal Stuff

### Notice

> This is the copyright work of The MITRE Corporation, and was produced
> for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
> is subject to Federal Aviation Administration Acquisition Management
> System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
> (Oct. 1996).  No other use other than that granted to the U. S.
> Government, or to those acting on behalf of the U. S. Government,
> under that Clause is authorized without the express written
> permission of The MITRE Corporation. For further information, please
> contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
> McLean, VA  22102-7539, (703) 983-6000.
>
> Copyright 2020 The MITRE Corporation. All Rights Reserved.
> Approved for Public Release; Distribution Unlimited. 15-1482

This project contains content developed by The MITRE Corporation. If this code is used in a deployment or embedded within another project, it is requested that you send an email to <opensource@mitre.org> in order to let us know where this software is being used.

### Licensing

[Apache 2.0](https://github.com/mitre/im_sample_algorithm/blob/master/LICENSE)

Any questions related to MITRE Open Source technologies may be emailed to <opensource@mitre.org>

## Published Documentation

Official algorithm descriptions are available via [RTCA's DO-361A](https://my.rtca.org/nc__store?search=do-361) documentation. Please contact RTCA for more information.

## Living Documentation

Living, developer-level documentation is provided on this GitHub site. Here the goal is to provide deeper detail regarding how our code works and how others might use this code to inform their own implementations of DO-361A. All topics assume the reader has access to DO-361A, specifically Appendix C.

- **Big Picture** of this codebase -- read [the context](context.md) overview

- **Modeling Topics**

  - Mapping DO-361A [Appendix C to published code](appendix_url_mapping.md)
  - Interval Management [clearance types](im_clearance_types.md)
  - [Navigation Database/ARINC-424](navdb.md) discussion
  - [Runtime Frequency](traffic_data.md) considerations
  - How does the [kinematic trajectory prediction](kinematic_prediction.md) work? -- [Coming Soon](coming_soon.md)
  - [Coordinate systems](coordinate_systems.md) used in the code -- [Coming Soon](coming_soon.md)
  - EUROCONTROL BADA v3.7. Do you need it? -- Review [the usage guide](bada_usage_guide.md)

- **Code Topics**

  - Adding this library to a larger CMake simulation framework? Review the [short CPM integration recommendation](context.md#short-recommendation-cpm-integration)
  - Who you gonna call? :ghosts: Read [code entry points](entry_points.md)
  - How does an algorithm receive IM Clearance details? Read [code entry points](entry_points.md)
  - How does an algorithm receive a new aircraft state? Read [code entry points](entry_points.md) and the [traffic data discussion](traffic_data.md)
  - Find your own (shorter) path: [IM Turn Implementation](imturn.md)
  - What are the ADS-B Data expectations? -- Review the [traffic data discussion](traffic_data.md)
  - Where is the [IFPI defined](ifpi.md)? -- [Coming Soon](coming_soon.md)
  - Uh...Which algorithm is running? -- Review [the algorithm stages](which_algorithm.md) discussion
  - Are we missing some code? Review the [missing code table](missing_code.md)

Not seeing what you need to know about? [Post an issue](https://github.com/mitre/im_sample_algorithm/issues).
Thanks!

## EUROCONTROL BADA Development -- Necessary?

Do you need a full BADA implementation to use this code base? **Nope**. Take a look at [the BADA usage guide](bada_usage_guide.md) to get more details.

## Related Project

Directly related to this Sample Algorithm implementation is the [FIM MOPS Aircraft Dynamics & Control Model](https://mitre.github.io/FMACM). That code is a fully operational aircraft simulation the comes with compile instructions and passing unit tests. It is the core simulation that MITRE used to implement and test the Sample Algorithm control law discussed here.

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>(Table of Contents generated with markdown-toc)</a></i></small>
