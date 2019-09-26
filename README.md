# MITRE's Interval Management Sample Algorithm

## NOTICE

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

Copyright 2019 The MITRE Corporation. All Rights Reserved.
Approved for Public Release; Distribution Unlimited. 15-1482

This project contains content developed by The MITRE Corporation. If this code is used in a deployment or embedded within another project, it is requested that you send an email to opensource@mitre.org in order to let us know where this software is being used.

## Licensing

[Apache 2.0](https://github.com/mitre/FMACM/blob/master/LICENSE)

## Documentation

Algorithm descriptions are available via [RTCA's DO-361A](https://my.rtca.org/nc__store?search=do-361) documentation (expected publication date Fall 2020). Please contact RTCA for more information.

## Licensing questions

Any questions related to MITRE Open Source technologies may be emailed to opensource@mitre.org

## Developer Notice

### EUROCONTROL BADA Development Necessary

This code uses [EUROCONTROL's BADA](https://eurocontrol.int/services/bada) for aircraft performance data that support the aircraft dynamics modeling. However, BADA functionality and code cannot be provided due to licensing restrictions imposed by EUROCONTROL. 

### Log4Cplus Dependency

Log4Cplus is a logging application used by this code base. It needs to be installed prior to building this code. You can download it from [their GitHub repo](https://github.com/log4cplus/log4cplus).

### Continuous Integration & Testing

Coming soon...

### Compile & Run

No attempt has been made to ensure that this code will compile on a range of operating systems. This code compiles successfully on Linux machines, specifically [CentOs](https://www.centos.org/) 7 using gcc 4.8.5. For all other computing environments, YMMV.

The [CMake](https://cmake.org/) utility is used to compile this code. If not already installed on the target environment, it is easily installed via `apt`. Please use version 3.0+. From the root directory, execute:

```
>> mkdir build
>> cd build
>> cmake ..
>> make
```

Disappointment time...this code does not produce a self-contained binary executable. Rather it is a library of implemented algorithms provided as a supplement to DO-361A. No attempt has been made to ensure that this compiles into a fully functional API. Examine the unit tests for an understanding of which sections of the code are fully operational.

### Run Unit Tests

Unit tests are implemented to show core algorithm functionality. Tests can be run via the CMake infrastructure.

Assuming the software already compiles:
```
>> cd build
>> make run
```
