# Developer Talk

[Back to Landing Page](/README.md)

## Build Dependencies

This code does not compile. We provide responses & hints related to [missing code here](missing_code.md). At the same time, most of the header include statments can be taken care of by bringing in some other projects to your dev environment.


### Dependency: FMACM

This code is missing a lot of include files that come from a related MITRE project: [FIM MOPS Aircraft Dynamics & Control Model](https://mitre.github.io/FMACM). You should definitely get that project into your dev environment, make sure it's compiling and passing the unit tests, then connect it to this code base. 

### Dependency: EUROCONTROL BADA

This code uses [EUROCONTROL's BADA v3.x](https://eurocontrol.int/services/bada) for aircraft performance data. However, our BADA functionality and code cannot be provided due to licensing restrictions imposed by EUROCONTROL. 

Do you need BADA implemented to use this code base? **Nope**. Take a look at [the BADA usage guide](bada_usage_guide.md) to get more details.

As an alternative to BADA, one may consider using the open-source [WRAP](https://github.com/junzis/wrap) research product. However no attempt has been made by MITRE to use that project. Let us know how it goes! :four_leaf_clover:

### Dependency: Log4Cplus

Log4Cplus is a logging application used by this code base. It needs to be installed prior to building this code. You can download it from [their GitHub repo](https://github.com/log4cplus/log4cplus).

### Dependency: Units of Measure Library

[Units of Measure Library](http://sourceforge.net/projects/tuoml/) is an open source (LGPLv2), NIST-compliant, C++ library for handling scientific units of measure. It provides abstract object types, mathematical operations, and compile-time checking of uses. It is used extensively in this code base in order to remove ambiguity and reduce the potential of UoM-related bugs.

## Continuous Integration & Testing

MITRE takes quality seriously. But, this repository houses code that is not expected to compile. We will do our best to provide unit testing and CI via travis-ci.

### Compile & Run

Disappointment time...this code does not produce a self-contained binary executable nor does it necessarily compile. :disappointed: Rather this repository represents a collection of implemented algorithms provided as a supplement to DO-361A. No attempt has been made to ensure that this compiles. Where possible, examine the unit tests for an understanding of which sections of the code are fully operational. Otherwise, please rely upon the documentation and post issues with questions or conerns.

No attempt has been made to ensure that this code will compile on a range of operating systems. For MITRE, this code compiles successfully on Linux machines, specifically [CentOs](https://www.centos.org/) 7 using gcc 4.8.5. For all other computing environments, YMMV.

The [CMake](https://cmake.org/) utility is used to compile this code. If not already installed on the target environment, it is easily installed via `apt`. Please use version 3.0+.

### Run Unit Tests

We hope to provide these in time.
