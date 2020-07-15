# The Context

[Back to Landing Page](/README.md)

In order to understand this code, what it can (or can't) do, and why it has been publicly released there needs to be a discussion of the context in which this code came about. 

## What Is This Code?

This code base is the Sample Algorithm developed & used for research and minimum requirements validation of the Flight-deck Interval Management concept. The code has been developed & used extensively for over a decade to examine technical questions that occurred during the writing of [RTCA DO-360 and DO-361A](https://my.rtca.org/nc__store?search=do-361a). 

The code is considered a mature prototype which takes into account as many of the minimum requirements as possible. In order to be used for validation of the minimum requirements, the prototype also intentionally attempts to _not exceed_ the minimum requirements. This means that we recognize that further optimization & maturation of the behavior (software or control-law-response perspective) could be undertake, but that work is left for others. 

The Sample Algorithm is made available as a reference implementation of these requirements and it is freely available (via the Apache 2.0 license agreement) for examination and reuse.

From a software perspective, this code base does not represent a self-contained executable. It should probably be compiled as a library and linked with other software that provides the rest of your simulation environment.

## How MITRE Has Used This Code

MITRE has hosted this Sample Algorithm in two different simulation paradigms, depending on the kind of research being performed. In each case, the algorithm is a library linked into a larger simulation environment. And the two environments are very different, which means the Sample Algorithm has been well exercised (from a software perspective and a control-law-behavior perspective) and shows similar performance in both paradigms. When key Interval Management metrics are compared between the two simulation paradigms, we find similar reponses from the Sample Algorithm.

### Parametric Simulations

The first paradigm is a fast-time, parametric simulation environment developed to implement and debug the initial algorithm concept. This simulation is capable of thousands of runs (in a short wall-time sense), sweeping the many inputs while adhering to [technically sound physics principles](https://www.mitre.org/publications/technical-papers/derivation-of-a-point-mass-aircraft-model-used-for-fast-time) in order to generate quality results appropriate to the studies. This simulation has been used, among other activities, to validate operational concepts supporting [RTCA DO-328B](https://my.rtca.org/nc__store?search=do-328b), explore [alternate algorithm variations](https://arc.aiaa.org/doi/abs/10.2514/6.2014-3149), and mature the final copy of the Sample Algorithm as published in [RTCA DO-361A](https://my.rtca.org/nc__store?search=do-361a). A simple architecture diagram of this parametric simulation environment follows.

![cartoon graphic of parametric simulation architecture](parametric_simulation_cartoon.png)


### Human-In-The-Loop Simulations

The second simulation paradigm is MITRE's [Aviation IDEA Lab](https://www.mitre.org/publications/project-stories/mitre-experimentation-lab-gives-wings-to-aviation-technology-research), a real-time simulation environment used to study humans and their behavior as they interact with new concepts in aviation technology. In this environment, the Sample Algorithm is one small cog in a large infrastucture of software and hardware. The only thing this simulation environment holds in common with the parametric environment is the Sample Algorithm library. All other simulation elements, including aircraft state propagation models, are completely different. 

![cartoon graphic of hitl simulation architecture](hitl_simulation_cartoon.png)

## How External Collaborators Can Use This Code

In order to use this Sample Algorithm code, you'll need to provide your own simulation environment that hosts the Sample Algorithm. This code base does not propagate aircraft state nor does it provide equations-of-motion, so your simulation will in the very least need a way to produce states for ownship and traffic aircraft and dynamically respond to the speed guidance coming from the Sample Algorithm. One possibility is to use MITRE's parametric simulation core, made available as [FMACM](https://github.com/mitre/FMACM). But use of FMACM is not required; you can use any simulation paradigm that is appropriate to your research needs. 