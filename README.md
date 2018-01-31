# xplane
X-Plane connector for Matlab

Unlike many other simulators, X-Plane calculates aerodynamics for your real airplane geometry rather than uses a pre-defined empirical model. This advantage makes it the sim of first choice if you are desighnin your own vehicle. It worth mentioning that X-Plane allows full control (well, almost full) via UDP.

Unfortunately, the protocol gets significantly changed from one version of XPlane to another, and old good connectors don't work anymore with the current v.11. That is why we desided to made our own. 

Unlike the NASA's connector [2] it uses a callback to update the plane's current state automatically, so no need to poll. It also uses organic Matlab and should work on any platform (albeit tested on Mac OS only).

The connector is released under MIT licence - do whatever you want. If you found it useful, we appreciate referring to the auPilot project. 

###### Kirill Makukhin
###### auPilot project
###### http://aupilot.com.au


## Usefull links:

[1] X-Plane simulator: http://www.x-plane.com

[2] NASA connector: https://github.com/nasa/XPlaneConnect

[3] list of commands: http://siminnovations.com/xplane/command/index.php

[4] list of datarefs: http://siminnovations.com/xplane/dataref/index.php

[5] XPlane specific quaternions representation: http://www.xpluginsdk.org/downloads/TestQuaternions.cpp
