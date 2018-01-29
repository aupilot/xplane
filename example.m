clear X; 

% create an instance and connect to X-Plane
% make sure you've changed your local computer and X-Plane computer addresses
X = xplane(); 

% init view, controls etc
X.init; 

% Request X-Plane to send out plane position and other datarefs. A parser callback (dataReceivedCallBack)
% will be executed with each datagram received. To stop this - clear X 
X.request;

% print the current state (updated automatically 20 times per second)
X

% thrust vector UP for 2 first engines
X.setVector([90,90])
 
% Go!
X.setThrottle([0.99 0.99 0.7])

%
% X.setApproach('KSEA', [0.4 0.4 0.02], [0 0])
