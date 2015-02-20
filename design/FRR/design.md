# AGSE 2015 FRR Design Document
-------------------------------

* Model Driven design and development : iterative
* Using ROS
  * provides networking infrastructure and inter-process communications
  * lightweight
  * can deploy on heterogeneous sytems
* Developed a ROS Component Model
  * why is this good
  * what is this useful for?
  * How do we use it
  * what has it done to our development cycle and development time?
* Developed a ROS Modeling Language and code generators
  * what is it?
  * Why do we use it?
  * how did we develop it?
  * how do we use it?
  * How the code generators work and how they DRASTICALLY shorten iterative development cycle
* Developed a graphical editor for design, generation, deployment, and monitoring
  * how did we do it : what were the stages
  * how does it affect our development cycle : makes iterative development have shorter cycles and easier to do
  * what is the deployment useful for
  * what is the monitoring useful for

* Iterative Development Progress:
  * Started with prototype (AGSE v1):
    * AGSE system (shown in CDR)
      * Hardware
      	* Code:
	  * Prototyped in python
	  * Grabbed sample from known (pre-programmed) position and orientation
	  * Not very maintainable
	  * Camera & object detection code was not integrated
	  * Code was split among jetson and arduino
	  * Serial communications issues (servo & arduino) and noise issues (servo pwm)
      	* Base:
	  * Unstable off the shelf turntable
	  * Bad servo control
	  * Horrible power consumption (~300 mA @ 12 V while doing literally nothing, not even hodling pos)
	* Linear Actuators:
	  * Misaligned mounting led to some jamming of horizontal actuator
	  * Waste space and weight (w.r.t. required power and throw)
	* Gripper:
	  * Flimsy modification of off the shelf gripper; not enough force
	  * two different servos
	  * Analog (PWM) servo had issues with noise picked up by long cable from control board
	  * Small servo couldn't be daisychained with larger (base rotation) servo
	  * Small servo is not as well documented and not as well supported
      	* Breadboard circuits for H-Bridges and serial buffers
	  * A few iterations of these circuits
	  * Off the shelf h-bridges we had on hand were not powerful enough (magic blue smoke)
	  * many different voltage levels required ( 5V, 12V, 1.8V, 7.2V )
	* Issues with the servos
	  * couldn't be daisychained
	  * Datasheets provided by vendors were wrong and contradictory
	  * libraries provided by vendors were quite buggy (took a while to figure out)
	* Off the shelf linear actuators:
	  * analog feedback (was an issue for our board)
	  * Current draw
	  * Mounting
	* Jetson
	  * Could run the software
	  * Couldn't interface with all of the hardware (not enough free GPIO)
	* Overall:
	  * Showed that we could get the system up and running
	  * didn't have full camera integration
	  * didn't have good feedback from the motors
	  * met the requirements set out in NASA's RFP
	    * picked up object in known orientation/position and placed it in payload bay of known orientation/position
	  * had issues with jamming, noise, and power consumption
	  * Code wasn't very maintainable and could not be distributed among nodes
	  * Control code was split between arduino and jetson without clear delineation of responsibilities
      * Software
    * Modeling Language / Code Generators
      * Component model and ROS concepts modeled (msg/srv/timer/pub/sub/client/server, component, nodes)
      * Code generated for build system, component model, and workspace
      * No concepts of hardware or deployment
      * No code preservation -> made iterative development more difficult
      * Useful for designing the system and describing the interactions of the system components
    * Editor
      * Could describe system software and use the generator to create the code
      * Used for testing out the modeling of the AGSE system and generating code
      * No code deployment or hardware description/assignment
  * Second Phase (AGSE v2):
    * New version of AGSE:
      * Hardware
      * Software
    * Modeling Language / Code Generators
    * Editor with deployment