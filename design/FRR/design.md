# AGSE 2015 FRR Design Document
-------------------------------

## Design Choice Reasoning:
---------------------------
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

## Development Status:
----------------------
* Iterative Development Progress:
  * Started with prototype (AGSE v1):
    * AGSE system (shown in CDR)
      * Hardware
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
          * Couldn't interface with all of the hardware
            * not enough free GPIO for user input switches
            * no analog input for motor potentiometer feedback
            * no encoder input for motor encoders (if we wanted)
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
        * Prototyped in python
        * Grabbed sample from known (pre-programmed) position and orientation
        * Not very maintainable
        * Camera & object detection code was not integrated
        * Code was split among jetson and arduino
        * Serial communications issues (servo & arduino) and noise issues (servo pwm)
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
        * Base:
          * Custom-built turn-table / rotation control
          * Stable base with very small positioning error
        * Unification of system servos
          * same protocol
          * same voltage
          * daisychaining for wire management
          * mounted well to sturdy platform
        * Linear Actuators:
          * Custom built
          * Reusing design principles and lessons from payload bay
          * Using encoders for positioning feedback
          * Same voltage level as servos
          * Lower power
          * No wasted space (i.e. volume and mass go down, work area goes up)
        * Gripper:
          * Same servos as base (voltage / protocol)
          * Higher torque for better gripping
          * Better design for phalanges (w.r.t. consistently grabbing sample)
          * Integration of camera
        * Circuitry:
          * H-Bridges for two linear actuators
          * Serial buffer for full-duplex uart @ 3.3V to half-duplex uart @ 5V
          * Resistor dividers for 5V QEP to 3.3V eQEP hardware
          * Power filtering and step-down from 12V ( Motors / Jetson ) to 5V (BBB & encoders)
          * Separate power conversion and isolation board for batteries (48V) to main system power (12V)
        * Jetson:
          * Runs the image processing / object detection
          * Runs the overall high-level system planning
          * Runs ROSCORE discovery mechanism and ROS backbone
          * connects using USB to BBB (linux usb/ethernet driver for BBB)
        * BeagleBoneBlack (BBB):
          * Many GPIO pins for motor control (to two H-bridges)
          * Servo control through serial buffer
          * eQEP (enhanced Quadrature Encoded Pulse) hardware integrated into processor for motor position feedback
          * usb camera connected to take images : sends the images to the jetson
          * connected to jetson using TCP
        * Overall:
          * More space efficient
          * More power efficient
          * Better materials and better construction
          * unification and simplification of system hardware and software components
          * integration of image processing and camera for autonomous sample detection and retrieval from anywhere within the workspace of the AGSE
          * Software
            * ROS based
          * provides communications middlware between processes on the same node or on different nodes
          * Abstraction above TCP & socket level transfers
    * Component Based Software (Our component model and implementation for ROS, ROSMOD)
      * threading model
      * interaction pattern model
      * separation of concerns between different subsystems
      * Scheduling model (timer and event based scheduling)
    * Downgraded linux kernel version to support simple device tree overlay for switching GPIO mode and enabling subsystems of the processsor 
      * useful for the configuration of the hardware based quadrature encoder decoding (eQEP)
    * Integrated all main subystem code into generated c++ ROS / ROSMOD code
      * supports reading linear actuator position through eQEP kernel driver
      * supports controlling linear actuators through GPIO
      * supports reading system switch states for propagation to rest of system
      * supports reading and writing to servo motors for control of the base rotation, gripper rotation, and gripper position
      * supports image processing code and camera image gathering
      * supports communications between system components to achieve overall system goals
    * Modeling Language / Code Generators
      * support for describing the hardware configuration of the system
        * host names
        * host ip addresses
        * host architecture
        * host login information
        * host hardware configuration scripting
      * support for describing deployment configuration for a specific hardware and software configuration
        * ROS Node (process) to Host (hardware) mapping with process identifier (name)
      * Code generators preserve business logic 
        * developer generates baseline code from model -> adds some business logic code to function stubs -> changes model -> regenerates base code; business logic from before is preserved
        * shortens iterative development time
        * increases utility and decreases user or system errors
    * Editor with deployment
      * specify system software model (component based)
      * specify hardware configuration and hosts' attributes
      * specify software to hardware mapping
      * Deploy the software onto the system
      * monitor status of deployed software