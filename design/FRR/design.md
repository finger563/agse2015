# AGSE 2015 FRR Design Document
-------------------------------

## Need to describe:
--------------------
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

## Design Choice Reasoning:
---------------------------
For the development of the hardware and software components of the AGSE sample capture system, we followed a model-driven iterative design and development.  This process ensured that at every step of the development, we had complete models of the system we were building (with respect to both hardware and software) as well as evaluations regarding the performance of the system and its effectiveness at meeting our design criteria.  Furthermore, by developing the system in a component-based fashion, we were able to iteratively address design concerns and improve subsystems of the AGSE without having to completely rebuild the entire AGSE.  

Following this process, we developed the following subsystems of the AGSE for its first prototype (demoed in a video for CDR): 
 * vertical linear actuator and carriage
 * horizontal / radial linear actuator and carriage
 * base servo motor for AGSE rotation with turntable and mounting plate
 * gripper phalanges 
 * gripper wrist servo motor for gripper rotation
 * gripper actuation servo motor for gripper opening and closing
 * image processing and object detection software component for sample detection
 * linear actuator software control
 * base servo software control
 * gripper wrist software control
 * gripper actuation software control

Having already progressed further in the development cycle for the payload bay, we had developed a custom complete hardware solution for a linear actuator which provided us with a more robust and efficient solution than using an off the shelf linear actuator.  From the development of that system, we had determined not only that the custom linear actuator and drivetrain would be better for the AGSE with respect to weight, power, and size requirements, but also that we were capable of developing such a system in a relatively quick time-frame.

However, the AGSE system is more complex than the payload bay since it requires one entire linear actuator and carriage plate / guide rod system to be mounted to another linear actuator and carriage plate / guide rod system to provide the horizontal and vertical axes translation.  Additionally, the system composed of these two linear actuators must be mounted to the top of a rotating base to provide us with the cylindrical workspace we determined would be best for the AGSE system.  All of these design complications led us to the conclusion that we should first build a prototype AGSE system using off the shelf linear actuators and grippers, so that we could ensure that the integration of these systems would work before spending the time and money on the development of the custom system.

Because we knew this system to be the prototype, and not necessarily the final electrical components that we would be using in the final system, we split our software and electrical development into several parallel efforts, which were relatively independent of each other:
  * _Development of the ROS Component Model, Modeling Language, Toolsuite, and Code Generators_:
    * We developed a complete component model and implementation (in c++) for the _Robot Operating System_ (ROS) middleware that removes the need for synchronization primitives such as mutexes, semaphores, message queues, and condition variables in our multi-thread, multi-process, and multi-node component-based code.  This component model allows us to specify timers which invoke certain operations which are enqueued into the component's operation queue.  All services provided by this component or data subscriptions this component has must go through this operation queue.  What this means is that any client's request of a service provided by this component or any data published on a topic to which the component subscribes is enqueued as an operation in the operation queue.  Using this queue, we ensure that (1) all components may only be executing one operation at a time and (2) no component's operation can be interrupted by another operation of the same component.
    * We also developed a modeling language (written using ANTLR) and toolsuite (written in python) which allows us to specify the software components of the system, their interactions, the datatypes they use, and their mapping to nodes in the system.  This modeling language allows us to ensure (1) the correctness of the software configuration, (2) the correctness of the component interactions, and (3) the reliability of the generated software.  The reliability of the software is ensured though the use of our custom code-generators which generate the infrastructural code to facilitate the component creation, scheduling, and middleware interfacing which is directly specified in the model or can be determined from the model.  For this reason, we chose to use the ROS middleware since it was lightweight (both size and effect on performance) and since it was relatively easy to develop a component model and code generators for.  The toolsuite we developed allows us (the developers) to create the software models in a visual fashion (rather than using text editors) and in this way to inspect the models visually to get a more comprehensive understanding of the components in the system and their interactions.  This visualization has the added benefit of providing an easy communications mechanism between members of the team, since it is easy for them to understand how the software of the system is designed based on this model representation.
    * Because the code generators (written in python using cheetah templates) generate most of the code which runs our system, we are able to not only cut down on the time it took to develop the code, but also ensure the correctness of the generated code since much of the generated code is similar or duplicated code between software components.  The generated code handles the following functions:
      * Sets up the application's processes and each process' components
      * For each component it configures:
        * Any data types (topics) on which the component publishes (sends) to other components
        * Any data types (topics) on which the component subscribes (receives) from other components
        * Any methods (functions) which the component provides as a service (server) to other components
        * Any methods (functions) which the component requires as a service (client) from other components
        * The initialization routine of the component
        * Any timers required for the component
      * Registering the publishers/subscribers and clients/servers with the ROS discovery and connection infrastructure, ROSCORE
    With all of this generated code, all that we had to write for the AGSE system (after writing the generators and modeling tools of course) was the actual business logic for any of the callbacks we needed.  Such callbacks include what happens when each component receives the pause command, or what happens when the goal position is updated for the horizontal/radial actuator controller component.  By only having this kind of purely mission goal-oriented code to write and modularizing it in such a way so as to keep it separated from the rest of the infrastructure code, we keep the mission goal code simpler, easier to read, easier to write, and easier to debug.  
  * _Development of the AGSE Prototype Control Software_:
    * The AGSE prototype control software had to serve the following functions: 
      * control the dynamixel xl320 servo for the gripper phalanges actuation
      * control the gripper wrist rotation servo
      * control the LAC-12 horizontal linear actuator
      * control the LAC-12 vertical linear actuator
      * control the dynamixel MX-28T base rotation / turntable servo
  * _Development of the AGSE Prototype Control Circuitry_:
    * The AGSE prototype control circuitry functioned to facilitate communication and control of all the AGSE hardware.  To perform this functionality, we required the following sub-circuits:
      * An H-Bridge which could handle 3-5A @ 12V for the vertical linear actuator.  This H-Bridge, shown in its relevant schematic, consists of 2 PFETs and 2 NFETs, (FQP27P06 and FQP30N60L, respectively), which form the 12V H-Bridge for the motor, as well as 2 NPN transistors (2N2222) which allow the Jetson to control the motor's power and direction.  
      * An H-Bridge which could handle 3-5A @ 12V for the horizontal linear actuator, using the same circuit design as described above for the vertical linear actuator.  
      * A buffer circuit to allow serial communication between the full-duplex UART on the Jetson TK1, running at 1.8V, and the half-duplex UART on the dynamixel xl320 servo, running at 5V.  This circuit, which is shown in its relevant circuit schematic, consists of 2 NPN transistors (2N3904) connected in a buffer configuration, running off a VDD of 7.2V (the xl320's power supply).  The input to the buffer is the Jetson's UART TX pin, and the output of the circuit is connected to both the servo's combined TX/RX pin, as well as to the Jetson's UART RX pin, through a resistor divider to bring the voltage from the 5V of the motor to the 1.8V of the Jetson.
  * _Development of the Final AGSE Image Processing and Object Detection Software_:
    * PRANAV YOU SHOULD WRITE HERE ABOUT WHAT YOU DID
  * _Development of the Final AGSE Control Circuitry_:
    * Having determined that the H-Bridges we developed were sufficient for the prototype's off the shelf motors, which require more power than the final AGSE motors (but are the same voltage), we decided to use the same H-Bridge circuits for the final AGSE.  Similarly, we use the same serial buffer circuit in the final AGSE as we used in the prototype AGSE, however we bias it to the voltage of the new servo motors (the dynamixel AX-12A) which run at 12 V.  Finally, we used an off-the shelf switched mode power supply (SMPS) DC/DC converter and isolation circuit to step the battery voltage down to the main 12V power required for most of the systems components.  This 12V supply powers the Jetson, the Linear Actuators, and all the servo motors.  The linear actuator encoders and the BeagleBone Blacks all run off of 5V, for which use use a DC/DC SMPS to step the 12V down to 5V.  

This prototype design process yielded beneficial information which were able to incorporate into the final AGSE design:
  * The servo motor datasheets, design documents, and sample code (from manufacturer) were incorrect
  * The dynamixel servo motors could not be daisychained as we had hoped, as they ran on different voltage levels (and used different communications protocols)
  * The servo motor controlling the base rotation (turntable) consumed too much current during standby (300 mA at 12V)
  * The servo motor controlling the gripper rotation, since it was controlled simply by using PWM, was susceptible to electrical noise. this caused jittering and some hysteresis in the gripper orientation
  * The linear actuators chosen for off the shelf consumed a lot of current, wasted space (i.e. unextended they took up more space than we wanted to use), did not have a long enough throw, and provided only analog feedback about position
  * The linear actuator's analog feedback could not be used by our main processor board (the Jetson TK1) since it had no on-board analog to digital converters (ADC).  
  * We were able to simultaneously develop the image processing and sample detection code for the final AGSE
  * We were able to simultaneously develop the hardware/software modeling language, development toolsuite, code generators, and code for the final system, since many of these lessons (especially with respect to the servo and linear actuator complications) were encountered early in the prototype's development

## Tests Performed:
-------------------

__DESCRIBE HERE THE TESTS THAT WE DID FOR__:
* Prototype hardware : xl320, mx28T, lac12, gripper, camera, circuits
* Prototype software : servo control, serial buffer, h-bridge control, analog feedback, NASA requirements (point to point)
* Modeling language
* Model Editor
* Code Generators
* ROS Code

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
