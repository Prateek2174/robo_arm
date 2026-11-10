# can protocol

Human-readable version of arm.dbc.

Control rate: 100 Hz

    Meets all requirements:
        Current, velocity and position: these run on the joint, so CAN only carries setpoints.
        Door compliance: impedance mode (#9) also runs on the joint, so CAN just sets the stiffness.
        Visual servoing: the camera runs at about 30 Hz, and 100 Hz is comfortably faster than that.
        Nav2 and diff_drive: these typically run at 20–50 Hz.

0010 is a winding current feedback signal from a motor
0011 is the tachometer speed

messages relating to current values always have a higher priority on the bus than those concerned with tachometer readings

Message	            Sender → receiver	    Rate	    Purpose

E-stop	            power node / PC → all	on event	everyone goes to safe-torque-off
Fault report	    joint → PC	            on event	which fault, plus a value
Command	            PC → joint	            100 Hz	    setpoints
Fast telemetry	    joint → PC	            100 Hz	    what ros2_control reads
State request	    PC → joint	            on event	arm, disarm, calibrate, clear fault
Heartbeat	        every node	            10 Hz	    “I’m alive” plus version
Slow status	        joint → PC	            10 Hz	    bus voltage, temperature
Parameter get/set	PC ↔ joint	            on demand	gains, limits, node ID
