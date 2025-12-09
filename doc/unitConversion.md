# Units Log

- drive (m/s & rad/s)
- toRobotOriented (any & rad)
- calculateModuleVelocities (m/s & rad/s)
- computeVelocityScaleFactor (power & power)
- discretize (m/s & rad/s)
- calculateModuleVelocities (power & power) -> (power & rad)
- save module directives (m/s & deg)
- setDesiredState (m/s & deg)
- optimizeWheelAngles (deg)
- cosineCompensator (rad)
- setReferenceVelocity (m/s)
- setReferenceAngle (deg)
-
- odometry (rad & (m & Rotation2d))

# Math Optimized

- drive (power & power)
- toRobotOriented (any & power)
- calculateModuleVelocities (power & power)
- computeVelocityScaleFactor (power & power)
- discretize (power & power)
- calculateModuleVelocities (power & power) -> (power & rad)
- save module directives (power & rad)
- setDesiredState (power & rad)
- optimizeWheelAngles (rad)
- cosineCompensator (rad)
- setReferenceVelocity (power)
- setReferenceAngle (rad)
-
- odometry (rad & (m & Rotation2d))

# Better Idea

- drive (m/s & deg/s)
- toRobotOriented (any & deg)
- calculateModuleVelocities (power & power)
-
- computeVelocityScaleFactor (power & power)
- discretize (power & power)
- calculateModuleVelocities (power & power) -> (power & rad)
- save module directives (m/s & rad)
- setDesiredState (m/s & rad)
- optimizeWheelAngles (rad)
- cosineCompensator (rad)
- setReferenceVelocity (m/s)
- setReferenceAngle (rad)
-
- odometry (rad & (m & Rotation2d))

# What Changed

- drive (rad/s -> deg/s)
- toRobotOriented (rad -> deg)
- V vars in core (m/s & rad/s -> power & power)
- kinematics.calculateModuleVelocities (m/s & rad/s -> power & power)
- discretize (m/s & rad/s -> power & power) (this is just sfs)
- module directives & down (deg -> rad)