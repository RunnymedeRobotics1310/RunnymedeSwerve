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

- drive (m/s & deg/s) - done
- toRobotOriented (any & deg) - done
- calculateModuleVelocities (power & power) - done
-
- computeVelocityScaleFactor (power & power) - done
- discretize (power & power) - done
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

- drive (rad/s -> deg/s) - done
- toRobotOriented (rad -> deg) - done
- V vars in core (m/s & rad/s -> power & power) - done
- kinematics.calculateModuleVelocities (m/s & rad/s -> power & power) - done
- discretize (m/s & rad/s -> power & power) (this is just sfs) - needs tunign
- module directives & down (deg -> rad)