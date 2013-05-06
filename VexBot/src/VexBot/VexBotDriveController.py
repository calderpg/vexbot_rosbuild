#   Calder Phillips-Grafflin
#
#   General tank-like skid steering drive controller (converts velocities to motor commands)
#   Currently does not account for dynamics!

﻿class VexBotTankDrive:
    """
    Implements a basic tank-like drive system with two tracks or four wheels controlled
    by two motors (left = motor 0, right = motor 1). Provides all basic functionality
    including forward/reverse, turning in place, and differential steering.
    """
    def __init__(self, max_linear, max_angular, max_total):
        #You will need to empirically determine these numbers on your own
        self.max_x_velocity = max_linear    #Maximum X (linear) velocity in M/S
        self.max_z_rotational = max_angular    #Maximum Z (rotation) in radians/S
        self.max_vehicle_velocity = max_total #Maximum vehicle velocity in M/S (probably the same as the maximum X velocity)
    
    def Compute(self, X_velocity, Z_rotational):
        """
        Arguments:
        
        X_velocity (M/S), Z_rotational (Radians/S)

        Returns:

        [M0, M1], all in the range between -1.0 and 1.0 (fraction of maximum motor power)
        
        This function does the critical step of converting a velocity vector into float wheel speed commands
        """
        # While this is hardly the most elegant way to do it, for readability and
        # understandability, the 4 different cases will be treated separately here
        if (X_velocity == 0.0 and Z_rotational == 0.0):
            # This is a trivial case where we're stopped
            return [0.0, 0.0]
        
        elif (X_velocity != 0.0 and Z_rotational == 0.0):
            # When we're going straight forwards/backwards
            fraction_of_max = abs(X_velocity / self.max_x_velocity)
            if fraction_of_max > 1.0:
                fraction_of_max = 1.0
            if X_velocity > 0.0:
                return [fraction_of_max, fraction_of_max]
            elif X_velocity < 0.0:
                return [-fraction_of_max, -fraction_of_max]
                
        elif (X_velocity == 0.0 and Z_rotational != 0.0):
            # When we're only turning in place
            fraction_of_max = abs(Z_rotational / self.max_z_rotational)
            if fraction_of_max > 1.0:
                fraction_of_max = 1.0
            if Z_rotational > 0.0:
                return [-fraction_of_max, fraction_of_max]
            elif Z_rotational < 0.0:
                return [fraction_of_max, -fraction_of_max]
            
        elif (X_velocity != 0.0 and Z_rotational != 0.0):
            # When we're going forwards/backwards and turning (moving like a car)
            # multiply inside motor commands by (1 - percentage of max rotational)
            fraction_of_max_x = abs(X_velocity / self.max_x_velocity)
            if fraction_of_max_x > 1.0:
                fraction_of_max_x = 1.0
                
            fraction_of_max_z = abs(Z_rotational / self.max_z_rotational)
            if fraction_of_max_z > 1.0:
                fraction_of_max_z = 1.0
                
            if X_velocity > 0.0 and Z_rotational > 0.0:
                return [(fraction_of_max_x * (1 - fraction_of_max_z)), fraction_of_max_x]
            elif X_velocity < 0.0 and Z_rotational > 0.0:
                return [-(fraction_of_max_x * (1 - fraction_of_max_z)), -fraction_of_max_x]
            elif X_velocity > 0.0 and Z_rotational < 0.0:
                return [fraction_of_max_x, (fraction_of_max_x * (1 - fraction_of_max_z))]
            elif X_velocity < 0.0 and Z_rotational < 0.0:
                return [-fraction_of_max_x, -(fraction_of_max_x * (1 - fraction_of_max_z))]
