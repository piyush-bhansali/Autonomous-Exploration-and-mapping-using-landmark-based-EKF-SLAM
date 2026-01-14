# Message for broadcasting robot information to other robots

string robot_id                  # Unique robot identifier (e.g., "tb3_1")
float64 x                        # X position in reference frame
float64 y                        # Y position in reference frame
float64 theta                    # Heading angle (radians)
float64[4] quaternion            # Orientation [qx, qy, qz, qw]
string reference_frame_id        # ID of the reference frame this robot uses
uint32 num_submaps               # Number of submaps created
builtin_interfaces/Time timestamp  # Message timestamp
