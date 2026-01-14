# Message for notifying frame transformation updates

string from_frame              # Original frame ID
string to_frame                # New reference frame ID
float64[16] transform_matrix   # Flattened 4x4 transformation matrix
builtin_interfaces/Time timestamp
