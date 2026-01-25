builtin_interfaces/Time io_stamp
int32 data_direction
uint8[] data

# Enum constants for the direction

# Uninitialized, error, etc.
int32 DIRECTION_UNKNOWN=0

# This is a packet set from the node out through the connection
# via a "send" call on the connection.
int32 DIRECTION_DATA_OUT=1

# This is a message sent to the node from the connection.
# It should trigger a callback in the node.
int32 DIRECTION_DATA_IN=2