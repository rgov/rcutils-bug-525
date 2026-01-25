# The command's state machine ID.  Managed by the state machine
uint64 id

# The command to send
string command

# Send the received buffer out on matching
bool emit_on_match

# Send a warning on timeout
bool timeout_warn

# Force the next message to come from the same queue.
# Useful when you need to run two commands (e.g., address + command)
bool force_next

# Send an error message on state transition guard condition check
bool state_trans_err

# Delay before sending the command, in milliseconds
float32 delay_before_ms

# Delay After receiving a reply, in milliseconds
float32 delay_after_ms

# The maximum amount of time (in milliseconds) to wait for
# a reply.  A timeout <= 0 means "wait forever"
float32 timeout_ms