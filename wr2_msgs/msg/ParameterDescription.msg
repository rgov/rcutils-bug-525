# A list of parameters that can be runtime-reconfigured
# Specifically designed to be used by the ParameterServer and ParameterClient classes

string node_name
string server_namespace
string[] parameter_names  # Relative to the node, not the server namespace
