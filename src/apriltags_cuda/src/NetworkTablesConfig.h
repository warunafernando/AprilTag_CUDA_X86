#ifndef NETWORK_TABLES_CONFIG_H_
#define NETWORK_TABLES_CONFIG_H_

// Default to localhost for local GUI communication
// Can be overridden by setting environment variable NT_SERVER_ADDRESS
inline const char *TABLE_ADDRESS = "localhost";
inline const char *TABLE_NAME = "/SmartDashboard";

#endif
