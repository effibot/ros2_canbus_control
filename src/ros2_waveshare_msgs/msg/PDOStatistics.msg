# PDO Manager performance statistics
# Frequency: ~1 Hz (diagnostics)

std_msgs/Header header

# Counters (cumulative since node start)
uint64 tpdo1_received                   # Total TPDO1 messages received
uint64 tpdo2_received                   # Total TPDO2 messages received
uint64 tpdo3_received                   # Total TPDO3 messages received
uint64 tpdo4_received                   # Total TPDO4 messages received
uint64 rpdo1_sent                       # Total RPDO1 messages sent
uint64 rpdo2_sent                       # Total RPDO2 messages sent
uint64 rpdo3_sent                       # Total RPDO3 messages sent
uint64 rpdo4_sent                       # Total RPDO4 messages sent
uint64 sync_sent                        # Total SYNC messages sent
uint64 errors                           # Total communication errors

# Performance metrics
float64 avg_latency_us                  # Average PDO latency in microseconds
uint64 latency_samples                  # Number of latency samples

# Rates (calculated over last second)
float64 tpdo1_rate_hz                   # TPDO1 reception rate
float64 tpdo2_rate_hz                   # TPDO2 reception rate
float64 rpdo1_rate_hz                   # RPDO1 send rate
float64 sync_rate_hz                    # SYNC send rate
