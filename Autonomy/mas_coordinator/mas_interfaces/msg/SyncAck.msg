# SyncAck.msg
# Acknowledges a SyncHello, completing the handshake

string sender_id
string target_id
uint32 known_mine_count
builtin_interfaces/Time stamp
