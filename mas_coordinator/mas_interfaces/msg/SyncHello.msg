# SyncHello.msg
# Initiates a sync window with a neighbor (Scheme B)

string sender_id
string target_id        # intended recipient; others ignore
uint32 known_mine_count # how many mines sender currently knows (for delta negotiation)
builtin_interfaces/Time stamp
