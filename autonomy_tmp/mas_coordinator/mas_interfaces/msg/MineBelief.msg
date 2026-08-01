# MineBelief.msg
# Represents a single mine candidate belief held by a drone

string  mine_id         # unique ID, e.g. "m_<drone_id>_<seq>"
float64 x               # arena frame, metres
float64 y               # arena frame, metres
float32 confidence      # [0.0, 1.0]
string  status          # "candidate" | "confirmed" | "rejected" | "uncertain"
string  last_updated_by # drone ID that last updated this belief
uint32  seq             # monotonic sequence number for dedup / merge
builtin_interfaces/Time stamp
