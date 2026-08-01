# TaskAnnounce.msg
# Any drone can announce a task to be auctioned

string  task_id         # unique: "<announcer_id>_<seq>"
string  task_type       # "VERIFY_TAG" | "RESCAN" | "BECOME_PATH_VERIFIER" | "BECOME_VERIFIER"
string  announcer_id
float64 target_x        # world frame target (if applicable)
float64 target_y
float32 priority        # higher = more urgent [0.0, 1.0]
float32 claim_window_s  # seconds bidders have to respond (e.g. 2.0)
builtin_interfaces/Time stamp
