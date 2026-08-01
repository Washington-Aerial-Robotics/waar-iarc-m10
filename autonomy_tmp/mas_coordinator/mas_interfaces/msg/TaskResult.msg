# TaskResult.msg
# Published by the winner when the task is complete

string  task_id
string  executor_id
string  outcome         # "confirmed" | "rejected" | "uncertain" | "failed"
string  mine_id         # which mine was verified (if applicable)
float32 confidence      # updated confidence after verification
builtin_interfaces/Time stamp
