# TaskClaim.msg
# A drone bids on an announced task

string  task_id
string  bidder_id
float32 cost            # lower = more willing/capable (e.g. distance to target)
builtin_interfaces/Time stamp
