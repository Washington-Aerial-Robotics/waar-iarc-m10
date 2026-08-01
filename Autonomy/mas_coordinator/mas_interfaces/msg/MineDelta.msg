# MineDelta.msg
# Carries one or more MineBelief updates during a sync window.
# Receiver deduplicates by (mine_id, seq): skip if local seq >= incoming seq.

string        sender_id
MineBelief[]  beliefs     # batch of updates (usually 1–10 per sync)
uint8         ttl         # hops remaining; sender sets 2, receiver decrements before relay
builtin_interfaces/Time stamp
