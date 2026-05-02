# Periodic snapshot of every tag the digital-twin node has observed so far.
#
# Published at ~1 Hz on /twin/state with RELIABLE + TRANSIENT_LOCAL depth 1
# (latched), so a late-connecting HMI immediately sees the current world
# without a bootstrap call. Empty `tags` is the legitimate startup state —
# nothing observed yet.

std_msgs/Header header
TwinTagState[] tags
