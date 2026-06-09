# One localized bloom inside a planter box, produced by lupin_perception's
# perception_aggregator (tag-anchored box geometry + a lateral cue). Carried
# as a repeated field on FlowerObservation and TwinTagState so the HMI can
# draw scattered, species-coloured dots inside the box rectangle instead of
# stacking every flower on the tag pin.

geometry_msgs/Point position   # map frame; z is estimated from tag/planter geometry

# Estimated vertical extent of the bloom/stem from known planter geometry.
# 0.0 when no height estimate was available.
float32 height_m

# YOLO class: "tulip_red" | "tulip_white" | "tulip_pink". May be "" when a
# column held only the "bug" anomaly class (anomaly True, no tulip species).
string species

float32 confidence             # [0, 1] for `species`

bool anomaly                   # the "bug" class co-located in this column
