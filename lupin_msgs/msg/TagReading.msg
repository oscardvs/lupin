# Bundled sensor readings observed at a single greenhouse tag.

string tag_id                              # tag identifier (matches mdp-greenhouse tag IDs)
builtin_interfaces/Time stamp              # wall-clock time when the bridge polled the simulator
float64 sim_time_of_day_seconds            # sim-internal time of day in [0, 86400)
SensorReading[] readings                   # one entry per sensor exposed at this tag (a subset
                                           # of the 5 default types; varies per tag)
