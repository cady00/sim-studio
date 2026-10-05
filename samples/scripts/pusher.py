def on_start():
    sim.set_bool("barrier", False)
    sim.set_joint_target("Pusher", 0)

hold = {"t": 0.0}

def on_step(dt):
    # The barrier sits upstream of the pusher. Hold the stroke so the part
    # arrives before the rod retracts.
    if sim.get_bool("barrier"):
        hold["t"] = 0.9
    if hold["t"] > 0.0:
        sim.set_joint_target("Pusher", 120)
        hold["t"] -= dt
    else:
        sim.set_joint_target("Pusher", 0)
