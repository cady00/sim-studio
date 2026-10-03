state = {"t": 0.0}

def on_start():
    state["t"] = 0.0

def on_step(dt):
    state["t"] += dt
    if state["t"] < 1.5:
        sim.set_joint_target("Slider", 80)
    else:
        sim.set_joint_target("Slider", 0)
        sim.set_joint_target("Hinge", 40)
