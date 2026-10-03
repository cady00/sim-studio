seen = {"n": 0, "hot": False}

def on_step(dt):
    blocked = sim.get_bool("barrier")
    if blocked and not seen["hot"]:
        seen["n"] += 1
        sim.set_int("parts_seen", seen["n"])
    seen["hot"] = blocked
