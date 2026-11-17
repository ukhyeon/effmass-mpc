"""b2a2_scenes.py - write the B2a-2 scene jsons (free_front, free_top, slot_front) from slots.json (ends and plates checked before any run).
Goals = hand-link targets (TCP + 0.1034 m in z), gripper down; q_start = IK solution at end A; slot_front adds the two slot_front plates."""
import json, os
G = os.path.expanduser("~/effmass_mpc/goal1/b2a/g2"); S = json.load(open(f"{G}/slots.json")); q4 = [0.0, 1.0, 0.0, 0.0]
for name, line, plates in (("free_front", "front", []), ("free_top", "top", []), ("slot_front", "front", S["plates"]["slot_front"]["plates"])):
    e = S["ends"][line]
    js = dict(path=line, scene=name, goals_wxyz=[list(e["hand_A"]) + q4, list(e["hand_B"]) + q4], q_start=e["q_A"], tcp_A=e["A_tcp"], tcp_B=e["B_tcp"], dy=e["dy"],
              plates=[dict(name=p["name"], pose=list(p["center"]) + [1.0, 0.0, 0.0, 0.0], dims=p["dims"]) for p in plates],
              note="B2a-2: single pass A -> B; ends from slots.json (end gaps >= 0.23 m); plates from slots.json (rules 1-3 checked)")
    json.dump(js, open(f"{G}/scene_b2a2_{name}.json", "w"), indent=1)
    print(name, "A", [round(x, 3) for x in e["A_tcp"]], "B", [round(x, 3) for x in e["B_tcp"]], "plates", [p["name"] for p in js["plates"]])
