"""step4_node_consts.py — (ii) 노드 자체 코드로 정지 자세 상수 계산 (노드는 띄우지 않음). run_sys_ros.sh -B 로 실행.
human_dynamics_node.mass_inertia() 와 같은 호출 순서: SMPLH(male).forward(betas, 영 자세) → compute_massinertia → merge_parts_24_to_19_vertex_avg
"""
import os, sys
import numpy as np, torch
from hrc_body_tracking.physcis import Physics, SMPLH
from hrc_body_tracking.utils import batch_roteulerSMPL
import hrc_body_tracking
print("[consts] hrc_body_tracking from", hrc_body_tracking.__file__, flush=True)
dev = torch.device("cpu")
phys = Physics(device=dev); smplh = SMPLH(gender="male", device=dev)
betas = torch.zeros((1, 10), dtype=torch.float32)
rot0, _ = batch_roteulerSMPL(torch.zeros((1, 72), dtype=torch.float32))
out = smplh.forward(betas=betas, rotmat=rot0)
m24, v24, I24, c24 = phys.compute_massinertia(out.vertices, out.joints_smpl)
m19, _, I19, c19 = phys.merge_parts_24_to_19_vertex_avg(m24, v24, I24, c24)
idx = [0, 1, 2, 3, 4, 5, 7, 8, 6, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21]
rest19 = out.joint_smpl_shaped[:, idx, :]
off19 = torch.zeros_like(rest19)
for j in phys._smpl19_topo_order:
    p = phys._smpl19_parent[j]
    if p >= 0:
        off19[:, j] = rest19[:, j] - rest19[:, p]
outp = os.path.expanduser("~/effmass_mpc/env_check/step4/node_consts_betas0.npz")
np.savez(outp, rest_joints_19=rest19[0].numpy().astype(np.float64), rest_joint_offsets_19=off19[0].numpy().astype(np.float64),
         com_local_offsets_19=(c19 - rest19)[0].numpy().astype(np.float64), centroid_19=c19[0].numpy(), mass_19=m19[0].numpy(),
         topo=np.array(phys._smpl19_topo_order), parent=np.array(phys._smpl19_parent))
print("[consts] saved", outp, "topo", phys._smpl19_topo_order, "parent", phys._smpl19_parent, flush=True)
