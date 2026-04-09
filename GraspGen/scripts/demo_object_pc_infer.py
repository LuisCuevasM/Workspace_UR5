import argparse, glob, json, os
import numpy as np
import torch


from grasp_gen.grasp_server import GraspGenSampler, load_grasp_cfg
from grasp_gen.utils.meshcat_utils import (
    create_visualizer,
    get_color_from_score,
    visualize_grasp,
    visualize_pointcloud,
)
from grasp_gen.utils.point_cloud_utils import point_cloud_outlier_removal
import trimesh.transformations as tra


def parse_args():
    p = argparse.ArgumentParser("Run GraspGen on YOUR point clouds (no precomputed grasps)")
    p.add_argument("--sample_data_dir", type=str, required=True)
    p.add_argument("--gripper_config", type=str, required=True)
    p.add_argument("--grasp_threshold", type=float, default=0.8)
    p.add_argument("--num_grasps", type=int, default=200)
    p.add_argument("--topk_num_grasps", type=int, default=-1)
    return p.parse_args()


def center_pc(pc):
    T = tra.translation_matrix(-pc.mean(axis=0))
    pc_c = tra.transform_points(pc, T)
    return pc_c, T


def keep_largest_cluster(pc, pc_color=None, eps=0.02, min_samples=30):
            # eps en metros: 0.015–0.03 suele andar bien en RealSense
            from sklearn.cluster import DBSCAN
            if len(pc) < min_samples:
                return pc, pc_color

            labels = DBSCAN(eps=eps, min_samples=min_samples).fit(pc).labels_
            # -1 es ruido
            valid = labels >= 0
            if valid.sum() == 0:
                return pc, pc_color

            # cluster más grande
            lbls, counts = np.unique(labels[valid], return_counts=True)
            best = lbls[np.argmax(counts)]
            sel = labels == best
            if pc_color is None:
                return pc[sel], None
            return pc[sel], pc_color[sel]
       

if __name__ == "__main__":
    args = parse_args()

    json_files = sorted(glob.glob(os.path.join(args.sample_data_dir, "*.json")))
    if not json_files:
        raise RuntimeError(f"No JSON files found in {args.sample_data_dir}")

    grasp_cfg = load_grasp_cfg(args.gripper_config)
    gripper_name = grasp_cfg.data.gripper_name
    sampler = GraspGenSampler(grasp_cfg)
    vis = create_visualizer()

    for jf in json_files:
        print(f"\nProcessing {jf}")
        vis.delete()

        data = json.load(open(jf, "r"))

        pc = np.asarray(data["pc"], dtype=np.float32)


        alpha_deg = 240  # <-- cambia aquí
        alpha = np.deg2rad(alpha_deg)

        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(alpha), -np.sin(alpha)],
            [0, np.sin(alpha),  np.cos(alpha)]
        ], dtype=np.float32)
        
        R_z = np.array([
            [-1, 0, 0],
            [ 0,-1, 0],
            [ 0, 0, 1]
        ], dtype=np.float32)

        R = R_z @ R_x

        pc = (R @ pc.T).T


        if pc.size == 0:
            print("Empty pc, skipping...")
            continue

        # optional colors
        if "pc_color" in data:
            pc_color = np.asarray(data["pc_color"], dtype=np.uint8)
        else:
            pc_color = np.tile(np.array([[180, 180, 180]], dtype=np.uint8), (pc.shape[0], 1))

        pc, pc_color = keep_largest_cluster(pc, pc_color, eps=0.02, min_samples=30)
        print("After DBSCAN:", pc.shape)

        # center
        pc_centered, T_center = center_pc(pc)

        # visualize raw
        visualize_pointcloud(vis, "pc_raw", pc_centered, pc_color, size=0.0025)

        # outlier removal (recommended)
        pc_filt, _ = point_cloud_outlier_removal(torch.from_numpy(pc_centered))
        pc_filt = pc_filt.numpy()

        # inference
        grasps, conf = GraspGenSampler.run_inference(
            pc_filt,
            sampler,
            grasp_threshold=args.grasp_threshold,
            num_grasps=args.num_grasps,
            topk_num_grasps=args.topk_num_grasps,
        )

        if len(grasps) == 0:
            print("No grasps inferred.")
            continue

        grasps = grasps.cpu().numpy()
        conf = conf.cpu().numpy()
        grasps[:, 3, 3] = 1

        colors = get_color_from_score(conf, use_255_scale=True)
        print(f"Inferred {len(grasps)} grasps | conf range {conf.min():.3f}..{conf.max():.3f}")

        # visualize grasps (back to centered frame)
        for i, g in enumerate(grasps):
            visualize_grasp(
                vis,
                f"grasps/{i:03d}/grasp",
                g,
                color=colors[i],
                gripper_name=gripper_name,
                linewidth=0.7,
            )

        input("Press Enter for next file...")
