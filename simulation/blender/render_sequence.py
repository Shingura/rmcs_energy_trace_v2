"""在 Blender 中按取矿规划渲染一段连续的 RGB-D 序列，用来离线回放整条位姿估计流程。

用法（在包目录下）：
    blender --background --factory-startup --python simulation/blender/render_sequence.py -- \\
        --out dataset/scenarios/<名称>/sequence --plan dataset/scenarios/<名称>/plan.json [--seed 0] \\
        [--samples 32] [--device CPU] [--hdri-dir dataset/inputs/hdri]

场景用 render_dataset.py 的搭建函数，整段序列只搭建一次。镜头效果和曝光整段相同，相机每帧的随机种子不同。

规划文件由 simulation/planning/pickup_plan 生成。机械臂（arm_description 的 STL）按每帧的关节角摆放，
相机装在规划文件指定的连杆上；中心块的位姿和各安装位有没有单元也取自规划文件；
目标单元在被夹住的帧随夹爪（link_6）运动。帧数、分辨率和视场角由规划文件决定。规划文件（JSON）：
    fps                 帧率
    base                底座（URDF 根连杆）到世界的 4×4 位姿
    hub.pose            中心块到世界的 4×4 位姿（中心块系：原点在中心，x 右，y 朝后，z 上）
    hub.slots           6 个布尔值，第 k 个安装位（方向 30° + 60°·k）有没有单元
    hub.target          目标单元所在的安装位
    camera.link         相机所在的连杆；camera.pose 是相机（OpenCV 坐标系）到该连杆的 4×4 位姿
    camera.width、camera.height、camera.hfov_deg  分辨率和水平视场角
    camera.bracket      可选：支架折线（该连杆坐标系中的点列），只用于画出支架
    frames[]            每帧的 joints（6 个关节角，rad）、phase（阶段名）和 attached（目标是否已被夹住）
世界坐标系：z 朝上，地面为 z = 0，中心块正面朝 -y（机器人一侧）。

每帧输出：
    color/<帧号>.jpg   彩色图
    depth/<帧号>.png   深度图：16 位无符号整数，单位 mm，沿光轴的平面深度，0 表示没有深度
    index/<帧号>.png   每个像素属于第几根能量单元（0 为其他物体）
    meta/<帧号>.json   各单元的位姿：mesh 到相机，OpenCV 相机坐标系，单位 m。原点在能量单元的包围盒中心，
                       z 轴指向封闭端，与 C++ 的 geometry::load_binary_stl 相同。另有相机到世界的位姿 camera，
                       以及这一帧的 joints、phase 和 attached
sequence.json 记录相机内参、目标单元的编号、帧数和用到的效果。已有 meta 的帧会跳过，中断后可以继续。
"""

import argparse
import json
import math
import random
import struct
import sys
import time
import zlib
from pathlib import Path

import bpy
import numpy as np
from mathutils import Matrix, Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
import arm as arm_model  # noqa: E402
import render_dataset as rd  # noqa: E402

MAX_DEPTH = 10.0  # m，更远的像素（包括没有击中任何物体的背景）记为没有深度


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser(description="Render a synthetic RGB-D sequence.")
    parser.add_argument("--out", type=Path, required=True, help="output directory")
    parser.add_argument("--plan", type=Path, required=True, help="pickup plan (JSON) that drives the arm and the camera")
    parser.add_argument("--seed", type=int, default=0, help="scene seed")
    parser.add_argument("--samples", type=int, default=32, help="Cycles samples per pixel")
    parser.add_argument("--device", choices=("CPU", "CUDA", "OPTIX"), default="CPU")
    parser.add_argument("--hdri-dir", type=Path, default=rd.PACKAGE_DIR / "dataset" / "inputs" / "hdri")
    return parser.parse_args(argv)


def write_png16(path, image):
    """写 16 位灰度 PNG（OpenCV 用 IMREAD_UNCHANGED 读出 CV_16U）。用标准库编码，避开 Blender 的色彩管理"""
    height, width = image.shape
    raw = b"".join(b"\x00" + row.astype(">u2").tobytes() for row in image)

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))

    header = struct.pack(">IIBBBBB", width, height, 16, 0, 0, 0, 0)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(raw, 6))
                     + chunk(b"IEND", b""))


def exr_to_depth_png(exr_path, png_path):
    image = bpy.data.images.load(str(exr_path))
    width, height = image.size
    pixels = np.empty(width * height * 4, dtype=np.float32)
    image.pixels.foreach_get(pixels)
    bpy.data.images.remove(image)
    depth = pixels[0::4].reshape(height, width)[::-1]  # Blender 的像素从下往上排
    millimeters = np.where((depth > 0.0) & (depth < MAX_DEPTH), np.rint(depth * 1000.0), 0.0)
    write_png16(png_path, millimeters.astype(np.uint16))
    exr_path.unlink()


def build_scene(args, rng, mesh, hub_matrix, hub_slots, keep_clear):
    """搭建资源区、中心块、地上的单元、干扰物和灯光，返回单元列表。
    keep_clear 中的 (位置, 半径) 留给机器人，地上的单元和干扰物不放在那里"""
    rd.clear_scene()
    units = []
    half_width, back, height = rd.build_zone(rng)
    rd.build_hub(rng, mesh, units, matrix=hub_matrix, slots=hub_slots)
    taken = list(keep_clear)
    rd.build_floor_units(rng, mesh, units, half_width, taken)
    rd.build_distractors(rng, half_width, taken)
    rd.build_world(rng, args.hdris)
    rd.build_sources(rng, half_width, back, height)
    rd.build_lights(rng, half_width)
    return units


class Plan:
    """按规划文件摆放机械臂、相机和目标单元"""

    def __init__(self, plan, units):
        self.plan = plan
        self.base = np.array(plan["base"], float)
        camera = plan["camera"]
        self.camera_link = camera["link"]
        self.camera_pose = np.array(camera["pose"], float)
        self.target = next(u for u in units if u.get("slot") == plan["hub"]["target"])
        self.attach = None  # 夹住时目标单元相对 link_6 的位姿（Blender 矩阵）

        self.arm = arm_model.Arm()
        self.arm.build(rd.principled("arm", rd.gray(0.6), 0.45, metallic=0.4))
        if camera.get("bracket"):
            self.arm.add_bracket(camera["bracket"], self.camera_link,
                                 rd.principled("bracket", rd.gray(0.03), 0.4, metallic=0.5))

    def apply(self, frame, camera):
        step = self.plan["frames"][frame]
        frames = self.arm.forward(step["joints"], self.base)
        self.arm.pose(frames)
        camera.matrix_world = arm_model.to_blender(frames[self.camera_link] @ self.camera_pose) @ rd.BLENDER_TO_OPENCV
        gripper = arm_model.to_blender(frames["link_6"])
        target = self.target["object"]
        if step["attached"]:
            # 第一次夹住时记下单元相对夹爪的位姿，之后单元随夹爪运动
            if self.attach is None:
                self.attach = gripper.inverted() @ target.matrix_world
            target.matrix_world = gripper @ self.attach
        return {"joints": list(step["joints"]), "phase": step["phase"], "attached": bool(step["attached"])}


def setup_depth_output(tree, layers, out):
    bpy.context.view_layer.use_pass_z = True
    node = tree.nodes.new("CompositorNodeOutputFile")
    node.name = "depth_output"
    node.base_path = str(out / "depth")
    node.format.file_format = "OPEN_EXR"
    node.format.color_depth = "32"
    node.format.color_mode = "BW"
    node.format.exr_codec = "ZIP"
    node.file_slots[0].path = "######"
    tree.links.new(layers.outputs["Depth"], node.inputs[0])
    return node


def main():
    args = parse_args()
    args.out = args.out.resolve()
    for sub in ("color", "depth", "index", "meta"):
        (args.out / sub).mkdir(parents=True, exist_ok=True)

    plan = json.loads(args.plan.read_text())
    args.frames = len(plan["frames"])
    args.width, args.height = plan["camera"]["width"], plan["camera"]["height"]
    fov = math.radians(plan["camera"]["hfov_deg"])

    bpy.ops.wm.read_factory_settings(use_empty=True)
    mesh = rd.load_unit_mesh()
    corners = np.array([v.co for v in mesh.vertices])
    center_offset = Vector(((corners.min(axis=0) + corners.max(axis=0)) / 2).tolist())
    args.hdris = sorted(args.hdri_dir.glob("*.hdr"))
    index_output = rd.setup_render(args)
    index_output.base_path = str(args.out / "index")

    rng = random.Random(args.seed)
    scene = bpy.context.scene
    base = Vector(np.array(plan["base"], float)[:3, 3].tolist())
    units = build_scene(args, rng, mesh, hub_matrix=arm_model.to_blender(plan["hub"]["pose"]),
                        hub_slots=plan["hub"]["slots"], keep_clear=[(Vector((base.x, base.y, 0.0)), 0.45)])
    driver = Plan(plan, units)

    data = bpy.data.cameras.new("camera")
    data.sensor_fit = "HORIZONTAL"
    data.angle = fov
    camera = rd.link(bpy.data.objects.new("camera", data))
    scene.camera = camera

    tree = scene.node_tree
    setup_depth_output(tree, tree.nodes["Render Layers"], args.out)
    effects = rd.build_effects(rng, tree, args)
    noisy = rng.random() < 0.6
    if noisy:
        effects.append("sensor_noise")
    scene.cycles.samples = rng.randint(12, 24) if noisy else args.samples
    scene.cycles.use_adaptive_sampling = not noisy
    scene.cycles.use_denoising = not noisy
    scene.view_settings.view_transform = rng.choice(("Standard", "AgX", "Filmic"))
    scene.view_settings.exposure = rng.uniform(-1.0, 1.0)
    scene.render.image_settings.quality = 90

    fx = (args.width / 2) / math.tan(fov / 2)
    info = {
        "frames": args.frames,
        "seed": args.seed,
        "target": driver.target["object"].pass_index,
        "camera": {"width": args.width, "height": args.height, "fx": fx, "fy": fx,
                   "cx": args.width / 2 - 0.5, "cy": args.height / 2 - 0.5, "depth_scale": 0.001},
        "effects": effects,
        "plan": str(args.plan.resolve()),
        "fps": plan["fps"],
    }
    (args.out / "sequence.json").write_text(json.dumps(info, indent=1))

    for frame in range(args.frames):
        if (args.out / "meta" / f"{frame:06d}.json").exists():
            continue
        begin = time.monotonic()
        extra = driver.apply(frame, camera)

        scene.frame_current = frame
        scene.cycles.seed = frame
        scene.render.filepath = str(args.out / "color" / f"{frame:06d}.jpg")
        bpy.ops.render.render(write_still=True)
        rd.exr_to_png(args.out / "index" / f"{frame:06d}.exr", args.out / "index" / f"{frame:06d}.png")
        exr_to_depth_png(args.out / "depth" / f"{frame:06d}.exr", args.out / "depth" / f"{frame:06d}.png")

        camera_to_world = camera.matrix_world @ rd.BLENDER_TO_OPENCV
        world_to_camera = camera_to_world.inverted()
        meta = {
            "frame": frame,
            "camera": [list(row) for row in camera_to_world],
            "units": [
                {"index": u["object"].pass_index, "where": u["where"],
                 "pose": [list(row) for row in
                          world_to_camera @ u["object"].matrix_world @ Matrix.Translation(center_offset)]}
                for u in units
            ],
            **extra,
        }
        (args.out / "meta" / f"{frame:06d}.json").write_text(json.dumps(meta, indent=1))
        print(f"frame {frame}: {time.monotonic() - begin:.1f} s", flush=True)


if __name__ == "__main__":
    main()
