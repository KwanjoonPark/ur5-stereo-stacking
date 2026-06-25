"""스테레오 비전 인식 + UR5 실제 파지로 피라미드 쌓기 (전체 통합)."""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import time
import math

from vision_test import grab_frame, detect_and_annotate
from stereo_test import compute_intrinsics, get_cam_pose, detections_to_dict, triangulate
from kinematics_test import find_by_alias, get_arm_joints, dist

SAFE_Z = 0.42
GRASP_SETTLE = 1.0
GRIP_TILT = 0.0             # 수직 top-down
GRIP_ROLL = 0.0            # 손가락 닫히는 축을 쌓는 축과 수직으로
GRIP_EULER = [math.pi, GRIP_TILT, GRIP_ROLL]
GRASP_Z_OFFSET = 0.02      # 파지 높이 보정
PLACE_CLEARANCE = 0.01     # 놓을 때 살짝 위에서
CUBE_SIZE = 0.05
CUBE_MASS = 0.05
GRIP_FRICTION = 50.0
STACK_CENTER = (-0.45, 0.0)

STACK_ORDER = ['red', 'green', 'blue']
COLOR_TO_TARGET = {'red': 'Target_Red', 'green': 'Target_Green', 'blue': 'Target_Blue'}
DETECT_FRAMES = 10

# 엔진별 마찰 파라미터 (활성 엔진 것만 적용됨)
FRICTION_PARAMS = ['bullet_body_friction', 'ode_body_friction',
                   'newton_body_staticfriction', 'newton_body_kineticfriction',
                   'mujoco_body_friction1', 'mujoco_body_friction2',
                   'vortex_body_primlinearaxisfriction', 'vortex_body_seclinearaxisfriction']


def slerp(q0, q1, t):
    """쿼터니언 구면 선형 보간 (자세를 부드럽게 회전)."""
    d = sum(q0[i] * q1[i] for i in range(4))
    if d < 0.0:
        q1 = [-c for c in q1]; d = -d
    if d > 0.9995:
        r = [q0[i] + t * (q1[i] - q0[i]) for i in range(4)]
    else:
        th0 = math.acos(max(-1.0, min(1.0, d)))
        s0 = math.sin((1 - t) * th0) / math.sin(th0)
        s1 = math.sin(t * th0) / math.sin(th0)
        r = [s0 * q0[i] + s1 * q1[i] for i in range(4)]
    nrm = math.sqrt(sum(c * c for c in r)) or 1.0
    return [c / nrm for c in r]


def boost_friction(sim, handle, val):
    """활성 엔진의 마찰계수 설정."""
    for name in FRICTION_PARAMS:
        pid = getattr(sim, name, None)
        if pid is None:
            continue
        try:
            sim.setEngineFloatParam(pid, handle, val)
        except Exception:
            pass


def cleanup_ik_dummies(sim):
    victims = []
    for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0):
        if sim.getObjectAlias(h, 1).split('/')[-1].startswith(('ikTip', 'ikTarget')):
            victims.append(h)
    for h in victims:
        try:
            sim.removeObjects([h])
        except Exception:
            pass


def detect_world_positions(sim, cam_left, cam_right):
    """스테레오로 블록 3개의 월드 좌표 검출 (여러 프레임 평균)."""
    first = grab_frame(sim, cam_left)
    H, W = first.shape[:2]
    f, cx, cy = compute_intrinsics(sim, cam_left, W, H)
    R_L, C_L = get_cam_pose(sim, cam_left)

    acc = {}
    for _ in range(DETECT_FRAMES):
        img_L = grab_frame(sim, cam_left)
        img_R = grab_frame(sim, cam_right)
        _, _, _, dets_L = detect_and_annotate(img_L)
        _, _, _, dets_R = detect_and_annotate(img_R)
        res = triangulate(detections_to_dict(dets_L), detections_to_dict(dets_R), f, cx, cy)
        for color, r in res.items():
            p_world = R_L @ np.array([r['Xc'], r['Yc'], r['Zc']]) + C_L
            acc.setdefault(color, []).append(p_world)
        time.sleep(0.05)

    positions = {c: list(np.mean(pts, axis=0)) for c, pts in acc.items()}
    print("검출 결과:")
    for c in STACK_ORDER:
        if c in positions:
            p = positions[c]
            print(f"  {c:<6} ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})")
        else:
            print(f"  {c:<6} 검출 실패")
    return positions


def main():
    print("CoppeliaSim 연결 중...")
    client = RemoteAPIClient()
    sim = client.require('sim')
    simIK = client.require('simIK')

    cam_left = find_by_alias(sim, 'Camera_Left')
    cam_right = find_by_alias(sim, 'Camera_Right')
    base = find_by_alias(sim, 'UR5')
    attach = find_by_alias(sim, 'attachPoint')
    gripper_joint = find_by_alias(sim, 'openCloseJoint')
    arm_joints = get_arm_joints(sim)

    cleanup_ik_dummies(sim)

    attach_pose = sim.getObjectPose(attach, -1)
    tip = sim.createDummy(0.02); sim.setObjectAlias(tip, 'ikTip')
    sim.setObjectPose(tip, -1, attach_pose); sim.setObjectParent(tip, attach, True)
    target = sim.createDummy(0.02); sim.setObjectAlias(target, 'ikTarget')
    sim.setObjectPose(target, -1, attach_pose)

    ik_env = simIK.createEnvironment()
    ik_group = simIK.createGroup(ik_env)
    simIK.setGroupCalculation(ik_env, ik_group, simIK.method_damped_least_squares, 0.3, 99)
    _, sim_to_ik, _ = simIK.addElementFromScene(
        ik_env, ik_group, base, tip, target, simIK.constraint_pose)

    # 물성 설정 (시뮬레이션 시작 전)
    shapes = {c: find_by_alias(sim, COLOR_TO_TARGET[c]) for c in STACK_ORDER}
    radius, height = CUBE_SIZE / 2.0, CUBE_SIZE
    for c in STACK_ORDER:
        try:
            sim.setShapeMass(shapes[c], CUBE_MASS)
        except Exception:
            pass
        boost_friction(sim, shapes[c], GRIP_FRICTION)
    # 그리퍼 손가락 셰이프에도 마찰 부여 (마찰은 두 접촉면에서 결합)
    for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0):
        if sim.getObjectType(h) != sim.object_shape_type:
            continue
        if 'RG2' not in sim.getObjectAlias(h, 1):
            continue
        try:
            if sim.getBoolProperty(h, 'respondable'):
                boost_friction(sim, h, GRIP_FRICTION)
        except Exception:
            pass

    sim.startSimulation()

    # 1) 비전 인식
    positions = detect_world_positions(sim, cam_left, cam_right)
    missing = [c for c in STACK_ORDER if c not in positions]
    if missing:
        print(f"{missing} 검출 실패 - 종료")
        sim.stopSimulation()
        return

    # 2) 피라미드 좌표
    z_bottom = float(np.mean([positions[c][2] for c in STACK_ORDER]))
    cx, cy = STACK_CENTER
    place_pos = {
        STACK_ORDER[0]: [cx, cy - radius, z_bottom],
        STACK_ORDER[1]: [cx, cy + radius, z_bottom],
        STACK_ORDER[2]: [cx, cy, z_bottom + height],
    }

    def set_gripper(open_):
        v = 1 if open_ else 0
        try:
            sim.setIntProperty(sim.handle_scene, 'signal.RG2_open', v)
        except Exception:
            sim.setInt32Signal('RG2_open', v)

    def lerp(a, b, t):
        return [a[i] + (b[i] - a[i]) * t for i in range(3)]

    def move_to(pos, tol=0.012, timeout=8.0):
        start = sim.getObjectPosition(tip, -1)
        q_start = sim.getObjectQuaternion(tip, -1)
        sim.setObjectOrientation(target, -1, GRIP_EULER)
        q_goal = sim.getObjectQuaternion(target, -1)
        for j in arm_joints:
            simIK.setJointPosition(ik_env, sim_to_ik[j], sim.getJointPosition(j))
        n = max(12, int(dist(start, pos) / 0.02))
        for s in range(1, n + 1):
            t = s / n
            sim.setObjectPosition(target, -1, lerp(start, pos, t))
            sim.setObjectQuaternion(target, -1, slerp(q_start, q_goal, t))   # 자세 보간
            simIK.setObjectPose(ik_env, sim_to_ik[target], sim.getObjectPose(target, -1))
            simIK.handleGroup(ik_env, ik_group, {'syncWorlds': False, 'allowError': True})
            for j in arm_joints:
                sim.setJointTargetPosition(j, simIK.getJointPosition(ik_env, sim_to_ik[j]))
            time.sleep(0.05)
        t0 = time.time()
        while time.time() - t0 < timeout:
            if dist(sim.getObjectPosition(tip, -1), pos) < tol:
                break
            time.sleep(0.05)
        return dist(sim.getObjectPosition(tip, -1), pos)

    def wait_gripper_settled(timeout=5.0, min_time=1.5):
        # 그리퍼가 충분히 닫힌 뒤 멈출 때까지 대기 (초반 정지 오판 방지)
        t0 = time.time()
        prev = sim.getJointPosition(gripper_joint)
        still = 0
        while time.time() - t0 < timeout:
            time.sleep(0.2)
            cur = sim.getJointPosition(gripper_joint)
            still = still + 1 if abs(cur - prev) < 2e-4 else 0
            prev = cur
            if time.time() - t0 >= min_time and still >= 2:
                break

    def grasp():
        set_gripper(False)
        wait_gripper_settled()
        time.sleep(0.3)

    def release():
        set_gripper(True)
        time.sleep(GRASP_SETTLE)

    def pick_and_place(color):
        pick = positions[color]
        pick_hover = [pick[0], pick[1], SAFE_Z]
        pick_down = [pick[0], pick[1], pick[2] + GRASP_Z_OFFSET]
        place = place_pos[color]
        place_hover = [place[0], place[1], SAFE_Z]
        # 중심보다 GRASP_Z_OFFSET 위에서 잡으므로 보정, 살짝 위에서 떨어뜨림
        place_down = [place[0], place[1], place[2] + GRASP_Z_OFFSET + PLACE_CLEARANCE]

        state = 'OPEN'
        while state != 'DONE':
            if state == 'OPEN':
                release(); state = 'APPROACH_PICK'
            elif state == 'APPROACH_PICK':
                move_to(pick_hover); state = 'DESCEND_PICK'
            elif state == 'DESCEND_PICK':
                move_to(pick_down); state = 'GRASP'
            elif state == 'GRASP':
                z0 = sim.getObjectPosition(shapes[color], -1)[2]
                grasp(); state = 'LIFT'
            elif state == 'LIFT':
                move_to(pick_hover)
                z1 = sim.getObjectPosition(shapes[color], -1)[2]
                print(f"  [{color}] {'파지 성공' if z1 - z0 > 0.05 else '파지 실패'}")
                state = 'APPROACH_PLACE'
            elif state == 'APPROACH_PLACE':
                move_to(place_hover); state = 'DESCEND_PLACE'
            elif state == 'DESCEND_PLACE':
                move_to(place_down); state = 'RELEASE'
            elif state == 'RELEASE':
                release(); state = 'RETREAT'
            elif state == 'RETREAT':
                move_to(place_hover); state = 'DONE'

    try:
        set_gripper(True)
        time.sleep(GRASP_SETTLE)
        for color in STACK_ORDER:
            pick_and_place(color)
        time.sleep(5)
    except KeyboardInterrupt:
        pass
    finally:
        sim.stopSimulation()
        for h in (tip, target):
            try:
                sim.removeObjects([h])
            except Exception:
                pass


if __name__ == '__main__':
    main()
