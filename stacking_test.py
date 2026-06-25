"""UR5로 2층 피라미드 쌓기 (Step 3, FSM)."""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math

from kinematics_test import find_by_alias, get_arm_joints, dist

SAFE_Z = 0.42                 # 이동 시 안전 고도
GRASP_SETTLE = 0.6
GRIP_TILT = 1.0               # approach 축 기울기
GRIP_ROLL = math.pi / 2       # 손가락 닫히는 평면
GRIP_EULER = [math.pi, GRIP_TILT, GRIP_ROLL]
KEEP_PLACED_STATIC = True
STACK_CENTER = (-0.45, 0.0)
STACK_ORDER = ['Target_Red', 'Target_Green', 'Target_Blue']   # 바닥 왼, 바닥 오, 꼭대기


def cleanup_ik_dummies(sim):
    """이전 실행에서 남은 ikTip/ikTarget 더미 제거."""
    victims = []
    for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0):
        leaf = sim.getObjectAlias(h, 1).split('/')[-1]
        if leaf.startswith('ikTip') or leaf.startswith('ikTarget'):
            victims.append(h)
    for h in victims:
        try:
            sim.removeObjects([h])
        except Exception:
            try:
                sim.removeObject(h)
            except Exception:
                pass


def main():
    print("CoppeliaSim 연결 중...")
    client = RemoteAPIClient()
    sim = client.require('sim')
    simIK = client.require('simIK')

    cleanup_ik_dummies(sim)

    base = find_by_alias(sim, 'UR5')
    attach = find_by_alias(sim, 'attachPoint')
    arm_joints = get_arm_joints(sim)

    attach_pose = sim.getObjectPose(attach, -1)
    tip = sim.createDummy(0.02); sim.setObjectAlias(tip, 'ikTip')
    sim.setObjectPose(tip, -1, attach_pose)
    sim.setObjectParent(tip, attach, True)
    target = sim.createDummy(0.02); sim.setObjectAlias(target, 'ikTarget')
    sim.setObjectPose(target, -1, attach_pose)

    ik_env = simIK.createEnvironment()
    ik_group = simIK.createGroup(ik_env)
    simIK.setGroupCalculation(ik_env, ik_group, simIK.method_damped_least_squares, 0.3, 99)
    _, sim_to_ik, _ = simIK.addElementFromScene(
        ik_env, ik_group, base, tip, target, simIK.constraint_pose)

    # 블록 치수로 피라미드 좌표 계산
    blocks = {n: find_by_alias(sim, n) for n in STACK_ORDER}
    sample = blocks[STACK_ORDER[0]]
    try:
        bb = sim.getShapeBB(sample)
        radius, height = bb[0] / 2.0, bb[2]
    except Exception:
        radius, height = 0.02, 0.08
    z_bottom = sim.getObjectPosition(sample, -1)[2]
    cx, cy = STACK_CENTER
    place_pos = {
        STACK_ORDER[0]: [cx, cy - radius, z_bottom],
        STACK_ORDER[1]: [cx, cy + radius, z_bottom],
        STACK_ORDER[2]: [cx, cy, z_bottom + height],
    }
    upright = {n: sim.getObjectOrientation(blocks[n], -1) for n in STACK_ORDER}

    sim.startSimulation()

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
        for j in arm_joints:
            simIK.setJointPosition(ik_env, sim_to_ik[j], sim.getJointPosition(j))
        n = max(1, int(dist(start, pos) / 0.02))   # 2cm 간격 보간
        for s in range(1, n + 1):
            sim.setObjectPosition(target, -1, lerp(start, pos, s / n))
            sim.setObjectOrientation(target, -1, GRIP_EULER)
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

    def grasp(shape):
        # 블록을 그리퍼 중심에 맞추고 부착
        ap = sim.getObjectPosition(attach, -1)
        bp = sim.getObjectPosition(shape, -1)
        sim.setObjectPosition(shape, -1, [ap[0], ap[1], bp[2]])
        sim.setBoolProperty(shape, 'dynamic', False)
        sim.setObjectParent(shape, attach, True)
        held['shape'] = shape
        set_gripper(False)
        time.sleep(GRASP_SETTLE)

    def release(shape, place, ori):
        sim.setObjectParent(shape, -1, True)
        held['shape'] = None
        sim.setObjectPosition(shape, -1, place)
        sim.setObjectOrientation(shape, -1, ori)
        sim.setBoolProperty(shape, 'dynamic', not KEEP_PLACED_STATIC)
        set_gripper(True)
        time.sleep(GRASP_SETTLE)

    def pick_and_place(name):
        shape = blocks[name]
        pick = sim.getObjectPosition(shape, -1)
        pick_hover = [pick[0], pick[1], SAFE_Z]
        pick_down = [pick[0], pick[1], pick[2]]
        place = place_pos[name]
        place_hover = [place[0], place[1], SAFE_Z]
        place_down = list(place)

        state = 'APPROACH_PICK'
        while state != 'DONE':
            if state == 'APPROACH_PICK':
                move_to(pick_hover); state = 'DESCEND_PICK'
            elif state == 'DESCEND_PICK':
                move_to(pick_down); state = 'GRASP'
            elif state == 'GRASP':
                grasp(shape); state = 'LIFT'
            elif state == 'LIFT':
                move_to(pick_hover); state = 'APPROACH_PLACE'
            elif state == 'APPROACH_PLACE':
                move_to(place_hover); state = 'DESCEND_PLACE'
            elif state == 'DESCEND_PLACE':
                move_to(place_down); state = 'RELEASE'
            elif state == 'RELEASE':
                release(shape, place_down, upright[name]); state = 'RETREAT'
            elif state == 'RETREAT':
                move_to(place_hover); state = 'DONE'

    held = {'shape': None}        # 중단 시 떼어내기 위해 추적
    try:
        set_gripper(True)
        time.sleep(GRASP_SETTLE)
        for name in STACK_ORDER:
            print(f"{name} 쌓기")
            pick_and_place(name)
        time.sleep(5)
    except KeyboardInterrupt:
        pass
    finally:
        if held['shape'] is not None:
            try:
                sim.setObjectParent(held['shape'], -1, True)
                sim.setBoolProperty(held['shape'], 'dynamic', True)
            except Exception:
                pass
        sim.stopSimulation()
        for h in (tip, target):
            try:
                sim.removeObjects([h])
            except Exception:
                pass


if __name__ == '__main__':
    main()
