"""IK로 TCP를 각 타겟 위로 이동시켜 도달 정확도 확인 (Step 2)."""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math

APPROACH_HEIGHT = 0.15
REACH_TOL = 0.01
REACH_TIMEOUT = 6.0

TARGET_NAMES = ['Target_Red', 'Target_Green', 'Target_Blue']


def full_path(sim, h):
    return sim.getObjectAlias(h, 1)        # 루트부터의 전체 경로


def leaf(sim, h):
    return full_path(sim, h).split('/')[-1]


def find_by_alias(sim, name):
    """leaf 이름이 일치하는 첫 객체 핸들 반환."""
    for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0):
        if leaf(sim, h) == name:
            return h
    raise ValueError(f"'{name}' 객체를 찾을 수 없습니다.")


def get_arm_joints(sim):
    """UR5 팔의 6개 조인트를 베이스->팁 순서로 반환 (그리퍼 제외)."""
    joints = []
    for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0):
        if sim.getObjectType(h) == sim.object_joint_type:
            p = full_path(sim, h)
            if 'RG2' not in p:
                joints.append((len(p), h))   # 경로가 짧을수록 베이스에 가까움
    joints.sort()
    return [h for _, h in joints][:6]


def dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def main():
    print("CoppeliaSim 연결 중...")
    client = RemoteAPIClient()
    sim = client.require('sim')
    simIK = client.require('simIK')

    base = find_by_alias(sim, 'UR5')
    attach = find_by_alias(sim, 'attachPoint')   # RG2 부착점을 TCP로 사용
    arm_joints = get_arm_joints(sim)

    # IK 사슬에 쓸 tip/target 더미
    attach_pose = sim.getObjectPose(attach, -1)
    tip = sim.createDummy(0.02)
    sim.setObjectAlias(tip, 'ikTip')
    sim.setObjectPose(tip, -1, attach_pose)
    sim.setObjectParent(tip, attach, True)

    target = sim.createDummy(0.02)
    sim.setObjectAlias(target, 'ikTarget')
    sim.setObjectPose(target, -1, attach_pose)

    ik_env = simIK.createEnvironment()
    ik_group = simIK.createGroup(ik_env)
    simIK.setGroupCalculation(ik_env, ik_group,
                              simIK.method_damped_least_squares, 0.3, 99)
    _, sim_to_ik, _ = simIK.addElementFromScene(
        ik_env, ik_group, base, tip, target, simIK.constraint_position)

    sim.startSimulation()

    def solve_and_command(goal_pos):
        # 현재 자세 동기화 -> 목표 설정 -> IK 풀이 -> 관절 목표 명령
        for j in arm_joints:
            simIK.setJointPosition(ik_env, sim_to_ik[j], sim.getJointPosition(j))
        pose = sim.getObjectPose(target, -1)
        pose[0], pose[1], pose[2] = goal_pos
        simIK.setObjectPose(ik_env, sim_to_ik[target], pose)
        simIK.handleGroup(ik_env, ik_group, {'syncWorlds': False, 'allowError': True})
        for j in arm_joints:
            sim.setJointTargetPosition(j, simIK.getJointPosition(ik_env, sim_to_ik[j]))

    def wait_reach(goal_pos):
        t0 = time.time()
        err = dist(sim.getObjectPosition(tip, -1), goal_pos)
        while time.time() - t0 < REACH_TIMEOUT:
            err = dist(sim.getObjectPosition(tip, -1), goal_pos)
            if err < REACH_TOL:
                break
            time.sleep(0.05)
        return err

    try:
        for name in TARGET_NAMES:
            obj = find_by_alias(sim, name)
            p = sim.getObjectPosition(obj, -1)
            goal = [p[0], p[1], p[2] + APPROACH_HEIGHT]
            solve_and_command(goal)
            err = wait_reach(goal)
            print(f"  {name:<13} 오차 {err*100:.1f}cm")
            time.sleep(0.8)
        time.sleep(5)
    except KeyboardInterrupt:
        pass
    finally:
        sim.stopSimulation()


if __name__ == '__main__':
    main()
