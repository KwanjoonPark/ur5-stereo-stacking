"""타겟 블록을 원위치로 복구 - 파지 도중 중단됐을 때 사용하는 보조 스크립트."""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from kinematics_test import find_by_alias

TARGETS = {
    'Target_Red':   [-0.500,  0.125, 0.19],
    'Target_Green': [-0.375,  0.025, 0.19],
    'Target_Blue':  [-0.525, -0.075, 0.19],
}


def main():
    client = RemoteAPIClient()
    sim = client.require('sim')

    for name, pos in TARGETS.items():
        try:
            h = find_by_alias(sim, name)
        except ValueError:
            print(f"{name} 을(를) 찾지 못했습니다.")
            continue

        if sim.getObjectParent(h) != -1:
            sim.setObjectParent(h, -1, True)       # 그리퍼에서 분리
        sim.setObjectPosition(h, -1, pos)
        sim.setObjectOrientation(h, -1, [0.0, 0.0, 0.0])
        try:
            sim.setBoolProperty(h, 'dynamic', True)
        except Exception:
            pass
        print(f"{name} 복구")


if __name__ == '__main__':
    main()
