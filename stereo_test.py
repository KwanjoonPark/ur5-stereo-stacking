"""스테레오 비전으로 색상 블록의 3D 월드 좌표 추정 (Step 1)."""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import cv2
import numpy as np

from vision_test import grab_frame, detect_and_annotate

BASELINE = 0.2          # 좌우 카메라 간격 (m), 월드 Y축 방향

# 이미지 u/v축이 센서 좌표와 반대로 들어와 부호 보정
SIGN_U = -1
SIGN_V = -1

EPIPOLAR_TOL = 5
PRINT_EVERY = 30

# 검증용 블록 실제 좌표
GROUND_TRUTH = {
    "red":   (-0.500,  0.125, 0.19),
    "green": (-0.375,  0.025, 0.19),
    "blue":  (-0.525, -0.075, 0.19),
}


def compute_intrinsics(sim, cam, width, height):
    """FOV로부터 초점거리 f(px)와 주점 계산. (긴 변 기준)"""
    fov = sim.getObjectFloatParam(cam, sim.visionfloatparam_perspective_angle)
    f = (max(width, height) / 2.0) / np.tan(fov / 2.0)
    cx, cy = width / 2.0, height / 2.0
    return f, cx, cy


def get_cam_pose(sim, cam):
    """카메라 월드 변환 (R, C) 반환. P_world = R @ p_cam + C."""
    M = np.array(sim.getObjectMatrix(cam, -1)).reshape(3, 4)
    return M[:, :3], M[:, 3]


def detections_to_dict(dets):
    out = {}
    for color, u, v in dets:
        if color not in out:
            out[color] = (u, v)
    return out


def triangulate(det_L, det_R, f, cx, cy):
    """좌우 매칭 후 삼각측량으로 카메라 기준 3D 좌표 계산."""
    out = {}
    for color, (uL, vL) in det_L.items():
        if color not in det_R:
            continue
        uR, vR = det_R[color]
        epi_err = abs(vL - vR)              # 에피폴라 검증 (v_L ~ v_R)
        d = abs(uL - uR)                    # 시차
        if d < 1e-6:
            continue
        Z = f * BASELINE / d
        Xc = SIGN_U * (uL - cx) * Z / f
        Yc = SIGN_V * (vL - cy) * Z / f
        out[color] = dict(Xc=Xc, Yc=Yc, Zc=Z, disparity=d, epi_err=epi_err)
    return out


def print_report(results, R_L, C_L):
    print("\n" + "=" * 78)
    print(f"{'color':<6} {'camera (Xc,Yc,Zc)':<26} {'world (X,Y,Z)':<26} {'err(cm)':>7}")
    print("-" * 78)
    for color, r in results.items():
        p_cam = np.array([r["Xc"], r["Yc"], r["Zc"]])
        p_world = R_L @ p_cam + C_L
        cam_s = f"({p_cam[0]:+.3f},{p_cam[1]:+.3f},{p_cam[2]:+.3f})"
        world_s = f"({p_world[0]:+.3f},{p_world[1]:+.3f},{p_world[2]:+.3f})"
        gt = GROUND_TRUTH.get(color)
        err = np.linalg.norm(p_world - np.array(gt)) * 100 if gt else float("nan")
        warn = "  epi!" if r["epi_err"] > EPIPOLAR_TOL else ""
        print(f"{color:<6} {cam_s:<26} {world_s:<26} {err:>7.1f}{warn}")
    print("=" * 78, flush=True)


def annotate_3d(img, det_L, results):
    for color, r in results.items():
        if color not in det_L:
            continue
        u, v = det_L[color]
        cv2.putText(img, f"Z={r['Zc']:.2f}m", (u - 28, v + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)


def main():
    print("CoppeliaSim 연결 중...")
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    try:
        cam_left = sim.getObject('/Table/Camera_Left')
        cam_right = sim.getObject('/Table/Camera_Right')
    except Exception as e:
        print(f"카메라를 찾을 수 없습니다: {e}")
        return

    sim.startSimulation()

    first = grab_frame(sim, cam_left)
    H, W = first.shape[:2]
    f, cx, cy = compute_intrinsics(sim, cam_left, W, H)
    R_L, C_L = get_cam_pose(sim, cam_left)
    print(f"intrinsics: f={f:.1f}px, res={W}x{H}, baseline={BASELINE}m")

    frame = 0
    try:
        while True:
            img_L = grab_frame(sim, cam_left)
            img_R = grab_frame(sim, cam_right)
            out_L, _, _, dets_L = detect_and_annotate(img_L)
            out_R, _, _, dets_R = detect_and_annotate(img_R)

            det_L = detections_to_dict(dets_L)
            det_R = detections_to_dict(dets_R)
            results = triangulate(det_L, det_R, f, cx, cy)
            annotate_3d(out_L, det_L, results)

            if frame % PRINT_EVERY == 0 and results:
                print_report(results, R_L, C_L)

            cv2.imshow('Left View (3D)', out_L)
            cv2.imshow('Right View', out_R)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            frame += 1
    except KeyboardInterrupt:
        pass
    finally:
        sim.stopSimulation()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
