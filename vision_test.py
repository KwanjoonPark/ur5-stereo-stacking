from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import cv2
import numpy as np

# 색상별 HSV 범위 (red는 0/180 양쪽에 걸쳐 두 구간 사용)
COLOR_RANGES = {
    "red":   [([0, 80, 50], [10, 255, 255]), ([170, 80, 50], [180, 255, 255])],
    "green": [([35, 80, 50], [85, 255, 255])],
    "blue":  [([100, 150, 50], [130, 255, 255])],
}

# 표시용 색상 (BGR)
DISPLAY_COLORS = {
    "red":   (60, 60, 255),
    "green": (60, 220, 60),
    "blue":  (255, 120, 60),
}

MIN_AREA = 100
MAX_AREA = 50000


def grab_frame(sim, cam):
    """카메라에서 한 프레임을 받아 BGR로 보정."""
    img_raw, res = sim.getVisionSensorImg(cam)
    img = np.frombuffer(img_raw, dtype=np.uint8).reshape((res[1], res[0], 3))
    img = cv2.flip(img, 0)                      # 상하 반전 보정
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)  # RGB -> BGR
    return img


def detect_and_annotate(img):
    """색상 객체를 검출하고 시각화. (annotated, mask, hsv, detections) 반환."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    debug_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    overlay = img.copy()
    found = []

    for color_name, ranges in COLOR_RANGES.items():
        mask = None
        for lower, upper in ranges:
            m = cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        debug_mask = cv2.bitwise_or(debug_mask, mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if MIN_AREA < area < MAX_AREA:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.drawContours(overlay, [cnt], -1, DISPLAY_COLORS[color_name], cv2.FILLED)
                    found.append((color_name, cnt, cX, cY))

    # 반투명 합성으로 검출 영역 채색
    alpha = 0.45
    out = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)

    detections = []
    for color_name, cnt, cX, cY in found:
        col = DISPLAY_COLORS[color_name]
        cv2.drawContours(out, [cnt], -1, col, 2)
        cv2.circle(out, (cX, cY), 3, (255, 255, 255), -1)

        label = f"{color_name} ({cX},{cY})"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        ty = cY - 12 if cY - 12 - th > 0 else cY + th + 14
        cv2.rectangle(out, (cX - tw // 2 - 3, ty - th - 4),
                      (cX + tw // 2 + 3, ty + 4), col, -1)
        cv2.putText(out, label, (cX - tw // 2, ty),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        detections.append((color_name, cX, cY))

    return out, debug_mask, hsv, detections


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
    print("시뮬레이션 시작 (종료: 영상 창에서 q)")

    try:
        while True:
            img_L = grab_frame(sim, cam_left)
            img_R = grab_frame(sim, cam_right)
            out_L, _, _, _ = detect_and_annotate(img_L)
            out_R, _, _, _ = detect_and_annotate(img_R)

            cv2.imshow('Left View', out_L)
            cv2.imshow('Right View', out_R)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        sim.stopSimulation()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
