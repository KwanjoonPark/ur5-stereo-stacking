# 🤖 UR5 Stereo Stacking

CoppeliaSim 환경에서 **스테레오 비전으로 색상 블록을 인식**하고, **UR5 로봇이 집어서 2층 피라미드를 쌓는** 시스템입니다.
Python에서 ZMQ Remote API로 시뮬레이터를 원격 제어합니다.

<!-- 대표 결과 이미지 (images/14_final_result.png 등으로 교체) -->
![demo](images/14_final_result.png)

---

## ✨ 주요 기능

- 🎥 **스테레오 비전** — OpenCV(HSV) 색상 검출 + 삼각측량으로 블록의 3D 월드 좌표 추정 (오차 ~0.5cm)
- 🦾 **역기구학(IK)** — `simIK`로 UR5 엔드이펙터를 목표 좌표로 이동 (오차 ~0.5cm)
- 📦 **탑 쌓기 FSM** — 접근 → 파지 → 이동 → 배치 시퀀스, 'ㄷ'자 충돌 회피 궤적
- ✋ **실제 물리 파지** — RG2 그리퍼를 닫아 마찰력으로 집기 (질량·마찰·자세 보간 처리)

---

## 🛠 요구사항

- CoppeliaSim (ZMQ Remote API 포함, 최신 버전 권장)
- Python 3.8+

```bash
pip install coppeliasim-zmqremoteapi-client opencv-python numpy
```

---

## 🎬 씬 구성 (CoppeliaSim)

씬에 다음 객체가 있어야 합니다.

| 객체 | 설명 |
|---|---|
| `UR5` + `RG2` | 6축 로봇 팔 + 그리퍼 (베이스는 월드 원점) |
| `/Table/Camera_Left`, `Camera_Right` | 스테레오 카메라 (Y축으로 0.2m 간격) |
| `Target_Red`, `Target_Green`, `Target_Blue` | 색상 정육면체 (5cm), **dynamic 속성 활성화** |

> ⚠️ UR5 모델에 기본 포함된 child script는 자체적으로 관절을 제어하므로 **제거**해야 외부 제어와 충돌하지 않습니다.

---

## ▶️ 실행 방법

CoppeliaSim에서 씬을 연 뒤(시뮬레이션은 정지 상태), 아래 순서로 실행합니다.

```bash
# 1. 통신 연결 확인
python connection_test.py

# 2. 색상 검출 확인 (영상 창, q로 종료)
python vision_test.py

# 3. 스테레오 3D 좌표 추정 + 정확도 검증
python stereo_test.py

# 4. 역기구학으로 타겟 위로 이동
python kinematics_test.py

# 5. 탑 쌓기 (전체 통합 - 인식부터 실제 파지까지)
python realgrasp_pipeline.py
```

---

## 📂 파일 구성

| 파일 | 설명 |
|---|---|
| `connection_test.py` | ZMQ 통신 연결 테스트 |
| `vision_test.py` | 색상 검출 + 시각화 |
| `stereo_test.py` | 스테레오 삼각측량 → 3D 월드 좌표 |
| `kinematics_test.py` | simIK 기반 엔드이펙터 이동 |
| `stacking_test.py` | 탑 쌓기 FSM |
| `realgrasp_pipeline.py` | **비전 인식 + 실제 파지 전체 통합** |
| `explore_scene.py`, `gripper_probe.py`, `reset_targets.py` | 셋업/디버깅용 보조 스크립트 |

---

## 🔄 동작 흐름

```
스테레오 카메라 → 색상 검출 (u,v) → 삼각측량 (X,Y,Z)
        → 역기구학 (관절 각도) → FSM 제어 → 파지 & 피라미드 쌓기
```

---

## 📊 결과

| 항목 | 정확도 |
|---|---|
| 스테레오 3D 위치 추정 | 실제값 대비 오차 0.4 ~ 0.5 cm |
| IK 도달 정확도 | 0.1 ~ 0.5 cm |

자세한 구현 과정과 시행착오는 [REPORT.pdf](REPORT.pdf) 참고.
