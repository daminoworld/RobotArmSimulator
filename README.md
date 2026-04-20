# Robot Arm Simulator

산업용 6축 로봇팔의 Waypoint 기반 경로 시각화 및 IK(Inverse Kinematics) 추종을 학습 목적으로 구현한 Unity 시뮬레이터입니다.
산업용 XR 분야로 방향을 확장하면서 로봇 제어의 기본 원리를 직접 구현하며 이해하기 위해 진행한 개인 R&D 프로젝트입니다.

## 시연

<!-- 시연 영상 또는 GIF를 여기에 추가 -->
> 추후 시연 영상/GIF 추가 예정

## 핵심 기능

- **Waypoint 기반 경로 시각화** — JSON으로 정의된 Waypoint를 3D 공간에 마커와 경로선으로 렌더링
- **CCD / FABRIK / Jacobian DLS IK Solver 전환** — UI에서 세 알고리즘을 실시간으로 전환하며 동작 차이를 직접 비교. CCD(greedy 순차), FABRIK(axis-constrained 위치 전파), Jacobian DLS(전 관절 동시 최적화) 모두 구현
- **Joint Trajectory 재생** — CSV 기반 Joint Angle 직접 재생 모드와 Waypoint IK 보간 재생 모드 지원
- **좌표계 변환** — Right-handed 좌표계(ROS 등)에서 Left-handed Unity 좌표계로의 Basis 변환 및 Euler ZYX 회전 처리
- **UI Toolkit 기반 Control Panel** — Dataset 전환, 재생 제어, Waypoint 선택 및 상세 정보 표시

## 기술 스택

![Unity](https://img.shields.io/badge/Unity_6-000000?style=flat-square&logo=unity&logoColor=white)
![C#](https://img.shields.io/badge/C%23-239120?style=flat-square&logo=csharp&logoColor=white)
![UI Toolkit](https://img.shields.io/badge/UI_Toolkit-222222?style=flat-square&logo=unity&logoColor=white)

## 학습한 개념들

이 프로젝트를 통해 탐색하고 학습한 로보틱스/시뮬레이션 핵심 개념들입니다.

### Forward Kinematics vs Inverse Kinematics

- **FK (Forward Kinematics)**: 각 관절 각도가 주어지면 End-Effector 위치를 계산 — CSV 재생 모드에서 Joint Angle을 직접 적용하는 방식이 이에 해당
- **IK (Inverse Kinematics)**: 목표 위치/자세가 주어지면 이를 만족하는 관절 각도를 역으로 계산 — Waypoint 추종 시 CCD Solver가 이 역할을 수행

### CCD (Cyclic Coordinate Descent) 알고리즘

- End-Effector에서 Base 방향으로 한 관절씩 순회하며, 각 관절을 목표 방향으로 회전시키는 반복 알고리즘
- 본 프로젝트에서는 Joint 5 → Joint 0 순서로 역방향 순회하며, 관절축에 수직인 평면에 벡터를 투영한 뒤 `SignedAngle`로 회전량을 계산
- 프레임당 최대 12회 반복, 수렴 조건은 `sqrMagnitude ≤ 0.0025²`
- 회전 속도 제한(220°/s)을 두어 급격한 관절 움직임을 방지

### Wrist Orientation 제어

- 위치 수렴 후 마지막 3개 관절(Wrist)을 활용해 End-Effector의 자세(Orientation)를 추가 보정
- `Quaternion.Inverse`로 현재-목표 간 회전 차이(delta)를 구하고, `orientationBlend` 팩터(0.35)로 점진 적용

### 좌표계 변환 (Coordinate Frame Conversion)

- ROS 표준(Right-handed, X-Forward/Y-Left/Z-Up) ↔ Unity(Left-handed, X-Right/Y-Up/Z-Forward) 간 Basis 변환 행렬 구성
- `sourceToUnityBasis * sourceMatrix * sourceToUnityBasis⁻¹` 형태의 유사 변환(Similarity Transform) 적용
- Workpiece Frame 정의 시 Euler ZYX 순서(Z→Y→X)와 Euler XYZ 순서(X→Y→Z) 회전의 차이를 구분하여 처리

### FABRIK (Forward And Backward Reaching IK)

FABRIK은 체인을 위치 기반으로 푸는 알고리즘이다. 관절 각도를 직접 다루는 CCD와 달리, 각 관절의 목표 위치를 먼저 계산한 뒤 그 결과를 실제 관절에 적용한다.

**알고리즘 구조:**
1. **Backward pass** — End-Effector를 타겟에 배치하고, 루트 방향으로 각 관절 위치를 역전파
2. **Forward pass** — 루트를 원래 위치에 고정하고, 팁 방향으로 각 관절 위치를 순전파
3. 수렴 조건을 만족할 때까지 반복

**단일 회전축 관절에서의 FABRIK 적용 — CCD와의 핵심 차이:**

순수 FABRIK은 관절 제약이 없는 자유 체인을 가정한다. 실제 6축 로봇팔처럼 각 관절이 단일 회전축으로 제한된 경우, FABRIK이 계산한 목표 위치를 그대로 적용할 수 없다. 이 프로젝트에서는 다음 방식으로 처리했다:

- FABRIK position pass로 각 관절의 목표 방향 벡터를 계산
- 해당 방향을 관절의 회전축에 수직인 평면에 투영(Project on Plane)
- 투영된 방향 기준으로 `SignedAngle`을 계산해 `ApplyWorldDeltaToJoint` 호출

이 axis projection 단계에서 본래 회전량의 일부가 "실현 불가능한 방향"에 소모된다. 결과적으로 **CCD에 비해 프레임당 실효 회전량이 줄어들어 수렴 속도가 느려진다.**

**실측 비교 (동일 `maxDegreesPerSecond` 기준):**

| | CCD | FABRIK (axis-constrained) |
|---|---|---|
| 회전 방향 결정 기준 | 글로벌 타겟 (End-Effector → Target) | FABRIK 계산 중간 위치 (Joint i → Joint i+1 목표) |
| axis projection 손실 | 없음 (타겟 방향 = 최적 방향) | 있음 (목표 방향의 일부가 관절축에 수직) |
| 프레임당 수렴 속도 | 빠름 | 느림 (보상을 위해 maxDegreesPerStep을 높여야 함) |
| 체인 형태 | 탐욕적(Greedy) — 원단 관절 위주로 해결 | 전체 체인을 고려한 형태가 더 자연스러울 수 있음 |

**결론:** 단일 회전축 제약이 있는 직렬 로봇팔에서는 CCD가 수렴 속도 면에서 실질적으로 유리하다. FABRIK은 관절 제약이 없거나 느슨한 체인(캐릭터 IK 등)에서 속도 이점이 두드러진다.

**수렴 안정성 차이 — 실제로 관찰된 동작:**

CCD와 FABRIK을 동일한 Waypoint에 반복 적용했을 때 뚜렷한 차이가 나타났다.

- **CCD**: 매 실행마다 거의 동일한 경로로 수렴. 타겟 근처에서 안정적으로 정지.
- **FABRIK**: 실행마다 수렴 경로가 다르고, 타겟 근처에서 끝점이 진동(oscillation)하며 멈추지 못하는 현상 발생.

이 차이는 알고리즘의 구조적 특성에서 비롯된다.

**CCD가 안정적인 이유:**
각 관절이 매 iteration마다 "현재 End-Effector → 글로벌 타겟" 방향을 직접 계산해 회전한다. 항상 전역 목표를 직접 참조하기 때문에 수렴이 단조롭고(monotone) 예측 가능하다. 이전 실행의 관절 상태에 관계없이 동일한 방향으로 수렴한다.

**FABRIK이 불안정한 이유:**
FABRIK은 현재 체인 형태를 시작점으로 backward/forward pass를 수행하고, 그 결과로 얻은 중간 목표 위치를 axis-constrained 방식으로 적용한다.

- **수렴 보장 없음**: 축 제약 적용 후 re-snapshot한 위치가 FABRIK이 가정한 위치와 다르기 때문에, 다음 iteration의 계산 방향이 예측하기 어렵다. 특히 타겟 근처에서 이 오차가 반복적으로 반대 방향 보정을 유발해 진동이 발생한다.
- **초기 형태 의존성**: FABRIK의 중간 목표 위치는 현재 체인 형태에서 계산된다. 이전 실행에서 관절이 다른 자세로 남아 있으면 → 다음 실행의 시작 형태가 달라져 → 수렴 경로도 달라진다. CCD처럼 글로벌 타겟을 직접 참조하지 않기 때문에 나타나는 현상이다.
- **설계 전제의 한계**: FABRIK은 원래 관절 제약이 없는 자유 체인을 전제로 설계되었다. 단일 회전축 제약을 가진 직렬 로봇팔에 적용하면 axis projection 손실과 함께 이 불안정성이 더 두드러진다.

### Jacobian DLS (Damped Least Squares)

Jacobian IK는 "관절 속도 → End-Effector 속도" 관계를 행렬로 표현하고, 그 역연산으로 관절 각도 변화량을 계산한다.

**Jacobian 구성 (3×6):**

관절 i의 컬럼 = `axis_i × (tip − pivot_i)`

각 관절이 단위 각속도로 회전할 때 End-Effector가 이동하는 방향과 속도를 나타낸다. 이 관계를 행렬로 쌓으면 `ẋ = J θ̇` 가 성립한다.

**DLS 공식:**

```
Δθ = Jᵀ (JJᵀ + λ²I)⁻¹ Δx
```

- `Δx`: End-Effector 위치 오차 (3-벡터)
- `JJᵀ`: 3×3 행렬 → Cramer's rule로 해석적 역행렬 계산
- `λ` (damping factor): 특이점 근처에서 역행렬이 발산하는 것을 억제하는 감쇠 항

**CCD와의 핵심 차이 — 관절 이동 방식:**

CCD는 관절을 끝에서부터 하나씩 greedy하게 최적화한다. DLS는 모든 관절의 Δθ를 한 번에 계산해 동시에 적용하며, 수학적으로 `||Δθ||²`(관절 이동량의 제곱합)를 최소화하는 해를 산출한다. 그 결과 한 관절에 쏠리지 않고 체인 전체가 균등하게 기여하는 움직임이 나온다.

**속도 제한 방식의 차이:**

- CCD/FABRIK: 관절별 개별 clamp → 큰 관절만 잘라내어 DLS가 계산한 최적 방향을 왜곡
- DLS: Δθ 전체를 균일 스케일링 → 관절 간 비율(방향) 보존, pseudo-inverse의 해 품질 유지

**Singularity (특이점) 억제:**

Pseudo-inverse만 쓰면 특이점 근처에서 `JJᵀ`의 행렬식이 0에 가까워져 역행렬 원소가 폭발적으로 커진다. `λ²I`를 더하면 행렬식이 최소 `λ²` 이상이 보장되어 역행렬이 유계(bounded)된다. λ가 클수록 안정적이지만 수렴 속도가 낮아지는 trade-off가 있다.

### IK 알고리즘 비교

| 알고리즘 | 관절 처리 방식 | 장점 | 단점 |
|---------|-------------|------|------|
| **CCD** (구현) | 끝 관절부터 순차적으로, 글로벌 타겟 직접 추종 | 단순, 단일 축 제약 환경에서 수렴 빠름 | Greedy — 체인 전체 형태 최적화 어려움, 끝 관절 위주로 해결 |
| **FABRIK** (구현) | 위치 기반 backward/forward 전파 후 축 투영 적용 | 체인 전체를 고려한 형태 | 축 제약 투영 손실로 단일축 관절에서 CCD보다 느림, 수렴 불안정 |
| **Jacobian DLS** (구현) | 전 관절 동시 최적화, 감쇠 역행렬 | 특이점 안정, 관절 이동 최소화, 자연스러운 움직임 | λ 튜닝 필요, 행렬 연산으로 구현 복잡 |
| **Jacobian Pseudo-inverse** | DLS에서 λ=0인 특수 케이스 | 수학적으로 최적 해 | 특이점 근처에서 발산 |

## 프로젝트 구조

```
Assets/RobotArmSimulator/
├── Scripts/
│   ├── Camera/
│   │   ├── OrbitCameraController.cs   # Orbit/Pan/Zoom 카메라 제어
│   │   └── PointerInput.cs            # 마우스 입력 추상화
│   ├── Data/
│   │   ├── RobotTaskLoader.cs         # JSON/CSV 데이터 로딩 및 파싱
│   │   └── RobotTaskModels.cs         # 데이터 모델 정의
│   ├── Path/
│   │   └── PoseTrajectoryRenderer.cs  # Waypoint 마커 및 경로선 렌더링
│   ├── Playback/
│   │   └── JointTrajectoryPlaybackController.cs  # 재생 제어 (IK/Joint 모드)
│   ├── Robot/
│   │   ├── IIkSolver.cs               # IK 솔버 공통 인터페이스
│   │   ├── Simple6AxisIkSolver.cs     # CCD 기반 IK Solver
│   │   ├── FabrikIkSolver.cs          # FABRIK IK Solver (axis-constrained)
│   │   ├── JacobianDlsIkSolver.cs     # Jacobian DLS IK Solver
│   │   └── SimpleJointRobotVisualizer.cs  # 6축 로봇팔 시각화
│   ├── Transform/
│   │   └── CoordinateTransformUtility.cs  # 좌표계 변환 유틸리티
│   └── UI/
│       └── SimulatorUIController.cs   # UI Toolkit 기반 컨트롤 패널
├── UI/
│   ├── RobotArmSimulator.uxml        # UI 레이아웃 정의
│   └── RobotArmSimulator.uss         # UI 스타일시트
└── SampleData/
    ├── task_a_motion_editor_plane.json      # 평면 가공 Waypoint
    ├── task_b_motion_editor_cylinder.json   # 원통면 가공 Waypoint
    ├── task_c_coordinate_transform.json     # 좌표계 변환 테스트
    └── task_d_joint_trajectory.csv          # Joint Angle 궤적 데이터
```

## 실행 방법

1. **Unity 6** (6000.x 이상)로 프로젝트 열기
2. `Assets/Scenes/MainScene.unity` 씬 열기
3. Play 버튼으로 실행
4. 좌측 패널에서 Dataset 선택 후 Play/Pause/Stop으로 재생 제어
5. Waypoint 리스트에서 항목 클릭 시 해당 위치로 IK 추종
6. 마우스 우클릭 드래그: 카메라 회전 / 중클릭 드래그: 카메라 이동 / 스크롤: 줌

## 한계 및 다음 개선 방향

현재 이 프로젝트는 학습 범위를 "IK 기본 원리 이해 + 경로 시각화"로 한정했기 때문에, 아래 항목들은 인지하고 있지만 의도적으로 구현 범위에서 제외했습니다.

- **Joint Limit 미적용** — 실제 로봇은 각 관절마다 회전 가능 범위가 제한되어 있으나, 현재 구현에서는 제한 없이 회전 가능 (각도는 -180°~180°로 정규화하지만 물리적 제약은 미반영)
- **Singularity 처리 — DLS만 부분적으로 대응** — Jacobian DLS는 감쇠 항(λ)으로 특이점 근처의 발산을 억제하지만, CCD/FABRIK은 특이점 감지나 회피 로직이 없어 불안정해질 수 있음
- **Collision Detection 없음** — 로봇팔 링크 간 자기 충돌이나 환경 충돌 감지가 구현되지 않음
- **실제 하드웨어 연동 미구현** — ROS 연동이나 실제 로봇 제어 프로토콜은 이번 범위에 포함하지 않음
- **샘플 데이터 한정** — Waypoint 데이터는 학습용으로 AI 도구를 활용해 생성한 샘플이며, 실제 로봇 작업 데이터가 아님

## 개발 배경 및 방법론

산업용 XR 분야로 방향을 확장하면서, 로봇 제어의 기본 원리를 코드 레벨에서 이해할 필요성을 느꼈습니다. 논문이나 강의만으로는 체감하기 어려운 IK 알고리즘의 수렴 과정, 좌표계 변환의 실제 동작을 직접 시뮬레이터를 만들어보며 학습했습니다.

- AI 코딩 도구(Claude Code 등)를 적극 활용해 빠르게 프로토타입을 구현했습니다
- 알고리즘 원리 학습, 동작 검증, 파라미터 튜닝 및 디버깅은 직접 수행했습니다
- Waypoint 샘플 데이터는 AI 도구를 통해 생성했으며, 실제 로봇 데이터가 아닙니다

## License

MIT
