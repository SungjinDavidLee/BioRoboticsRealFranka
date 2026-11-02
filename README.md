# 실물 로봇 자동 시연으로 만든 데이터로 VLA(GR00T N1.7) 미세조정·실행하기 — 논문용 방법 정리

> - **대상 과제:** Franka FR3가 빨간 튜브를 집어 빨간 링으로 표시된 빈 랙 구멍에 꽂기 (지시문 “put the red tube in the red slot”)
> - **기준 코드:** GELLO 수집 스테이션 `gello_tools` (수집 2026-11-01, 버전 30p–30s / 실행 2026-11-02b), LeRobot 0.6.2, GR00T N1.7
> - **표기:** **【확인】** = 논문에 넣기 전에 [`paper_stats.py`](scripts/paper_stats.py) 출력이나 체크포인트 설정 파일로 확인할 값

## 저장소 구성

```text
.
├── README.md              이 문서 (방법 정리 전문)
├── figures/               본문에 들어간 그림 7장 (PNG)
├── media/                 VLA 실행 영상 — 본문용 GIF(2배속) + 실제 속도 MP4
└── scripts/
    └── paper_stats.py     논문용 데이터 통계 (로봇 PC에서 실행, 읽기만 함)
```

> [!NOTE]
> 수집 스테이션 코드(`gello_tools`: `gello_to_lerobot.py`, `groot_server.py`, `gello_rollout.py` 등)는 이 저장소에 들어 있지 않다. [`scripts/paper_stats.py`](scripts/paper_stats.py)는 로봇 PC의 `~/gello_tools`에서 순기구학(`gello_to_lerobot.fk`)과 보정 함수(`gello_scene.fit_homography`)를 불러오므로 로봇 PC에서 실행한다. 로봇을 움직이거나 파일을 고치지 않는다.

## 목차

- [0. 요약 (초록용)](#0-요약-초록용)
- [1. 시스템 구성](#1-시스템-구성)
- [2. 과제와 장면](#2-과제와-장면)
- [3. 자동 시연 데이터 생성](#3-자동-시연-데이터-생성)
- [4. 학습 데이터셋 변환 (LeRobot v3)](#4-학습-데이터셋-변환-lerobot-v3)
- [5. VLA 미세조정 (GR00T N1.7, LeRobot)](#5-vla-미세조정-gr00t-n17-lerobot)
- [6. 폐루프 실행 (롤아웃)](#6-폐루프-실행-롤아웃)
- [7. 초기 실행 관찰 (실행 영상 포함)](#7-초기-실행-관찰-2026-11-02-체크포인트-20k--예비-결과-n--2)
- [8. 논의와 한계](#8-논의와-한계)
- [9. 논문에 쓰기 전에 확인할 것](#9-논문에-쓰기-전에-확인할-것-확인-모음)
- [부록 A. 주요 파라미터](#부록-a-주요-파라미터) · [부록 B. 그림 설명 (영문)](#부록-b-그림-설명-영문-논문용) · [부록 C. 참고문헌](#부록-c-참고문헌-서지-형식은-투고처에-맞게)

---

## 0. 요약 (초록용)

사람이 원격조종으로 시연을 모으는 대신, **실물 로봇이 스스로 과제를 수행하는 자동 시연(scripted demonstration)** 을 반복 실행하며 녹화해 VLA 학습 데이터를 만들었다. 매 시행마다 사람이 물체 배치만 바꾸면, 홈 자세의 손목 카메라로 장면을 인식하고(색 분할), 미리 보정한 화면–로봇 변환으로 목표를 계산한 뒤, 최소 저크 궤적·감쇠 최소제곱 역기구학·관절 임피던스 제어로 실행한다. 궤적을 매번 새로 계획하고 실물에서 다시 실행하므로 영상과 동작이 모두 실제로 새로 생성된다. 기록 형식은 사람의 GELLO 원격조종 시연과 같다.

이렇게 모은 100시행(10 Hz, 34,209프레임)을 LeRobot 형식으로 변환해 GR00T N1.7을 부분 미세조정했다(시각·언어 백본 동결, 행동 헤드 등만 학습, 상대 행동). 실행할 때는 로봇 PC가 영상 2장과 손끝 상태를 원격 GPU 서버로 보내고, 서버가 돌려주는 16스텝(1.6 s) 행동 묶음 중 앞 8스텝을 역기구학으로 바꿔 같은 임피던스 컨트롤러로 실행하는 동기식 폐루프를 쓴다. 자동 시연기의 인식·보정은 **데이터를 만들 때와 평가 판정에만** 쓰이고, 실행 중인 정책에는 영상과 고유감각(손끝 상태)만 입력된다.

![그림 1. 전체 파이프라인](figures/fig1_pipeline.png)

**그림 1.** 전체 파이프라인. ① 실물 로봇에서 자동 시연 생성(사람은 배치만 바꿈) → ② LeRobot 데이터셋 변환(상태·행동 각 10차원) → ③ GR00T N1.7 부분 미세조정 → ④ 원격 추론 서버와 로봇 PC 사이의 동기식 폐루프 실행·평가.

---

## 1. 시스템 구성

| 구성 | 사양 |
|---|---|
| 로봇 | Franka Research 3 (7자유도) + Franka Hand (평행 그리퍼, 최대 폭 약 80 mm) |
| 카메라 | Intel RealSense D405 × 2 — 3인칭(삼각대 고정), 손목(핸드 장착). 848×480 RGB 30 fps, 깊이 미사용 |
| 제어 | ROS 2 Jazzy, `franka_ros2` 관절 임피던스 컨트롤러(libfranka 1 kHz). 스테이션이 관절 목표를 100 Hz로 발행 |
| 수집·실행 PC | Intel Core i7-9800X, Ubuntu (PREEMPT_DYNAMIC 커널을 실행 시 full 선점으로 전환, 실시간 우선순위·CPU 깊은 절전 끔) |
| 학습·추론 서버 | NVIDIA RTX PRO 6000 Blackwell × 2 (로봇 PC와 SSH 터널로 연결) |
| 소프트웨어 | 수집 스테이션(자체 개발, GELLO 기반 웹 UI), LeRobot 0.6.2, GR00T N1.7 (`nvidia/GR00T-N1.7-3B`) |

- 사람 원격조종 장치 GELLO(리더 암)는 **화면–로봇 보정점을 기록할 때만** 썼다. 데이터 수집은 리더 암 없이(리더리스 모드) 자동 시연으로만 했다 【확인: 출처 `auto` 100%】.
- 실시간성 조치(재부팅 없음): 커널 선점 full, 제어 스레드 우선순위 재정렬(초당 500회 넘게 깨어나는 실시간 스레드를 최상위로, 물려받은 우선순위로 올라간 저빈도 스레드는 일반 스케줄로), `/dev/cpu_dma_latency` 5 µs로 C6 진입 차단. libfranka `communication_constraints_violation`으로 인한 컨트롤러 정지를 줄이기 위한 것이다.

## 2. 과제와 장면

- **지시문(전 시행 동일):** “put the red tube in the red slot”.
- **장면:** 4구 랙 2개. 원래 랙에는 빨간 튜브 1개(파란 뚜껑 + 빨간 몸통)와 다른 색 튜브(방해물)가 꽂혀 있고, 빈 랙의 4구멍 중 하나에 빨간 고무 링을 둘러 표시한다. 로봇은 빨간 튜브를 집어 빨간 링 구멍에 꽂는다(색 맞추기는 영상으로 학습).
- **튜브·랙:** 뚜껑 지름 약 32 mm(쥔 폭 실측 31~34 mm). 튜브와 구멍 사이 여유가 작아(입구를 넓혀 사용) 수 mm 정밀도가 필요하다.
- **배치 변화:** 랙 위치·방향, 빨간 튜브 칸, 빨간 링 칸, 방해 튜브 위치를 바꿨다. 같은 배치(랙 자리)로 약 10시행씩 모은 뒤 자리를 옮겼다(배치 안에서도 튜브 칸·링 칸을 바꿈) 【확인: 서로 다른 (잡을 칸, 꽂을 칸) 조합 수】.

---

## 3. 자동 시연 데이터 생성

### 3.1 설계 원칙

1. **복제가 아니라 재실행.** 녹화된 시연을 복제하면 영상이 같아 새 정보가 없다. 배치마다 동작을 새로 계획하고 실물에서 다시 실행하며 새로 녹화한다(시뮬레이션 기반 데이터 증식인 MimicGen과 달리 렌더링 차이가 없다).
2. **같은 명령 통로·같은 기록 형식.** 계획한 관절 궤적을 사람 시연 때 GELLO 리더 신호가 들어가던 자리에 넣어 같은 임피던스 컨트롤러로 보낸다. 로봇의 물리(임피던스 처짐·지연)와 기록 형식이 사람 시연과 같다.
3. **학습된 모델 없음.** 모든 단계가 고전적인 영상 처리·기하·최소제곱·다항식 궤적·역기구학·임피던스 제어다.
4. **사람의 역할은 배치 바꾸기뿐.** 실행·판정·저장·홈 복귀는 스테이션이 한다.

한 시행의 순서(그림 2): ① 장면 인식 → ② 화면→로봇 좌표 변환 → ③ 목표 계산 → ④ 궤적 생성 → ⑤ 역기구학 → ⑥ 임피던스 제어로 실행·기록 → ⑦ 품질 판정·저장 → 홈 복귀.

**그림 2. 자동 시연의 원리** — (a)는 3.2절, (b)는 3.4절, (c)·(d)는 3.5절에 있다.

### 3.2 장면 인식 (홈 자세 손목 카메라)

손목 카메라는 항상 같은 홈 자세에서 찍으므로 화면과 책상 평면의 관계가 고정된다. 약 1.2 s 동안 최대 5장을 찍어 **각 장을 따로 검출한 뒤 과반수(⌊n/2⌋+1) 이상에서 나온 물체만** 인정하고 위치는 중간값을 쓴다(빛·노출 변화로 인식이 뒤집히는 것을 막음). 입력은 3×3 가우시안 후 HSV(OpenCV, H 0–179)로 바꾸고, 화면 아래 10%(로봇 받침대)는 무시한다. 크기 기준은 화면 폭에 비례(k = W/640).

<p align="center"><img src="figures/fig2a_perception.png" width="70%" alt="그림 2(a). 장면 인식과 목표"></p>

**그림 2(a).** 홈 자세 손목 카메라 영상에서 뚜껑·빨간 튜브·랙·빨간 링을 찾고 목표를 정함(손가락 영역은 옆 뚜껑 충돌 검사용).

| 대상 | 색 조건 (HSV) | 형태 처리·판정 |
|---|---|---|
| 뚜껑 (파랑) | H 105–125, S ≥ 110, V ≥ 60 | 닫힘 5×5·열림 3×3 → 연결 요소(넓이 120k²–6000k² px) → 최소 외접원, 충전율 A/(πr²) ≥ 0.45, 종횡비 0.5–2. 붙은 덩어리는 거리 변환의 극대점마다 하나로 분할 |
| 빨간 튜브 | 몸통: H ≤ 10 또는 ≥ 165, S ≥ 80, V ≥ 50 | 각 뚜껑 중심 반지름 2.2r 안의 빨간 픽셀 수가 가장 많고 다음 후보의 3배 이상인 뚜껑 |
| 랙 (청록) | H 86–102, S ≥ 150, V ≥ 50 | 닫힘 9×9 → 큰 덩어리(최대의 25% 이상)마다 최소 넓이 사각형 → 중심·긴 축 방향 |
| 빨간 링 표시 | (H ≥ 140 & S ≥ 40) 또는 (H ≤ 5 & S ≥ 110), V ≥ 50 | 랙 사각형(24k px 확장) 안에서만, 뚜껑 반지름 2.4배 안은 제외 → 충전율 < 0.55인 **링 모양**만, 잡을 튜브에서 3.5r 이상 떨어진 것. 감싸는 원의 중심 = 꽂을 구멍. 빨간 튜브와 같은 랙이면 인식 오류로 보고 거부 |

- 링 표시 안에 보이는 작은 파란 덩어리(빈 구멍 바닥 반사)는 뚜껑에서 뺀다(가장 큰 뚜껑의 0.9배 미만 + 링 중심 근처).

### 3.3 화면 → 로봇 좌표 보정

사람이 GELLO로 튜브를 실제로 쥐는 자세를 5~8회 기록해 (화면의 뚜껑 중심 u, v) ↔ (그리퍼 끝 x, y, z) 쌍을 얻는다. 두 모델을 최소제곱으로 맞춘다.

- 아핀(6개 값): `[x y]ᵀ = A [u v 1]ᵀ`
- 호모그래피(8개 값, 점 6개 이상): `[x y 1]ᵀ ∝ H [u v 1]ᵀ` (OpenCV, 모든 점 최소제곱)

점을 하나씩 빼고 맞춘 변환으로 뺀 점을 예측했을 때의 **최대 오차(LOO)가 작은 모델**을 고른다. 잡는 높이 z_g는 기록한 점들의 그리퍼 높이 **중앙값**이다. 홈 자세를 바꾸면 다시 보정한다.

【확인: 이 세션의 점 수·선택된 모델·rms·LOO 최대 — `paper_stats.py` 4절. 예전 바구니 과제 세션의 실물 8점 보정은 아핀, rms 3.2 mm였다.】

### 3.4 목표 계산

- **잡을 점:** (x_g, y_g) = H(u_c, v_c).
- **그리퍼 방향(yaw):** 기본은 손목(7번 관절)을 돌리지 않고 팔 전체가 옆으로 도는 만큼만 따라 도는 방향이다.

  ```math
  \psi_{\mathrm{base}} = \psi_{\mathrm{home}} + \mathrm{wrap}\left(\mathrm{atan2}(y_g,\, x_g) - \mathrm{atan2}(y_{\mathrm{home}},\, x_{\mathrm{home}})\right)
  ```

  손가락 두 개를 폭 20 mm × 두께 10 mm 사각형으로 보고, 그리퍼 y축 방향으로 중심에서 ±(벌린 폭/2 + 5 mm)에 둔다 — 잡으러 내려갈 때는 완전히 벌린 상태(±45 mm), 꽂을 때는 쥔 폭 30 mm + 벌림 12 mm(±26 mm). 다른 뚜껑 중심에서 사각형까지의 거리가 뚜껑 반지름 16 mm + 여유 6 mm보다 작으면 닿는다고 본다. 닿으면 **손가락이 그 랙의 줄과 수직이 되게**(그리퍼 x축 ∥ 랙 긴 축) 돌린다. 그리퍼는 180° 대칭이므로 덜 도는 쪽을 고른다. 그래도 닿으면 계획을 거부한다. 손목 회전을 최소화하는 이유는 7번 관절이 임피던스 제어에서 명령을 시간상수 약 1 s로 늦게 따라가기 때문이다.
- **꽂을 구멍 — 원근 보정:** 보정 H는 뚜껑 윗면 높이의 평면에 대한 것이다. 구멍(랙 윗면)은 그보다 Δh 낮아 화면에서 카메라 연직점 C 쪽으로 당겨져 보인다(그림 2b). 이를 되돌린다.

  ```math
  p = C + \left(H(u_r) - C\right)\,\frac{D + \Delta h}{D},\qquad
  D = f_x\, m,\qquad
  f_x = \frac{W/2}{\tan(87^\circ/2)} \approx 447\ \mathrm{px},\qquad
  m = \sqrt{\left| \det J_H(c) \right|}
  ```

  f_x는 D405 가로 화각으로 구한 초점거리, m은 화면 중심의 m/px, C = H(화면 중심)이다. Δh(뚜껑 윗면 − 랙 윗면)는 자로 잰 값을 설정한다 【확인: 세션 `cap_above`】.

  <p align="center"><img src="figures/fig2b_parallax.png" width="70%" alt="그림 2(b). 구멍 위치의 원근 보정"></p>

  **그림 2(b).** 보정 평면(뚜껑 윗면)보다 Δh 낮은 구멍의 원근 보정.
- **(선택) 꽂은 기록으로 구멍 위치 바로잡기:** 저장된 꽂기 시행에서 튜브가 실제로 들어간 위치(놓는 순간 손끝, 구멍 테두리에 밀려 들어가므로 구멍 중심 근처)를 정답으로, 계산값에 더할 보정량을 링 화면 위치의 1차식 `[1, Δu/100, Δv/100]·W`로 릿지 회귀한다(기울기 λ = 2). 하나 빼고 맞춘 오차가 기존의 0.8배 미만이고 0.5 mm 넘게 나아질 때만, 보정량 ≤ 10 mm일 때만, 6시행 이상일 때 쓴다 【확인: 적용된 시행 수 — `paper_stats.py` 2절】.
- **꽂을 때 방향:** 같은 규칙(쥔 폭 30 mm + 벌림 12 mm로 충돌 검사).

### 3.5 궤적 생성

**웨이포인트**(z_g: 잡는 높이, 랙→랙에서는 잡는 높이 조정 grasp_dz를 잡기·놓기 높이에 같이 더함):

| 점 | 위치 |
|---|---|
| 홈 | 홈 자세의 TCP |
| 접근 | (x_g, y_g, z_g + a),  a = 0.08 + U(−0.02, 0.02) m |
| 잡기 | (x_g, y_g, z_g) |
| 들기 | (x_g, y_g, z_g + l),  l = 0.15 + U(−0.02, 0.02) m |
| 호 경유 | (들기 + 칸 위)/2 + (U(−1,1)·0.025, U(−1,1)·0.025, h),  h = U(0,1)·0.04 m |
| 칸 위 | (x_h, y_h, z_g + l) |
| 놓기 | (x_h, y_h, z_g + 0.01) — 원래 꽂혀 있던 깊이보다 1 cm 위에서 놓음 |
| 빠지기 | 칸 위 + 0.05 m |

**연속 경로.** 시행을 3개 동작으로 나눈다: ① 홈→접근→수직 하강, ② 들기→호→칸 위→수직 하강, ③ 위로 빠지기. 각 동작의 꺾은선에서 안쪽 꼭짓점 b를 반지름 r(①: 3 cm, ②: 4 / 6 / 4 cm, 단 r ≤ 0.45 × 인접 변 길이)로 2차 베지어로 둥글게 만든다(p₁, p₂: 꼭짓점에서 r만큼 떨어진 두 변 위의 점).

```math
B(u) = (1-u)^2\, p_1 + 2(1-u)\,u\, b + u^2\, p_2
```

수직으로 넣고 빼는 구간은 끝까지 수직이다. 둥글린 경로를 호 길이 비율 s ∈ [0, 1]로 매개화하고, 동작 하나에 **최소 저크 다항식 하나**를 쓴다(양 끝 속도·가속도 0, 최고 속도 = 평균의 1.875배; Flash & Hogan, 1985).

```math
s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5,\qquad \tau = t/T
```

따라서 한 동작 안에서는 멈추지 않고, 멈추는 곳은 쥐기·놓기·끝뿐이다.

![그림 2(c)·(d). 궤적 측면과 최소 저크 시간 진행](figures/fig2cd_trajectory.png)

**그림 2(c)·(d).** (c) 측면에서 본 궤적 — 실제 계획 식(꼭짓점 2차 베지어, 호 길이 최소 저크)으로 계산. (d) 최소 저크 시간 진행 s(τ)와 속도.

**시간.** 수평 이동 v = 0.12 m/s, 수직 v = 0.06 m/s, 속도 배율 k(기본 0.5, 0.2–1.5로 제한), ε ~ U(−0.15, 0.15) 【확인: 실제 k·ε 분포 — `speed`】.

```math
T = \max\!\left(1.5\ \mathrm{s},\ \frac{\sum_i d_i / v_i}{k\,(1+\varepsilon)}\right)
```

**자세.** 수직 축 회전만 쓴다. `R(u) = exp(u · log(R₁R₀ᵀ)) R₀`, u는 지정 구간(rot_span) 안에서 smoothstep. 랙→랙 옮기기에서는 튜브를 5 cm 들어 올린 뒤부터 꽂기 5 cm 전까지만 회전해 손가락이 옆 뚜껑을 스치지 않게 한다.

**(선택) 칸 위 맞춤.** 놓는 높이 + 6 cm에서 1 s 동안 정착 보정(3.7절)을 한 뒤 수직으로 꽂는다. 100시행 중 21시행에 켜져 있었다(이후 원근 계산 오차가 맞춤으로 오히려 드러나 끔).

**무작위화 요약.** 접근 높이 ±2 cm, 들기 높이 ±2 cm, 호 높이 0–4 cm, 경유점 수평 ±2.5 cm, 속도 ±15%.

### 3.6 역기구학

궤적을 0.05 s 간격으로 샘플하고, 각 샘플의 목표 자세 (p\*, R\*)를 직전 해에서 시작하는 감쇠 최소제곱으로 관절각 q로 바꾼다(최대 40회 반복, 위치 허용 오차 2×10⁻⁵ m).

```text
e  = [ p* − p(q) ;  log(R* R(q)ᵀ) ]                 (6차원: 위치 + 회전 벡터)
J  = ∂(p, R)/∂q   (6×7, 수치 미분 Δq = 10⁻⁶)
J⁺ = Jᵀ (J Jᵀ + λI)⁻¹,  λ = 10⁻⁵                      (Wampler 1986; Nakamura & Hanafusa 1986)
q ← q + J⁺e + (I − J⁺J) · 0.02 · (q_home − q)          (영공간 항: 여유 1자유도를 홈 자세 쪽으로; Liégeois 1977)
```

남은 오차가 1 mm 또는 0.5°를 넘으면 계획을 거부한다. 관절이 한계에서 0.05 rad 안쪽인지 검사하고, 관절 속도가 상한(1–6번 0.8 rad/s, 7번 0.15 rad/s)을 넘으면 그 동작의 시간을 `T ← T · (최대 비율) · 1.1`로 늘려 최대 4번 다시 계획한다.

### 3.7 실행 — 관절 임피던스 제어

- 스테이션의 명령 중계가 계획된 관절 궤적을 **100 Hz로 선형 보간**해 관절 목표 q_cmd(t)로 발행하고, `franka_ros2` 관절 임피던스 컨트롤러가 1 kHz로 따라간다(관절마다 스프링·댐퍼처럼 τ ≈ K(q_cmd − q) − Dq̇ + 중력 보상; Hogan 1985). 강성이 유한해 실제 자세는 명령보다 조금 처지고 늦다 — 사람 원격조종 때와 같은 물리다.
- **정착 보정(적분):** 잡기 직전 0.5 s 동안 남은 관절 오차를 적분해 명령을 조금 더 민다(|off| ≤ 6°/관절, 최대 오차 8° 미만일 때만).

  ```math
  \mathrm{off} \leftarrow \mathrm{off} + \mathrm{clip}\left(2.0\,(q_{\mathrm{goal}} - q)\,dt,\ \pm 4^\circ/\mathrm{s}\cdot dt\right)
  ```

  홈에서는 같은 방식(이득 1.0 s⁻¹, 2°/s, 6°)으로 맞춰 둔 보정량을 출발할 때 이어 붙여 시작점이 끊기지 않게 한다. 이 보정은 **명령(q_cmd)에 들어가 그대로 기록**되므로, 학습 행동(명령)은 처짐 보정이 포함된 값이다.
- **그리퍼(Franka Hand):** 쥐기는 Grasp 동작(목표 폭 0, 허용 오차 ±0.08 m, 힘 70 N 【확인: 세션 설정 — 기본 40 N】, 속도 0.1 m/s)이라 물체 폭과 상관없이 힘으로 쥔다. 놓기는 옆 튜브에 손가락이 닿지 않으면 바로 최대 폭까지 Move, 닿으면 쥔 폭 + 12 mm만 벌린 뒤 위로 빠지고 나서 완전히 연다. 쥐기·놓기는 그리퍼 동작이 끝났다고 응답할 때까지 기다린 뒤 다음 동작으로 간다.

### 3.8 기록과 품질 판정

- **10 Hz 기록:** 3인칭·손목 영상(JPEG), 관절 실측 q·속도, 명령 q_cmd, TCP 위치·자세, 손가락 폭, 리더 자리의 명령값(`leader_q = q_cmd`), 리더 그리퍼 자리의 그리퍼 명령(1 열기 / 0 닫기), 각 신호의 수신 지연. `meta.json`에 `source: auto`, 계획값·무작위 seed·장면 사진·카메라 노출값이 남는다.
- **품질 판정(사람 시연과 같은 기준):** ‘나쁨’이면 자동으로 휴지통에 간다. 나쁨 조건은 길이 < 3 s, 기록 주기 < 목표의 85%, 입력 끊김 > 5% 프레임, 관절·TCP 값 누락, 카메라 멈춤(같은 영상 5프레임 연속), 시작 자세가 홈에서 10 mm·5° 넘게 벗어남, 파지 없음·헛잡음·놓지 않고 끝남, 리더(명령)–로봇 차이 > 20°, 움직임 없음, 포화 픽셀 > 15%다. 주의(저장은 함)는 밝기가 기준과 15 넘게 다름, 포화 > 5%, 중간 멈춤 등이다.
- 그리퍼 폭으로 파지·놓기를 판정할 때 그리퍼 불감시간(명령 → 폭 변화 시작, 실측 중앙값 1.82 s)을 고려해 폭 변화 전후 구간은 ‘대기’로 본다.

### 3.9 수집 결과

| 항목 | 값 |
|---|---|
| 저장한 시행 | **100** (자동 시연 100%) 【확인: 판정 좋음/주의 수, 휴지통 수 → 저장 비율】 |
| 칸 위 맞춤 켠 시행 | 21 / 100 |
| 잡는 높이 조정 | 앞 약 30시행 0 mm, 이후 +2 mm 【확인】 |
| 잡은 자리 (닫는 순간 TCP) | x 458–655 mm, y −210–274 mm, z 102.9–115.0 mm (중간 109.9) |
| 쥘 때 그리퍼 각도 (바닥 기준) | −31° ~ +21° (중간 0°) |
| 쥔 폭 (닫고 2 s 뒤) | 중간 40 mm (최종 31~34 mm) |
| 닫기 명령 → 5 mm 들기 시작 | 중간 6.1 s (5.2–7.0) — 그리퍼 응답 대기 + 최소 저크의 느린 출발 |
| 수집 날짜·서로 다른 배치 수·시행 길이 | 【확인: `paper_stats.py` 1·2절】 |
| 자동 시연 정확도 (쥐기·놓기 위치 실제 − 계획) | 【확인: `paper_stats.py` 3절】 |

---

## 4. 학습 데이터셋 변환 (LeRobot v3)

변환기 `gello_to_lerobot.py`는 원본을 그대로 두고 새 데이터셋을 만든다.

| 특징 | 차원 | 정의 |
|---|---|---|
| `observation.state` | 10 | [x, y, z, r6d(6), w] — **관절 실측** q의 순기구학(fr3_link0 → fr3_hand_tcp), w: 실제 손가락 폭(m) |
| `action` | 10 | [x, y, z, r6d(6), g] — **명령 관절** q_cmd의 순기구학, g: 그리퍼 명령 1 = 열기 / 0 = 닫기 |
| `observation.images.third`, `.wrist` | 480×848×3 | 3인칭·손목 RGB (데이터셋 안에서는 영상 파일로 인코딩) |
| `task` | — | “put the red tube in the red slot” |

- **rot6d:** 회전 행렬의 첫 두 열 [R₀₀ R₁₀ R₂₀ R₀₁ R₁₁ R₂₁] (Zhou et al., 2019). 그리퍼가 아래를 볼 때 roll ≈ ±180°라 오일러각은 불연속이고, 상대 행동(행동 − 상태)을 칸별로 빼면 튀기 때문이다. 상태와 행동의 순서·길이를 같게 맞췄다(LeRobot 상대 행동은 같은 칸끼리 뺌).
- **행동 = 명령:** 실측이 아니라 보낸 명령을 행동으로 쓴다. 명령에는 정착 보정(처짐 보정)이 들어 있으므로, 실행 때 같은 컨트롤러로 이 값을 보내면 수집 때와 같은 곳에 간다.
- **10 Hz 격자:** 기록 시각 t 기준으로 0.1 s 간격의 고정 격자를 만들고, 각 격자 시각에 가장 가까운 **유효 프레임**(관절·명령·영상 2장이 모두 있음)을 고른다. 손가락 폭·그리퍼 명령의 빈 값은 앞 값으로 채운다.
- **칸 위 정지 구간 제거:** 칸 위 맞춤 시행에서 튜브를 쥔 채 맞추는 자리 6 mm 안에서 6 mm/s 미만으로 거의 서 있는 프레임을 잘라냈다(연속 구간의 첫·끝 프레임은 남김). 21시행에서 239프레임(시행당 약 11.4), 잘라낸 곳 앞뒤 명령 위치 차이 최대 7.6 mm. 모델이 데이터 수집용 ‘멈춤’을 배우지 않게 하려는 것이다. 쥔 뒤 그리퍼 응답을 기다리는 정지(약 4 s)는 이번 데이터셋에서 잘라내지 않았다.
- **제외:** 판정 ‘나쁨’ 시행(이미 자동 폐기), 다른 놓는 곳(바구니) 시행.
- **결과:** 100시행, 격자 34,448프레임 → 제거 후 **34,209프레임**(약 57분, 시행당 평균 342프레임 = 34.2 s). 상태 범위 x 0.43–0.67, y −0.21–0.28, z 0.10–0.31 m, 손가락 폭 24–80 mm, 그리퍼 yaw 약 −32° ~ +22°.

---

## 5. VLA 미세조정 (GR00T N1.7, LeRobot)

**모델.** GR00T N1.7 (기본 모델 `nvidia/GR00T-N1.7-3B`, 임베디먼트 `new_embodiment`). LeRobot 0.6.2 구현의 공개 N1.7 설정 기준 구조는 다음과 같다 【확인: 체크포인트 `config.json`】.

- 시각·언어 백본: Cosmos-Reason2-2B (Qwen3-VL 계열), 언어 모델 16층까지의 특징 사용(`select_layer = 16`), 출력 2048차원 — **동결**
- VL 자기주의 4층 + LayerNorm(`vlln`) — **학습**
- 임베디먼트별 상태 인코더·행동 인코더·행동 디코더(`projector`) — **학습**
- 행동 헤드: DiT 32층(어텐션 32헤드 × 48차원, AdaNorm), **flow matching**(Lipman et al., 2023)으로 행동 묶음 생성, 추론 4단계 적분 — **학습**
- 행동 묶음 길이 16(1.6 s), 학습 중 상태 드롭아웃 0.2(공개 설정값)

**상대 행동.** 행동 묶음의 각 칸에서 현재 상태를 뺀 값(위치·rot6d 9차원)으로 정규화·학습하고, 그리퍼 g는 절대값으로 둔다(`use_relative_actions = true`, `relative_exclude_joints = ["gripper"]` — 이름에 ‘gripper’가 든 차원 제외). 상대 행동 통계는 학습할 때 데이터셋에서 자동 계산된다. 추론 후 후처리에서 역정규화하고 상태를 다시 더해 절대값으로 되돌린다.

**학습 설정.**

| 항목 | 값 |
|---|---|
| 데이터 | 100시행 전부 (검증용으로 빼지 않음 — 체크포인트 3개를 실물로 비교해 고름) |
| 스텝 | 20,000 (`--steps = --policy.max_steps = 20000`) |
| 배치 | GPU당 32 × 2 = 64 → 약 37.4 epoch (20,000 × 64 / 34,209) |
| 최적화 | AdamW, lr 1×10⁻⁴, β (0.9, 0.999), ε 10⁻⁸, weight decay 10⁻⁵, gradient clip 1.0 |
| 학습률 일정 | 코사인, 예열 5% (1,000스텝) |
| 정밀도 | bf16 |
| 영상 증강 | LeRobot 기본: 밝기 0.8–1.2, 대비 0.8–1.2, 채도 0.5–1.5, 색조 ±0.05, 선명도 0.5–1.5, 아핀(±5°, 이동 5%) 중 최대 3개 |
| 체크포인트 | 7,000 / 14,000 / 20,000 스텝 (`--save_freq = 7000`) |
| 하드웨어 | RTX PRO 6000 Blackwell × 2 (`accelerate launch --multi_gpu`) |
| 마지막 학습 손실 | 약 0.007–0.008 (flow matching 손실) |
| 학습 시간 | 【확인: 체크포인트 폴더 시각】 |

```bash
cd ~/lerobot && uv run accelerate launch --multi_gpu --num_processes=2 -m lerobot.scripts.lerobot_train \
  --dataset.repo_id=local/medbio_redslot_v1 --dataset.root=$HOME/lerobot_data/medbio_redslot_v1 \
  --dataset.image_transforms.enable=true \
  --policy.type=groot --policy.device=cuda --policy.base_model_path=nvidia/GR00T-N1.7-3B \
  --policy.embodiment_tag=new_embodiment --policy.chunk_size=16 --policy.n_action_steps=16 \
  --policy.use_relative_actions=true --policy.relative_exclude_joints='["gripper"]' \
  --policy.use_bf16=true --policy.push_to_hub=false \
  --batch_size=32 --steps=20000 --policy.max_steps=20000 --save_freq=7000 --use_policy_training_preset=true \
  --env_eval_freq=0 --eval_steps=0 --log_freq=50 --num_workers=8 --seed=42 \
  --output_dir=outputs/train/medbio_redslot_v1_groot17 --job_name=medbio_redslot_v1_groot17
```

---

## 6. 폐루프 실행 (롤아웃)

**그림 3. 폐루프 실행** — (a)·(b)는 6.2절, (c)는 6.3절에 있다.

### 6.1 구성

- **추론 서버** (`groot_server.py`): 체크포인트 하나를 GPU에 올려 두고 HTTP(`/act`)로 요청을 받는다. 로봇 PC는 SSH 터널(`ssh -N -L 5555:127.0.0.1:5555 …`)로 접속한다. 한 요청 처리 순서: JPEG 해독 → `prepare_observation_for_inference` → 전처리(정규화, 상태 기억) → `predict_action_chunk` → 후처리(역정규화 + 상태 더하기) → 16 × 10 절대 행동. 상대 행동 정책이라 LeRobot의 `select_action`(한 스텝씩 꺼내기) 대신 묶음 전체를 한 번에 후처리한다.
- **스테이션** (`gello_rollout.py`): 관측·안전 검사·역기구학·실행·기록·판정.

### 6.2 한 주기 (동기식)

![그림 3(a). 한 주기 — 동기식 행동 묶음 실행](figures/fig3a_cycle.png)

**그림 3(a).** 한 주기: 로봇이 정지한 상태에서 관측 → 서버 추론 → 행동 묶음 처리 → 앞 8칸(0.8 s) 실행.

1. **관측:** 3인칭·손목 영상(JPEG 품질 90, 848×480) + 손끝 상태 s_t = [관절 실측의 FK 위치, rot6d, 손가락 폭] — 학습 데이터와 같은 식.
2. **추론:** 서버가 행동 묶음 Â = [â₀ … â₁₅] (0.1 s 간격, â₀ ≈ 지금)를 돌려준다. 추론하는 동안 로봇은 마지막 목표에 정지해 있다. 왕복 시간(영상 인코딩·터널 포함) 실측 중간값 184–189 ms (2회) 【확인: 서버 추론 시간 — `paper_stats.py` 6절】.
3. **그리퍼 사건:** 현재 열림이면 g < 0.4, 닫힘이면 g > 0.6이 **두 칸 연속**인 첫 칸 k를 찾는다(0.5 근처 깜빡임·한 칸짜리 튐 무시).
4. **실행:** â₁ … â_E (E = 8, 사건이 있으면 k까지)의 손끝 자세를 역기구학(3.6절과 같은 식, 홈 쪽 영공간)으로 관절 목표로 바꾸고, 수집 때와 같은 명령 중계(100 Hz 선형 보간)로 0.1 s 간격으로 실행한다. 학습 행동이 이미 ‘보낸 명령’이므로 정착 보정을 더하지 않는다(raw). 관절 속도가 1 rad/s를 넘는 칸만 시간을 늘린다.
5. **쥐기/놓기:** 사건이 있으면 k까지 움직인 뒤 그리퍼 명령(쥐기: Grasp, 놓기: 최대 폭 Move)을 보내고 끝날 때까지 기다린 다음 1로 돌아간다.

<p align="center"><img src="figures/fig3b_chunk.png" width="80%" alt="그림 3(b). 행동 묶음 처리 — 그리퍼 사건이 있는 주기"></p>

**그림 3(b).** 그리퍼 사건이 있는 주기: 문턱(0.4/0.6)을 두 칸 연속 넘는 첫 칸 k까지만 실행하고 쥐기/놓기 후 다시 관측.

### 6.3 안전장치 (정책의 움직임을 대신 정하지 않음)

위험할 때만 그 자리에 정지하고 실행을 끝낸다: 작업 범위(x 0.33–0.78, y −0.34–0.40, z 0.085–0.45 m) 밖, 그리퍼가 수직 아래에서 25° 넘게 기울어짐(학습 데이터 최대 약 3°), 0.1 s에 5 cm 넘게 움직임(학습 데이터 최고 속도 약 0.12 m/s), 역기구학 잔차 2 mm 또는 0.02 rad 초과, 관절 한계 0.02 rad 이내. 유일한 수정은 바닥 높이 제한(z < 0.09 m인 목표를 0.09 m로 올리고 횟수를 기록; 학습 데이터 최저 0.10 m)이다.

<p align="center"><img src="figures/fig3c_safety.png" width="65%" alt="그림 3(c). 안전장치·종료·판정"></p>

**그림 3(c).** 안전장치와 종료 조건(종료·판정 규칙은 6.4절).

### 6.4 종료와 판정 — 평가 프로토콜

- **종료:** 쥐었다가 놓은 뒤 2.5 s / 최대 시간 / 사람이 Space(멈춤) / 컨트롤러 꺼짐. GR00T는 ‘끝났다’는 신호를 내지 않으므로 종료 규칙은 평가 프로토콜의 일부이며, **모든 체크포인트·시행에 같은 값**을 쓰고 보고한다. 권장: 최대 시간 120 s(학습 시행 평균 34 s의 약 3.5배), 한 번에 실행할 칸 E 고정.
- **판정 기준:** 성공 = 빨간 튜브를 빨간 링 구멍에 꽂고 시간 안에 놓음. 부분 = 튜브를 쥐었지만 성공 못 함(들기·옮기기·꽂기 중 실패, 또는 꽂았지만 시간 안에 못 놓음). 실패 = 쥐지 못함. 무효 = 잘못 시작한 시행 등(통계에서 빼고 사유를 보고). 성공률은 Wilson 95% 신뢰구간과 함께 보고한다.
- **자동 판정 (스테이션 2026-11-02b, 사람 확인·수정 가능):** 실행을 시작할 때 홈 자세 손목 카메라로 장면을 인식해(3.2절 인식 재사용) 빨간 튜브와 빨간 링 구멍 위치를 기록한다. 구멍 위치는 보정 H와 원근 보정(3.4절)만으로 계산하고, 정책에는 주지 않는다. 실행 기록의 손끝 궤적·손가락 폭·그리퍼 명령으로 단계를 판정한다 — 잡기(닫고 3 s 뒤 폭 20–50 mm), 들기(쥔 채 3 cm 이상 상승; 폭이 12 mm 미만이 되면 낙하), 칸 위(쥔 채 구멍에서 수평 20 mm 안), 꽂기(구멍 14 mm 안에서 학습 시행의 놓은 높이 중앙값 + 12 mm 아래까지 하강), 놓기. 놓은 뒤 끝난 시행은 놓은 높이 + 5 cm까지 수직으로 뺀 뒤 홈으로 돌아와 끝 장면을 인식하고, 빨간 튜브 뚜껑이 구멍에서 14 mm 안(인접 구멍 간격 32 mm)이면 꽂힘으로 본다. 성공 = 놓았고 끝 장면에서 구멍에 있음(끝 장면은 구멍인데 손이 충분히 내려가지 않았으면 얹힘 의심으로 부분), 부분 = 쥐었지만 성공 못 함, 실패 = 못 쥠. 사람이 모든 시행을 확인하고 필요하면 고치며, 자동 판정과 사람 판정을 둘 다 기록해 일치율을 보고한다(`paper_stats.py` 6절). 결과는 체크포인트 × 실행 조건(E, 최대 시간)별로 따로 묶는다.
- **체크포인트 선택:** 7k / 14k / 20k를 **같은 배치 집합**(배치 사진으로 재현)에서 번갈아 시험해 성공률로 고른다(검증 손실이 아니라 실물 성공률로 선택).

### 6.5 기록

실행마다 `<세션>/_rollouts/rollout_날짜_시각/`에 `meta.json`(설정·자동 판정·단계·지표·시작/끝 장면·사람 확인), `data.npz`(10 Hz 관절·명령·손가락 폭·그리퍼 명령), `chunks.jsonl`(묶음마다 관측 상태, 받은 행동 16개, 실행한 칸, 지연), 3인칭·손목 사진이 남는다. 학습 데이터(`episode_*`)와 섞이지 않는다.

---

## 7. 초기 실행 관찰 (2026-11-02, 체크포인트 20k — 예비 결과, n = 2)

> [!IMPORTANT]
> 정식 평가 전 동작 확인이다. 논문에는 ‘예비 관찰’이나 논의 근거로만 쓰고 성공률로 쓰지 않는다.

| 시행 | 설정 | 경과 | 결과 |
|---|---|---|---|
| 11:05 | E = 8, 최대 40 s 【확인】 | 닫은 자리 y = 286 mm — 학습 데이터의 잡은 자리 범위(y ≤ 274 mm) 밖. 손가락이 뚜껑 옆에 걸려 폭 68.6 mm에서 멈춤 | 실패 (못 쥠) |
| 11:12 | E = 8, 최대 40 s | 닫은 자리가 장면 인식으로 계산한 잡을 자리와 수평 4.2 mm, 높이 학습 범위 안, 쥔 폭 32 mm → 빨간 칸에 꽂음. 40 s에 끝나 놓기 전 종료 | 부분 (꽂음, 시간 초과) |

- **학습 분포 가장자리에서의 실패:** 실패한 시행은 튜브가 학습 데이터에서 가장 왼쪽(+y)에 있던 위치보다 약 12 mm 더 바깥이었다. 성공한 시행은 가장 가까운 학습 시행에서 5 mm 떨어진 자리였다(30 mm 안에 6시행).
- **쥔 뒤 멈춤의 모방:** 닫기 명령 뒤 들기 시작까지 학습 데이터는 중간 6.1 s였다. 이 중 손가락 폭이 멈춘 뒤에도 약 4 s 정지가 있는데, 자동 시연이 그리퍼 응답을 기다리고 최소 저크로 천천히 출발했기 때문이다. 실행에서는 9.7 s였다: 그리퍼 응답 3.2 s 뒤 5개 묶음이 1~7 mm씩만 움직였다. 정책이 데이터의 정지를 그대로 배운 데다, 동기식 실행에서는 16칸 중 앞 8칸만 실행하고 다시 묻기 때문에 ‘천천히 출발’의 앞부분이 반복된 것으로 보인다.
- **처짐 보정의 학습:** 쥐는 순간 손끝 실제 − 명령 차이(y 약 −8 mm)가 가까운 학습 시행과 같았다. 정책이 컨트롤러 처짐까지 포함한 명령을 출력한다는 뜻이다(행동 = 명령으로 정의한 근거).
- **밝기:** 실행 때 밝기(3인칭 125, 손목 136)가 수집 기준(127, 139)과 같아 조명 차이는 원인이 아니었다(밝기 자동 맞춤 사용).

### 실행 영상 (2026-11-02 14:22)

<p align="center"><img src="media/rollout_demo_2x.gif" width="80%" alt="미세조정한 GR00T N1.7 정책으로 FR3가 빨간 튜브를 빨간 링 칸에 꽂는 모습 (2배속)"></p>

**영상 1.** 자동 시연 100시행으로 미세조정한 GR00T N1.7 정책이 폐루프로 FR3를 움직이는 모습(2배속). 홈에서 출발해 화면 오른쪽 랙의 빨간 튜브를 쥐고 들어 올린 뒤, 왼쪽 랙의 빨간 링 칸으로 옮겨 꽂고, 놓은 다음 위로 빠진다. 휴대폰으로 따로 찍은 영상이며 정책에 들어가는 카메라 영상은 아니다. 실제 속도 원본(38.7 s, 소리 제거): [MP4](media/rollout_demo.mp4)

위 표의 두 시행보다 나중에 찍은 시행이다 【확인: 체크포인트 · 한 번에 실행할 칸 E · 최대 시간 · 자동 판정 결과 — `_rollouts/` 기록과 맞춰 위 표에 추가】.

---

## 8. 논의와 한계

1. **스크립트 시연의 특성이 그대로 학습된다.** 자동 시연에는 실패 후 다시 시도하는 동작이 없고, 그리퍼 응답 대기 같은 수집 절차의 정지가 데이터에 남는다. 정지는 변환 단계에서 잘라낼 수 있다(칸 위 정지는 잘라냈고, 쥔 뒤 정지는 다음 버전 후보). 사람 원격조종 시연을 일부 섞으면 회복 동작을 보완할 수 있다.
2. **배치 분포가 성능의 경계.** 학습 분포 바깥(1~2 cm) 배치에서 정밀도가 크게 떨어졌다. 수집할 때 작업 영역 전체에 고르게 배치해야 하고, 평가 배치는 분포 안·가장자리를 나눠 보고하는 것이 공정하다.
3. **동기식 실행.** 추론 중 로봇이 정지하므로 묶음 사이에 멈춤이 생기고(왕복 약 0.19 s), 묶음 앞부분만 반복 실행되면 느린 출발 구간이 늘어난다. 한 번에 실행할 칸 E(8 또는 16)는 평가 전에 정해 고정한다.
4. **보정·인식에 대한 의존은 데이터 생성 단계뿐.** 화면–로봇 보정, 원근 보정, 색 기반 인식은 데이터 생성과 판정에만 쓰이고 정책에는 영상과 손끝 상태만 들어간다. 다만 생성된 시연의 정밀도(보정 오차 수 mm)가 학습 데이터 정밀도의 상한이 된다.
5. **놓기 방식 차이.** 수집 때는 옆 튜브가 있으면 12 mm만 벌렸다가 나중에 완전히 열었지만, 실행 때 놓기는 항상 최대 폭이다. 빨간 링 칸이 있는 랙의 다른 구멍을 비워 둔 이번 과제에서는 대부분 바로 다 열었다 【확인: open_full 시행 수】.
6. **고정 조건.** 홈 자세·3인칭 카메라 위치·조명이 수집 때와 같아야 한다. 노출은 수집 시행의 홈 밝기 중앙값을 목표로 자동으로 맞췄다. 이전 과제에서 3인칭 카메라를 조금 옮기는 것만으로 기준 모델이 재현되지 않은 경험이 있다.
7. **실시간성.** 일반 커널에서 libfranka 통신 제약 위반으로 컨트롤러가 정지하는 일이 있었다. 커널 선점·스레드 우선순위·C-state 조치로 줄였고, 정지된 시행은 자동 폐기되거나(수집) 실행 중단으로 기록된다(실행).

---

## 9. 논문에 쓰기 전에 확인할 것 (【확인】 모음)

1. 로봇 PC에서 `python3 paper_stats.py > stats.md` ([`scripts/paper_stats.py`](scripts/paper_stats.py), 다른 세션은 `--session ~/gello_data/<세션>`) → 수집 날짜·판정별 수·휴지통 수(저장 비율)·서로 다른 배치 수·시행 길이·속도 배율 분포·hole_learn 적용 수·원근 보정량·보정 점 수/모델/rms·자동 시연 정확도(쥐기·놓기 실제 − 계획)·그리퍼 힘·실행 기록 표.
2. 서버에서 체크포인트 설정 확인:

   ```bash
   cd ~/lerobot && python3 - <<'EOF'
   import json
   p = 'outputs/train/medbio_redslot_v1_groot17/checkpoints/020000/pretrained_model/'
   t = json.load(open(p + 'train_config.json')); c = t['policy']
   print({k: t.get(k) for k in ('steps', 'batch_size', 'seed', 'save_freq', 'num_workers')})
   print({k: c.get(k) for k in ('type', 'base_model_path', 'embodiment_tag', 'chunk_size', 'n_action_steps', 'use_relative_actions',
                                'relative_exclude_joints', 'tune_llm', 'tune_visual', 'tune_projector', 'tune_diffusion_model', 'tune_vlln',
                                'optimizer_lr', 'optimizer_weight_decay', 'warmup_ratio', 'use_bf16', 'max_steps', 'state_dropout_prob')})
   print('image_transforms:', t['dataset']['image_transforms'].get('enable'))
   EOF
   ls -l --time-style=+%F_%T outputs/train/medbio_redslot_v1_groot17/checkpoints/
   ```

3. 정식 평가: 체크포인트 3개 × 같은 배치 N개(분포 안 / 가장자리 구분), 최대 시간·E 고정, 성공률과 Wilson 95% 구간.

---

## 부록 A. 주요 파라미터

| 범주 | 파라미터 | 값 |
|---|---|---|
| 인식 | 다중 프레임 | 약 1.2 s 동안 최대 5장, 과반 채택, 위치 중간값 |
| 보정 | 모델 선택 | 아핀 / 호모그래피(6점 이상) 중 LOO 최대 오차가 작은 것 |
| 목표 | 손가락 충돌 모델 | 20 × 10 mm 사각형 2개, 중심 ±(벌린 폭/2 + 5 mm): 잡기 ±45 mm, 꽂기 ±26 mm · 뚜껑 반지름 16 mm + 여유 6 mm |
| 목표 | 원근 보정 | f_x = (W/2)/tan(43.5°), Δh = 세션 `cap_above` |
| 궤적 | 접근 a / 들기 l | 0.08 ± 0.02 m / 0.15 ± 0.02 m |
| 궤적 | 호 높이 h / 경유점 | U(0, 0.04) m / 수평 각 ±0.025 m |
| 궤적 | 꺾임 반지름 | 동작 ① 0.03 m, 동작 ② 0.04 / 0.06 / 0.04 m |
| 궤적 | 놓는 높이 / 빠지기 | z_g + 0.01 m / 칸 위 + 0.05 m |
| 궤적 | 평균 속도 | 수평 0.12, 수직 0.06 m/s × 속도 배율(기본 0.5) × (1 ± 0.15) |
| 궤적 | 칸 위 맞춤(선택) | 놓는 높이 + 0.06 m에서 1 s |
| 역기구학 | 감쇠 / 영공간 이득 | λ = 10⁻⁵ / 0.02 (홈 자세 쪽) |
| 역기구학 | 샘플 간격 / 관절 속도 상한 | 0.05 s / 0.8 rad/s (7번 0.15 rad/s) |
| 제어 | 명령 주기 / 기록 주기 | 100 Hz / 10 Hz |
| 제어 | 잡기 전 정착 | 0.5 s, 이득 2.0 s⁻¹, ≤ 4°/s, ≤ 6° |
| 그리퍼 | Grasp | 폭 0, ε ±0.08 m, 힘 70 N 【확인】, 0.1 m/s |
| 변환 | 칸 위 정지 제거 | 맞추는 자리 6 mm 안 + 6 mm/s 미만 |
| 학습 | 묶음 / 스텝 / 배치 / lr | 16 / 20,000 / 64 / 1×10⁻⁴ (코사인, 예열 5%) |
| 실행 | 한 번에 실행할 칸 E | 8 (설정 가능, 1–16) |
| 실행 | 그리퍼 문턱 | 열림→닫기 0.4 미만, 닫힘→열기 0.6 초과, 두 칸 연속 |
| 실행 | 안전 | 작업 범위, 기울기 25°, 0.1 s당 5 cm, IK 2 mm / 0.02 rad, 관절 한계 0.02 rad, 바닥 0.09 m |
| 실행 | 종료 | 놓은 뒤 2.5 s, 최대 시간(권장 120 s), Space |

## 부록 B. 그림 설명 (영문, 논문용)

- **Fig. 1.** Overview. ① Scripted demonstrations are generated on the real robot (the operator only re-arranges objects); ② episodes are converted to a LeRobot dataset with 10-D state and action; ③ GR00T N1.7 is partially fine-tuned (frozen VLM backbone; trained VL layers, state encoder and DiT flow-matching action head) with relative actions; ④ the policy runs in a synchronous closed loop between the robot PC and a remote GPU server.
- **Fig. 2.** Principle of the scripted demonstrator. (a) Colour-based perception in the wrist-camera image at the home pose and target computation (finger footprints are used to check collisions with neighbouring caps). (b) Parallax correction for the hole, which lies Δh below the calibration plane (cap tops). (c) Side view of a planned trajectory computed with the planner (quadratic-Bézier corner blending, one minimum-jerk profile per motion over arc length). (d) Minimum-jerk timing s(τ) and its velocity.
- **Fig. 3.** Closed-loop execution. (a) One cycle: observe while the robot holds, infer on the server, process the chunk, and execute the first 8 steps (0.8 s). (b) A cycle with a gripper event: execution stops at the first step k where the gripper value crosses the hysteresis threshold on two consecutive steps; the gripper then acts and the robot re-observes. (c) Safety checks and termination.

## 부록 C. 참고문헌 (서지 형식은 투고처에 맞게)

- T. Flash, N. Hogan. The coordination of arm movements: an experimentally confirmed mathematical model. *J. Neuroscience*, 1985. (최소 저크)
- N. Hogan. Impedance control: an approach to manipulation. *ASME J. Dynamic Systems, Measurement, and Control*, 1985.
- C. W. Wampler. Manipulator inverse kinematic solutions based on vector formulations and damped least-squares methods. *IEEE Trans. SMC*, 1986.
- Y. Nakamura, H. Hanafusa. Inverse kinematic solutions with singularity robustness for robot manipulator control. *ASME J. Dynamic Systems, Measurement, and Control*, 1986.
- A. Liégeois. Automatic supervisory control of the configuration and behavior of multibody mechanisms. *IEEE Trans. SMC*, 1977. (영공간)
- R. Hartley, A. Zisserman. *Multiple View Geometry in Computer Vision*. Cambridge Univ. Press. (호모그래피)
- Y. Zhou, C. Barnes, J. Lu, J. Yang, H. Li. On the continuity of rotation representations in neural networks. *CVPR*, 2019. (rot6d)
- Y. Lipman, R. T. Q. Chen, H. Ben-Hamu, M. Nickel, M. Le. Flow matching for generative modeling. *ICLR*, 2023.
- T. Z. Zhao, V. Kumar, S. Levine, C. Finn. Learning fine-grained bimanual manipulation with low-cost hardware. *RSS*, 2023. (행동 묶음, ACT)
- NVIDIA. GR00T N1: An open foundation model for generalist humanoid robots. arXiv:2503.14734, 2025. (N1.7은 모델 카드 인용)
- P. Wu, Y. Shentu, Z. Yi, X. Lin, P. Abbeel. GELLO: A general, low-cost, and intuitive teleoperation framework for robot manipulators. *IROS*, 2024.
- R. Cadene et al. LeRobot: state-of-the-art machine learning for real-world robotics in PyTorch. GitHub, 2024.
- A. Mandlekar et al. MimicGen: a data generation system for scalable robot learning using human demonstrations. *CoRL*, 2023.
- H. Walke et al. BridgeData V2: a dataset for robot learning at scale. *CoRL*, 2023. (스크립트 수집 + 원격조종 혼합 사례)
- S. Haddadin et al. The Franka Emika robot: a reference platform for robotics research and education. *IEEE Robotics & Automation Magazine*, 2022.
- E. B. Wilson. Probable inference, the law of succession, and statistical inference. *JASA*, 1927. (성공률 신뢰구간)
