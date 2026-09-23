# 차량 경로 추종 제어 테스트베드

[English](README.md) | **한국어**

바로 실행해 볼 수 있는 MATLAB/Simulink 횡방향 경로 추종 제어 환경입니다.
차량이 20 m/s로 S자 차선변경 경로를 따라 달리고, 조향 제어기가 차량을
경로 위에 유지합니다.

- **플랜트**: 비선형 single-track 차량 모델(*Vehicle Body 3DOF Single Track* 블록)
- **경로와 오차**: S자 차선변경 경로. 횡방향 오차 `e_y`와 헤딩 오차 `e_psi`를 실시간으로 계산
- **레퍼런스 제어기**: full-state feedback(pole placement, LQR). baseline 풀이 전체 포함
- **도구**: 빠른 MATLAB 버전 시뮬레이션, 표준 지표와 플롯, 실시간 X-Y 모니터, 자체 테스트

직접 제안하는 프로젝트의 출발점으로 쓰면 됩니다.

![Baseline 결과](docs/img/baseline_place_summary.png)

## 필요 환경

| 제품 | 용도 |
|---|---|
| MATLAB, Simulink | 모델, 시뮬레이션 |
| Control System Toolbox | `place`, `lqr`, `ss` |
| Vehicle Dynamics Blockset | 차량 플랜트 블록 |

MATLAB R2026a에서 검증했습니다. 더 낮은 버전이라면
`path_following_baseline.slx`를 지우고 `build_simulink_model`을 실행해
모델을 다시 만드세요.

## 빠른 시작

프로젝트 폴더에서 실행합니다.

```matlab
run_baseline                             % 레퍼런스 설계 + Simulink 실행, 그림 -> results/
open_system('path_following_baseline')   % Simulink 모델을 열고 Run
my_design_template                       % 내 설계용 템플릿 -> results/my_design/
cd tests; run_tests                      % 환경 자체 점검(약 2분)
```

모델을 직접 실행하면 차량 위치를 보여주는 실시간 X-Y 그림이 뜹니다.
끄려면 workspace에서 `xy_monitor_on = 0`으로 설정하세요.

## 사용법

**신호 규약.** 오차 상태 `e = [e_y; e_y_dot; e_psi; e_psi_dot]`
(`e_y > 0`: 차량이 경로 왼쪽, 각도 단위는 rad). 제어 입력은 앞바퀴 조향각
`delta_f` [rad]이고 ±30°에서 포화됩니다.

### 1. 내 설계로 돌려보기

`my_design_template.m`을 복사해서(예: `my_design.m`) `EDIT` 표시가 있는
부분만 고치면 됩니다. 고칠 곳은 scenario와 제어기입니다. 스크립트는 빠른
MATLAB 시뮬레이션을 돌려 레퍼런스 설계와 비교하고, Simulink에서 확인한
뒤, 지표와 그림을 `results/<tag>/`에 저장합니다.

```matlab
ctrl = design_state_feedback(A, B, 'place', struct('poles', p));    % 내가 고른 극점
ctrl = design_state_feedback(A, B, 'lqr', struct('Q', Q, 'R', R));  % 또는 LQR
```

### 2. Scenario 바꾸기

| `scenario` | 경로 | 사용처 |
|---|---|---|
| `'baseline'` | `Xoff = 80 m`, `eta = 40 m` | baseline 프로젝트 |
| `'sharp_lane_change'` | `Xoff = 50 m`, `eta = 20 m` | 곡률이 큰 경로 (feedforward 등) |

```matlab
[veh, pth, sim_opt] = load_params('sharp_lane_change');   % MATLAB
init_model_workspace([], [], 'sharp_lane_change');        % Simulink (sim 전에 실행)
```

차량 파라미터, 속도, 시뮬레이션 시간은 `src/load_params.m`에 있습니다.

### 3. 내 제어기 연결하기

- **MATLAB 시뮬레이션**: `delta = law(e, aux)` 형태의 정적 제어 법칙이면 무엇이든 됩니다.

  ```matlab
  ctrl.law = @(e, aux) ... ;     % aux = [Xp, Yp, psi_p, kappa, psi_dot_des]
  log = simulate_closed_loop(veh, pth, ctrl, sim_opt);
  ```

- **Simulink**: *State Feedback Controller* 서브시스템의 내용을 바꾸되
  포트는 그대로 둡니다. 입력은 `y_e, y_e_dot, psi_e, psi_e_dot`, 출력은
  `WhlAngF` [rad]입니다. 내부 상태가 있는 블록(observer, 식별기, 필터)은
  *Path Tracking Errors*와 제어기 사이에 넣어도 됩니다. 경로 곡률은
  *Path Tracking Errors*의 `path_info` 출력 4번째 원소입니다.
  `build_simulink_model`은 `path_following_baseline.slx`를 덮어쓰므로,
  수정한 모델은 다른 이름으로 저장하세요.

### 4. 평가와 비교

```matlab
log = simulink_log_to_struct(out);          % Simulink 결과 -> MATLAB 시뮬레이션과 같은 구조체
M   = performance_metrics(log);             % e_y, e_psi, delta의 max / rms
plot_results(log, pth, ctrl, 'results/mine', 'mine');                  % e_y, e_psi, X-Y, 조향 그림
plot_comparison({log_a, log_b}, {'A', 'B'}, 'results/mine', 'a_vs_b', 'A vs B');
```

### 주요 파일

| 파일 | 내용 |
|---|---|
| `src/load_params.m` | 모든 파라미터와 scenario |
| `src/error_model_linear.m` | 선형 오차 모델 `A, B, Bd, C` |
| `src/design_state_feedback.m` | pole placement / LQR |
| `src/path_tracking_errors.m` | `e_y`, `e_psi`와 그 변화율 |
| `src/simulate_closed_loop.m` | Simulink 루프와 같은 동작을 하는 빠른 MATLAB 시뮬레이션 |
| `path_following_baseline.slx` | Simulink 모델(`build_simulink_model.m`으로 생성) |

## Example proposals

이 테스트베드에서 해 볼 수 있는 주제의 예시입니다. 여기에 없는 주제나
개인 과제도 괜찮습니다.

- **Observer 기반 feedback**: 차선 센서는 `e_y`와 `e_psi`만 측정합니다. 나머지 상태를 추정합니다.
- **실시간 식별**: 조향각 `delta_f`에서 yaw rate `r`까지의 yaw 동특성을 실시간으로 식별합니다.
- **Feedforward**: 경로 곡률을 이용해 `'sharp_lane_change'` scenario에서 추종 성능을 높입니다.
- **MPC**: 선형 오차 모델 기반 예측 제어. 조향각과 조향 속도 제약 포함.
- **강화학습(RL)**: 빠른 MATLAB 시뮬레이션에서 조향 정책을 학습하거나, RL이나 Bayesian optimization으로 `K`를 튜닝한 뒤 Simulink에서 검증.
- **기하학적 제어기**: Pure Pursuit, Stanley와 state feedback 비교.
- **속도 변화**: `A`, `B`가 `V_x`에 따라 달라지므로 속도 구간에 대한 gain scheduling.
- **강건성**: 모델 불일치(`'raw'` block mode가 바로 쓸 수 있는 예), H∞ 설계.
- **센서와 액추에이터**: 센서 노이즈와 Kalman filter, 조향 지연과 속도 제한.
- **다른 경로**: double lane change, 원형, waypoint 경로(`path_reference.m`, `path_closest_point.m` 수정).

## 더 보기

- [`docs/baseline_reference.md`](docs/baseline_reference.md): baseline 풀이의 모델, 제어기 설계, 결과, 참고 사항(영어)
- [`docs/plant_model.html`](docs/plant_model.html): plant 모델 유도 과정과 `A`, `B`, `Bd`, `C`, `D`의 의미(브라우저로 열기)

## 라이선스

[MIT](LICENSE)
