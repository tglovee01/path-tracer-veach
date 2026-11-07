# 🚀 AGENTS.md: Veach MIS 기반 Path Tracer 개발 지침서

본 문서는 Eric Veach의 Multiple Importance Sampling (MIS) 기법을 반영한 C++ Path Tracer 개발을 위한 단계별 기획 지침 및 검증 기준(Pass Criteria)을 정의합니다. 
AI 에이전트 및 개발자는 반드시 이전 단계의 **Pass Criteria**를 충족한 후 다음 단계로 진행해야 합니다.

---

## 🎯 1. 핵심 개발 목표 (Core Objectives)
- **이론적 기반**: Eric Veach (1995/1997) "Multiple Importance Sampling" 적용
- **핵심 알고리즘**: 
  - 광원 샘플링(Explicit Light Sampling / Next Event Estimation)과 BSDF 샘플링의 결합
  - Balance Heuristic 및 Power Heuristic ($\beta=2$) 가중치 함수 구현
- **설계 제약**: Modern C++ (C++17/20), 가독성 중심 아키텍처, 단계별 독립 테스트 코드 작성

---

## 🧪 2. 단계별 개발 및 통과 조건 (Phase & Pass Criteria)

### Phase 1: 기본 수학 및 프레임워크 구축 (Foundation & Geometry)
- **주요 작업**:
  - `Vector3`, `Ray`, `HitRecord` 기본 클래스 구현
  - 난수 생성기 (`pcg32` 또는 `std::mt19937_64` 기반 0~1 균일 난수) 세팅
  - 람베르시안(Lambertian Diffuse) 및 거울(Specular Reflection) BSDF 기본 구조체 정의
- **Pass Criteria**:
  - [ ] Vector3 내적, 외적, 정규화(Normalize) 단위 테스트 통과
  - [ ] Ray-Sphere 및 Ray-Triangle 교차 판정 정밀도 테스트 통과

### Phase 2: 단일 샘플링 경로 추적기 (Single-Technique Path Tracer)
- **주요 작업**:
  - 기본 Recursive Path Tracing 알고리즘 구현
  - BSDF 기반 샘플링만으로 렌더링하는 파이프라인 구성
  - 면광원(Area Light Source) 구체화 및 광원 교차 시 Radiance 반환 처리
- **Pass Criteria**:
  - [ ] 면광원과 기본 상자가 포함된 Cornell Box 씬 생성
  - [ ] BSDF 샘플링만으로 노이즈가 포함된 렌더링 이미지 파일(.ppm 또는 .png) 출력 성공

### Phase 3: Direct Light Sampling (Next Event Estimation, NEE)
- **주요 작업**:
  - 광원 표면에서의 직접 샘플링(Light Sampling) 로직 구현
  - Shadow Ray 발사를 통한 차폐(Occlusion) 여부 검사
  - BSDF 샘플링과 Light Sampling 각각으로 렌더링하는 기능 분리
- **Pass Criteria**:
  - [ ] BSDF 샘플링 시 매끄러운 큰 광원에서 발생하는 노이즈 확인
  - [ ] Light Sampling 시 점광원/작은 광원에서 노이즈가 획기적으로 줄어드는 현상 확인 및 결과 비교 완료

### Phase 4: Veach Multiple Importance Sampling (MIS) 통합
- **주요 작업**:
  - **Balance Heuristic** 가중치 함수 구현:
    $$w_i(x) = \frac{n_i f_i(x)}{\sum_j n_j f_j(x)}$$
  - **Power Heuristic** ($\beta=2$) 가중치 함수 구현:
    $$w_i(x) = \frac{(n_i f_i(x))^2}{\sum_j (n_j f_j(x))^2}$$
  - Light Sampling $p_\text{light}(\omega_i)$와 BSDF Sampling $p_\text{bsdf}(\omega_i)$ 확률 밀도 함수(PDF) 상호 계산 구현
  - 두 기법의 Radiance 합산 파이프라인 통합
- **Pass Criteria**:
  - [ ] **Veach의 대표 테스트 씬(Veaches MIS Test Scene)** 구상: 매끄러운 재질(Roughness 변화) 바닥판 + 서로 다른 크기/광도의 면광원 배치
  - [ ] 광원 샘플링 단독, BSDF 샘플링 단독 대비 MIS 결합 렌더링의 분산(Variance/Noise) 극적 감소 확인

### Phase 5: 성능 최적화 및 결과 검증 (Optimization & Verification)
- **주요 작업**:
  - OpenMP 또는 C++ `std::thread` 기반 멀티스레딩 타일 렌더링 적용
  - 러시아 룰렛(Russian Roulette) 기반 Path Termination 적용하여 레이 깊이 최적화
- **Pass Criteria**:
  - [ ] 멀티스레딩 적용 후 스레드 개수에 비례하는 렌더링 속도 향상 확인 (Race Condition 없음)
  - [ ] 동일 씬 기준 MIS 적용 결과와 렌더링 표준 가이드 간 에너지가 보존(Energy Conservation)되는지 검증

---

## ⛔ 3. 절대 준수 규칙 (Don'ts for AI)
1. **역슬래시 사용 금지**: 마크다운 서식(`-`, `*`, `#`) 작성 시 기호 앞에 역슬래시(`\`)를 붙이지 않는다.
2. **단계를 건너뛰지 말 것**: Phase 1의 테스트가 통과하기 전에 Phase 4의 MIS 수식을 코딩하지 않는다.
3. **가중치 수식 오류 주의**: MIS 가중치 계산 시 $p_\text{light}$와 $p_\text{bsdf}$의 단위(Solid Angle vs Area) 변환이 올바르게 되었는지 항상 확인한다.