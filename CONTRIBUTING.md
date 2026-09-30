# Contributing Guide

argus-edge-vision 프로젝트의 브랜치, 커밋, PR 규칙입니다. 작업 전에 한 번 읽어주세요.

## 0. 처음 한 번만 — 로컬 커밋 검사 켜기

커밋 메시지는 [commitlint](https://commitlint.js.org) 로 검사하며, [husky](https://typicode.github.io/husky/) 가 git hook 을 설치합니다.

**준비물: Node.js 22.12 이상** (`node --version` 으로 확인)

| OS | 설치 |
|---|---|
| macOS | `brew install node@22` |
| Windows | `winget install OpenJS.NodeJS.LTS` |
| Ubuntu | [NodeSource](https://github.com/nodesource/distributions) 또는 `nvm install 22` |

clone 한 뒤 저장소 루트에서 한 번만 실행하세요.

```bash
npm install
```

- `commitlint`, `husky` 가 `node_modules/` 에 설치되고, `commit-msg` hook 이 자동으로 연결됩니다.
- 이후 규칙에 맞지 않는 커밋 메시지는 `git commit` 시점에 거부됩니다. (터미널, VS Code, CLion 등 어떤 도구로 커밋해도 동일)
- `package.json` 은 **커밋 메시지 검사 도구 전용**입니다. 프로젝트 코드(C/C++, Python)와는 무관합니다.
- `git commit --no-verify` 로 검사를 건너뛰지 마세요. 건너뛰어도 PR 에서 CI 에 다시 걸립니다.

PR 에서는 GitHub Actions(`commitlint`)가 `package-lock.json` 에 고정된 **같은 버전**의 commitlint 로 한 번 더 검사하며, 통과해야 머지할 수 있습니다.

---

## 1. 브랜치 전략 (Git Flow)

| 브랜치 | 용도 | 분기 원본 | 머지 대상 |
|---|---|---|---|
| `main` | 배포(릴리즈) 버전 | — | — |
| `develop` | 다음 릴리즈 통합 | `main` | — |
| `feature/<이슈번호>-<설명>` | 기능 개발 | `develop` | `develop` |
| `release/v<X.Y.Z>` | 릴리즈 준비 (버그 수정, 버전 표기만) | `develop` | `main`, `develop` |
| `hotfix/v<X.Y.Z>` | 배포 버전 긴급 수정 | `main` | `main`, `develop` |

브랜치 이름 예시

```
feature/12-onboard-frame-capture
feature/27-gcs-telemetry-view
release/v0.2.0
hotfix/v0.2.1
```

- 브랜치 이름은 **소문자 + 하이픈(-)** 만 사용합니다.
- `main`, `develop` 에는 직접 push 할 수 없습니다. 반드시 PR로 머지합니다.

---

## 2. 커밋 메시지 (Conventional Commits)

### 형식

```
<type>(<scope>): <subject>

<body>        ← 선택 사항: 무엇을, 왜 바꿨는지

<footer>      ← 선택 사항: 이슈 연결, BREAKING CHANGE
```

### type

| type | 언제 |
|---|---|
| `feat` | 새 기능 |
| `fix` | 버그 수정 |
| `docs` | 문서만 변경 |
| `style` | 포맷, 세미콜론 등 동작 변화 없는 코드 스타일 |
| `refactor` | 기능 변화 없는 구조 개선 |
| `perf` | 성능 개선 |
| `test` | 테스트 추가/수정 |
| `build` | 빌드 시스템, 툴체인 (CMake, Makefile 등) |
| `ci` | GitHub Actions 등 CI 설정 |
| `chore` | 그 외 변경사항 (.gitignore, 폴더 구조 등) |
| `revert` | 이전 커밋 되돌리기 |

### scope (생략 가능)

| scope | 대상 |
|---|---|
| `onboard` | `onboard/` — 기체 탑재 SW |
| `gcs` | `ground_control_station/` — 지상 통제 SW |
| `models` | `models/` — 학습/추론 모델 |
| `datasets` | `datasets/` — 데이터셋, 전처리 |
| `docs` | `docs/` |
| `ci` | `.github/` 워크플로 |
| `deps` | 의존성 버전 변경 |

여러 영역에 걸치거나 루트 설정 파일만 바꿨다면 scope 를 생략합니다.
새 scope 가 필요하면 `commitlint.config.mjs` 와 이 문서를 **함께** 수정하는 PR 을 올려주세요.

### subject 규칙

- 첫 줄(header)은 **72자 이내**
- `type` 과 `scope` 는 영문, **subject 는 한글**로 작성합니다.
- subject 는 **`~ 추가`, `~ 수정`, `~ 삭제` 처럼 명사형**으로 끝냅니다. (~~`추가했습니다`~~, ~~`추가함`~~)
- 끝에 마침표(`.`)를 붙이지 않습니다.
- 본문(body)은 한 줄 **100자 이내**를 권장합니다. (넘으면 경고)

### 예시

```
feat(onboard): 카메라 프레임 캡처 태스크 추가
fix(gcs): 텔레메트리 패킷이 잘릴 때 크래시 발생 문제 수정
docs: Jetson 빌드 방법 갱신
chore: 초기 프로젝트 디렉토리 구조 추가
refactor(models): 전처리 로직을 별도 모듈로 분리
feat(onboard)!: 텔레메트리 패킷 형식을 v2로 변경
```

본문과 footer 가 있는 예시

```
fix(onboard): 타임아웃 시 mutex 미해제로 태스크가 멈추는 문제 수정

타임아웃 경로에서 mutex 를 해제하지 않아 다음 주기에
detection task 가 영구 대기하는 문제 수정.

Closes #34
```

### 호환되지 않는 변경 (Breaking Change)

type 뒤에 `!` 를 붙이고 footer 에 설명을 남깁니다.

```
feat(onboard)!: 텔레메트리 패킷 형식을 v2로 변경

BREAKING CHANGE: GCS 는 v2 파서로 업데이트해야 수신 가능
```

---

## 3. Pull Request

### 머지 방식

| PR 방향 | 머지 방식 |
|---|---|
| `feature/*` → `develop` | **Squash and merge** |
| `release/*` → `main` / `develop` | **Create a merge commit** |
| `hotfix/*` → `main` / `develop` | **Create a merge commit** |

> Squash merge 는 **PR 제목이 그대로 커밋 메시지**가 됩니다.
> 그래서 PR 제목도 커밋 메시지와 같은 형식이어야 하며, CI 가 검사합니다.

### 리뷰 규칙

| 대상 브랜치 | 필요 승인 수 |
|---|---|
| `develop` | 3명 |
| `main` | 5명 |

- 승인 후 새 커밋을 push 하면 승인이 초기화되어 다시 리뷰를 받아야 합니다.
- 리뷰 코멘트(conversation)가 모두 resolve 되어야 머지할 수 있습니다.
- PR 은 가능한 작게 — 하나의 PR 에는 하나의 목적만 담습니다.
- PR 본문에 관련 이슈를 연결합니다. (`Closes #12`)

### PR 올리기 전 체크리스트

- [ ] 최신 `develop` 을 반영했다 (`git pull --rebase origin develop`)
- [ ] 빌드/테스트가 로컬에서 통과한다
- [ ] PR 제목이 Conventional Commits 형식이다
- [ ] 불필요한 파일(빌드 산출물, 데이터셋 원본, 모델 가중치 등)이 포함되지 않았다

---

## 4. 예시

```bash
# 1) 기능 브랜치 생성
git switch develop
git pull origin develop
git switch -c feature/12-onboard-frame-capture

# 2) 작업 & 커밋
git add onboard/
git commit -m "feat(onboard): 카메라 프레임 캡처 태스크 추가"

# 3) push 후 GitHub 에서 develop 대상으로 PR 생성
git push -u origin feature/12-onboard-frame-capture
```

머지된 브랜치는 GitHub 에서 자동 삭제됩니다. 로컬에서도 정리해주세요.

```bash
git switch develop
git pull origin develop
git branch -d feature/12-onboard-frame-capture
```
