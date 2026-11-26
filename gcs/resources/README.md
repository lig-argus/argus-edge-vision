# 통제기 리소스

통제기 실행 파일에 넣는 파일. 실행 파일(`app`)이 Qt 리소스로 등록한다.

| 파일 | 내용 | 출처와 허가 |
|---|---|---|
| `fonts/Roboto-*.ttf` | 영문 글꼴 Roboto 3.016 (Regular, Medium, Bold) | googlefonts/roboto-classic 릴리스 v3.016 의 `android/static`. SIL Open Font License 1.1 (`fonts/Roboto-OFL.txt`) |
| `fonts/Pretendard-*.otf` | 한글 글꼴 Pretendard (Regular, Medium, Bold) | orioncactus/pretendard 의 `packages/pretendard/dist/public/static`. SIL Open Font License 1.1 (`fonts/Pretendard-OFL.txt`) |
| `icons/my-location.svg`, `location-searching.svg`, `add.svg`, `remove.svg` | 지도 버튼 아이콘 (현재 위치 따라가기, 따라가기 해제, 확대, 축소) | Astro UXDS (RocketCommunicationsInc/astro 의 `packages/web-components/src/icons`). 배포본 그대로 사용. 저작권 문구를 함께 배포하는 조건 (`icons/Astro-LICENSE.txt`) |
| `icons/home-h.svg` | 지도의 이륙 지점 표시 [H] | 이 저장소에서 작성 |
| `lig_white.svg`, `lig_white.png` | LIG 워드마크 (흰색). 상단 막대, 시작 화면, 실행 아이콘 | LIG CI. CI 규정상 로고의 비율과 색을 바꾸지 않는다 |
| `app_icon.ico`, `app_icon.rc` | Windows 실행 파일 아이콘 | `brand::paintAppIcon` 으로 생성 |
| `app_icon.icns` | macOS 실행 파일 아이콘 | `brand::paintAppIcon` 으로 생성 |
| `px4/indoor.params`, `px4/outdoor.params` | FC 비행 환경 파라미터 묶음 (실내, 실외) | PX4 v1.16 파라미터 파일 형식. 각 파일 머리 주석 참조 |

- 글꼴: 화면 글자는 Roboto(영문, 숫자)와 Pretendard(한글)를 쓴다. 실행 파일에 넣어 운용 PC 에 글꼴을 설치하지 않아도 된다. Pretendard 는 글자 상자 비율이 Roboto 와 비슷해 두 글꼴을 섞어도 세로 가운데가 맞는다
- 아이콘: Astro 아이콘은 검은색이라 통제기가 불러올 때 버튼 글자색으로 칠한다
