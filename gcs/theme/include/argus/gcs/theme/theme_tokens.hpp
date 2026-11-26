// 통제기 화면의 색과 크기 값 (디자인 토큰). 값은 이 파일 한 곳에만 둔다.
// QML 화면은 quick 모듈의 연결 객체로, 직접 그리는 코드는 이 상수로 읽는다.
// 이 파일은 값만 둔다 (색을 섞는 blend 와 반투명 색을 만드는 withAlpha 외에는 코드를 넣지 않는다).
//
// 색 원칙
// - 바탕, 패널, 버튼은 LIG CI 의 두 색과 그 명암: 주색 Innovative Blue #002F6D (PANTONE 294 C),
//   보조색 Futuristic Gray #D0D3D4 (PANTONE 427 C)
// - 상태는 Astro UXDS 상태 체계(Status System, 어두운 테마 값). 색과 모양을 함께 바꿔 색약도
// 구분한다.
//   상태 색을 상태가 아닌 곳(장식, 태그)에 쓰지 않고, 빨강은 즉시 조치가 필요한 상태에만 쓴다
// - 면의 밝기 순서는 Astro 어두운 테마 토큰의 관계를 주색 명암에 옮긴다: 바탕이 가장 어둡고, 제목
//   띠와 입력칸, 팝업 같은 어두운 면이 그다음, 패널 본문이 가장 밝다. 글자는 Astro 값(흰 글자, 보조
//   글자 #D4D8DD)을 쓴다
// - 표적 표시는 MIL-STD-2525 를 따른다 (Astro 에 피아 개념이 없다)
#ifndef ARGUS_GCS_THEME_THEME_TOKENS_HPP_
#define ARGUS_GCS_THEME_THEME_TOKENS_HPP_

#include <QColor>
#include <cstdint>

namespace argus::gcs::theme {

// ---------------------------------------------------------------- LIG CI
constexpr QRgb kCiBlue = 0x002f6d;  // 주색 Innovative Blue: 상단 막대, 버튼, 선택 행

// 보조색 Futuristic Gray: 칸 경계선, 선택 표시, 흐린 강조 글자, 실행 아이콘의 체계 이름
constexpr QRgb kCiGray = 0xd0d3d4;
constexpr QRgb kCiFuturisticGray = kCiGray;

constexpr QRgb kCiBlueLight = 0x6f9be0;  // 주색의 밝은 명암 (기울인 막대, 재생 구간, 명령 표식)

// CI 그래픽 모티프 그러데이션의 두 끝 (CI 매뉴얼에 화면 값이 없어 그림에서 잰 값)
constexpr QRgb kCiMotifLight = 0x2d3d6f;
constexpr QRgb kCiMotifDark = 0x1a2643;

// ---------------------------------------------------------------- 바탕과 면 (주색의 어두운 명암)
// 패널 본문이 바탕보다 2.65 배 밝다 (Astro surface-default 와 base-default 의 관계)
constexpr QRgb kBgBase = 0x04142f;     // 화면 바탕, 영상 칸, 탭
constexpr QRgb kBgVideo = 0x010a1c;    // 영상 그림 바탕
constexpr QRgb kPanelBody = 0x0a2450;  // 패널 본문, 표, 기록

// 어두운 면: 패널 제목 띠, 표 머리, 입력칸, 선택칸, 팝업, 알림. 본문의 0.74 배 밝기 단색이다
// (Astro surface-header). 입력칸을 화면 바탕색으로 하면 검은 상자로 떠 보여 같은 어두운 면으로
// 모은다
constexpr QRgb kPanelHeader = 0x081e44;
constexpr QRgb kSurfaceRaised = kPanelHeader;        // 메뉴, 팝업, 연결 표시, 꺼진 버튼
constexpr QRgb kSurfaceBadge = kPanelHeader;         // 배지, 이륙 상자
constexpr QRgb kSurfaceInput = kPanelHeader;         // 입력칸, 선택칸
constexpr QRgb kSurfaceNotification = kPanelHeader;  // 알림 띠, 알림 상자
constexpr QRgb kClockBg = kPanelHeader;              // 시계: 상단 막대보다 어두운 상자
constexpr QRgb kSliderThumbBg = kPanelHeader;        // 슬라이더 손잡이

// 밝은 면: 본문보다 한 단계 밝다
constexpr QRgb kSurfaceTile = 0x123566;    // 숫자 칸, 가상 조종기 키캡
constexpr QRgb kTooltipBg = kSurfaceTile;  // 도움말 상자

// ---------------------------------------------------------------- 선
constexpr QRgb kBorderDefault = 0x2b5597;    // 패널, 버튼, 입력 테두리
constexpr QRgb kBorderSubtle = 0x10305f;     // 꺼진 버튼 테두리, 표 행 구분선, 탭 사이
constexpr QRgb kDivider = 0x1b3f7a;          // 상태 띠 칸 구분, 진행 막대 테두리, 빈 신호 막대
constexpr QRgb kLeader = 0x1f3f73;           // 상태 목록의 이름과 값 사이 점선
constexpr QRgb kContainerBorder = 0x000000;  // 패널 사이 1 px
constexpr QRgb kColumnDivider = 0x2ee6f6;    // 세 칸 경계 (형광 청록)
constexpr QRgb kControlBorder = 0x8a939c;    // 가상 조종기 패드, 꺼진 스위치 트랙
constexpr QRgb kStickGuide = 0x3a5580;       // 가상 조종기 안내선

// ---------------------------------------------------------------- 글자
constexpr QRgb kTextPrimary = 0xffffff;
constexpr QRgb kTextMuted = 0xd4d8dd;    // 보조 글자: 이름, 단위, 시각
constexpr QRgb kTextLabel = kTextMuted;  // 상태 띠 칸 이름
constexpr QRgb kTextSoft = kTextMuted;   // 가상 조종기 안내 글, 꺼진 스위치 손잡이
constexpr QRgb kTextReplay = 0x8be6f5;   // 녹화 재생 표시
constexpr QRgb kTextInverse = 0x080c11;  // 밝은 바탕 위 글자

// ---------------------------------------------------------------- 꺼진 조작부
// 꺼진 조작부는 투명도 40 % 로 흐리게 한다 (Astro opacity-disabled). 조작부가 놓이는 패널 본문 위에
// 40 % 로 겹친 색을 미리 계산해 둔다
constexpr std::uint32_t kDisabledPercent = 40;
constexpr std::uint32_t kPercentFull = 100;

// fg 를 bg 위에 percent % 로 겹친 색
constexpr QRgb blend(QRgb fg, QRgb bg, std::uint32_t percent) {
    const auto channel = [percent](QRgb a, QRgb b, QRgb shift) {
        const QRgb x = (a >> shift) & 0xffU;
        const QRgb y = (b >> shift) & 0xffU;
        return (((x * percent) + (y * (kPercentFull - percent)) + (kPercentFull / 2U)) /
                kPercentFull)
               << shift;
    };
    return channel(fg, bg, 16U) | channel(fg, bg, 8U) | channel(fg, bg, 0U);
}

constexpr QRgb kTextDisabled = blend(kTextPrimary, kPanelBody, kDisabledPercent);
constexpr QRgb kDisabledBg = blend(kCiBlue, kPanelBody, kDisabledPercent);  // 꺼진 주 버튼 바탕
constexpr QRgb kDisabledBorder = blend(kBorderDefault, kPanelBody, kDisabledPercent);
constexpr QRgb kDisabledInputBg = blend(kSurfaceInput, kPanelBody, kDisabledPercent);

// ---------------------------------------------------------------- 버튼 상태 면
constexpr QRgb kButtonHover = 0x0d3f85;    // 마우스를 올렸을 때, 지금 모드, 녹화 중 바탕
constexpr QRgb kButtonPressed = 0x021f4a;  // 누르는 중
constexpr QRgb kFlashBg = 0x0d3060;        // 이륙 고도를 바꾼 직후 칸 바탕

// ---------------------------------------------------------------- 상태 색 (Astro, 어두운 테마)
constexpr QRgb kStatusNormal = 0x56f000;    // 정상: 원
constexpr QRgb kStatusStandby = 0x2dccff;   // 대기: 고리
constexpr QRgb kStatusCaution = 0xfce83a;   // 주의: 사각형
constexpr QRgb kStatusSerious = 0xffb302;   // 심각: 마름모
constexpr QRgb kStatusCritical = 0xff3838;  // 위급: 아래 삼각형
constexpr QRgb kStatusOff = 0xa4abb6;       // 꺼짐: 작은 원

// 상태 배지 바탕 (상태 색의 어두운 명암). 대기는 입력칸 바탕과 같다
constexpr QRgb kPillOkBg = 0x062a2a;
constexpr QRgb kPillWarnBg = 0x2e1f10;
constexpr QRgb kPillBadBg = 0x2e1016;

constexpr QRgb kInteractive = 0x4dacff;       // 조작 가능 요소 테두리 (기본 알림 띠)
constexpr QRgb kInteractiveHover = 0x92cbff;  // 조작 가능 요소에 마우스를 올렸을 때
constexpr QRgb kInteractiveMuted = 0x2b659b;  // 라디오 단추 테두리

// 키보드 초점 테두리: 흰색. 파랑은 주색과 같아 강조로 보이지 않고, 흰색은 색에 뜻이 없어 상태 색과
// 섞이지 않는다
constexpr QRgb kFocus = 0xffffff;
constexpr QRgb kSelectCaretBg = 0x1c3f5e;  // 선택칸 화살표 칸

// ---------------------------------------------------------------- 운용 로그 종류 배지
// 종류를 나누는 태그라 상태 색과 겹치지 않는 색
constexpr QRgb kTagCommand = kBorderDefault;
constexpr QRgb kTagReply = 0x00777a;
constexpr QRgb kTagGateway = kTooltipBg;
constexpr QRgb kTagFc = 0x48417f;
constexpr QRgb kTagGcs = kDivider;
constexpr QRgb kSearchHighlight = 0x1d4279;  // 녹화 재생 로그의 지금 줄

// ---------------------------------------------------------------- 팝업과 대화상자
constexpr QRgb kMenuDivider = 0x51555b;
constexpr QRgb kLogHeaderText = 0xa9c4ee;  // 운용 로그 머리줄 글자
constexpr QRgb kScrim = 0x000000;          // 모달 대화상자 뒤 화면 가림
constexpr std::int32_t kScrimAlpha = 128;

// ---------------------------------------------------------------- 영상 위 표식 (MIL-STD-2525)
// 탐지는 피아를 판정하지 않으므로 모두 미확인(노랑), 운용자가 고른 표적과 추종 표적은 적(빨강).
// 값은 2525 의 기본색이다. 종류는 색이 아닌 글자로 구분한다. 미확인 노랑은 상태 기호가 아니라 표적
// 틀에만 쓴다
constexpr QRgb kTargetUnknown = 0xffff00;    // 미확인 표적 (탐지 상자, 스캔 표식, 방위 띠)
constexpr QRgb kOverlaySelected = 0xff0000;  // 고른 표적, 추종 표적, 추종 오차

// 스캔 원판의 훑는 빛줄기와 지나간 자리: 상태가 아닌 데이터라 데이터 시각화 색을 쓰고, 지나간
// 시간은 진하기로 보인다. 빛줄기와 잔광을 같은 색으로 해 빛이 지나간 흔적으로 읽힌다
constexpr QRgb kDataVisualization1 = 0x00c7cb;

constexpr QRgb kTrail = 0xffffff;  // 지도 항적: 남색 바탕에서 대비가 가장 높다 (약 16 : 1)

// 카메라 시야 (지도, 스캔 원판): 흰색 채움과 연회색 외곽선. 청록 스캔 빛줄기와 겹쳐도 구분된다
constexpr QRgb kFovFill = 0xffffff;
constexpr QRgb kFovEdge = 0xcccccc;

// 스캔 원판 바탕 빛: 가운데 청록에서 가장자리 남색으로. 빛줄기 앞선은 거의 흰 청록
constexpr QRgb kGlowCenter = 0x0a4a5c;
constexpr QRgb kGlowMid = 0x061c33;
constexpr QRgb kGlowEdge = 0x04122a;
constexpr QRgb kSweepEdge = 0xe8fbff;

// 명령 진행 막대: 화면의 다른 진행 표시(스캔 원판, 시야)와 같은 청록 하나로 맞춘다
constexpr QRgb kProgressFill = kDataVisualization1;
constexpr QRgb kProgressTrack = kSurfaceTile;
constexpr QRgb kProgressHead = kSweepEdge;
constexpr QRgb kProgressHeadGlow = 0x6ff2f4;
constexpr QRgb kMarkerTrack = 0x35c6e0;  // 녹화 재생 시간 막대의 추종 시작, 정지 표식

// 자세 계기의 땅 쪽 그러데이션이 수평선에 닿는 색 (바탕과 패널 본문 사이 명암). 아래로 갈수록 영상
// 바탕까지 어두워진다
constexpr QRgb kHorizonGround = 0x06193a;

// 시작 화면 무대: 바닥 원판 가운데 빛, 스캔 잔광과 지형 격자, 아직 훑지 않은 지형 격자
constexpr QRgb kEntryFloorCenter = 0x0d2a5a;
constexpr QRgb kEntryScanGlow = kColumnDivider;
constexpr QRgb kEntryTerrain = kDivider;

// 시작 화면 기체 재질. 상태 값이 없는 화면이라 상태 색 대신 실물 재질 색으로 그린다
constexpr QRgb kMaterialPlate = 0x22262d;           // 위, 아래판 (카본)
constexpr QRgb kMaterialMetal = kCiFuturisticGray;  // 나사, 허브, 케이블 타이
constexpr QRgb kMaterialFc = 0x1e2025;
constexpr QRgb kMaterialMissionComputer = kCiFuturisticGray;
constexpr QRgb kMaterialGnss = 0x1a1c20;
constexpr QRgb kMaterialSensor = 0x4a5260;
constexpr QRgb kMaterialBattery = 0x3a4250;
constexpr QRgb kMaterialStrap = 0x30353d;  // 탑재물을 묶는 띠
constexpr QRgb kMaterialCamera = 0x16181c;

// 팔, 몸통 옆면, 착륙 다리는 한 색이다 (옆면의 로고 바탕이 끊기지 않게). 이 바탕에는 흰 로고를 쓴다
constexpr QRgb kMaterialArm = 0x1a1c20;
constexpr QRgb kMaterialLeg = kMaterialArm;
constexpr QRgb kMaterialLogo = 0xffffff;
constexpr QRgb kMaterialMotor = 0xd2d6dc;  // 은색
constexpr QRgb kMaterialPropeller = 0x1c1f24;
constexpr QRgb kMaterialPropellerDisc = 0x0e1014;  // 빨리 돌 때 보이는 흐린 원판

// 탑재물
constexpr QRgb kMaterialBalloon = 0x6f5a30;      // 원뿔과 몸통
constexpr QRgb kMaterialBalloonSeam = 0x2e2818;  // 이음매와 홈
constexpr QRgb kMaterialBalloonTip = 0xa9abad;   // 앞 꼭지
constexpr QRgb kMaterialBalloonNeck = 0x574a2a;  // 뒤 경사와 목
constexpr QRgb kMaterialBalloonRing = 0x8c8d8e;  // 뒤 고리
constexpr QRgb kMaterialRc = 0xe6e8ea;
constexpr QRgb kMaterialEsc = 0x5b6573;

// 반투명 색 (영상 위 십자선, 글자 뒤 띠 등). 투명도(0~255)는 쓰는 곳이 정한다
inline QColor withAlpha(QRgb rgb, std::int32_t alpha) noexcept {
    QColor color(rgb);
    color.setAlpha(alpha);
    return color;
}

// ---------------------------------------------------------------- 크기 (px)
// 버튼: 높이 세 가지 (Astro Button)
constexpr std::int32_t kBorderWidth = 1;
constexpr std::int32_t kButtonS = 28;
constexpr std::int32_t kButtonM = 36;  // 기본. 입력칸과 선택칸도 같은 높이
constexpr std::int32_t kButtonL = 44;  // 오른쪽 명령 칸의 명령

// 간격: 4 px 격자
constexpr std::int32_t kSpacing1 = 4;
constexpr std::int32_t kSpacing2 = 8;
constexpr std::int32_t kSpacing3 = 12;
constexpr std::int32_t kSpacing4 = 16;
constexpr std::int32_t kSpacing5 = 20;

// 모서리
constexpr std::int32_t kRadiusTooltip = 1;
constexpr std::int32_t kRadiusInner = 2;  // 알림 띠 안쪽 상자

// 패널 안의 모든 상자 (버튼, 입력칸, 선택칸, 숫자 칸, 상태 칸, 배지, 팝업, 알림 상자)
constexpr std::int32_t kRadiusBase = 3;

// 글자 크기: 14, 16, 18, 20, 24 단계. 가장 작은 글자는 14 px
constexpr std::int32_t kFontSmall = 14;
constexpr std::int32_t kFontKeyCap = 12;  // 가상 조종기 키캡 글자 (20 px 아이콘 안)
constexpr std::int32_t kFontBody = 16;    // 기본
constexpr std::int32_t kFontClock = 24;
constexpr std::int32_t kFontIcon = 20;      // 글자로 그린 아이콘
constexpr std::int32_t kFontDisplay = 24;   // 큰 숫자
constexpr std::int32_t kFontAppTitle = 24;  // 상단 막대의 체계 이름
constexpr std::int32_t kFontDialogTitle = 24;
constexpr std::int32_t kFontPanelTitle = 20;  // 패널 제목 띠

// 배치: 한 칸 안의 요소는 왼쪽, 오른쪽 끝이 패널 안쪽 여백만큼 들어간 같은 세로선에 온다. 상자(숫자
// 칸, 상태 띠) 안 글자는 상자 안쪽 여백만큼 더 들어간다
constexpr std::int32_t kPanelPaddingX = kSpacing4;  // 패널 제목 띠, 본문, 안내, 영상 칸의 좌우 여백
constexpr std::int32_t kPanelPaddingY = kSpacing4;  // 패널 본문의 위아래 여백
constexpr std::int32_t kGap = kSpacing2;            // 한 묶음 안 조작부 사이
constexpr std::int32_t kGroupGap = kSpacing4;       // 묶음 사이
constexpr std::int32_t kLabelGap = kSpacing1;       // 입력칸 이름과 칸 사이
constexpr std::int32_t kHelpGap = kSpacing2;        // 칸과 안내 글 사이
constexpr std::int32_t kBoxPadding = kSpacing3;     // 상자 안쪽 좌우 여백

// 그 밖
constexpr std::int32_t kTileHeight = 36;        // 숫자 칸 (24 px 숫자가 들어가는 높이)
constexpr std::int32_t kListRowHeight = 24;     // 상태 목록 한 줄
constexpr std::int32_t kBannerHeight = 52;      // 알림 띠 최소 높이
constexpr std::int32_t kBannerStripe = 11;      // 알림 띠 안쪽 상자 왼쪽 상태 색 띠
constexpr std::int32_t kSelectCaretWidth = 32;  // 선택칸 오른쪽 화살표 칸
constexpr std::int32_t kSelectCaretIcon = 20;   // 화살표 칸 안 꺾쇠 아이콘

// 스위치: 트랙 32 × 12, 손잡이 20 px 원
constexpr std::int32_t kSwitchTrackWidth = 32;
constexpr std::int32_t kSwitchTrackHeight = 12;
constexpr std::int32_t kSwitchThumb = 20;

constexpr std::int32_t kErrorIcon = 20;          // 입력 오류 글 앞 경고 아이콘
constexpr std::int32_t kPanelHeaderHeight = 40;  // 패널 제목 띠 (여백 8 + 줄 24 + 여백 8)
constexpr std::int32_t kClockHeight = 40;        // 시계 값 상자
constexpr std::int32_t kDialogWidth = 448;
constexpr std::int32_t kPopUpArrow = 8;  // 팝업 화살표가 나오는 높이
constexpr std::int32_t kScrollbarWidth = 16;
constexpr std::int32_t kScrollbarRadius = 4;

// 표: 머리 칸 40, 본문 칸 32 (행 아래 선 1 px 포함)
constexpr std::int32_t kTableHeaderHeight = 40;
constexpr std::int32_t kTableRowHeight = 32;

// 슬라이더: 지나온 구간 4 px, 나머지 1 px, 손잡이 20 px 원 (테두리 2 px)
constexpr std::int32_t kSliderTrackActive = 4;
constexpr std::int32_t kSliderTrackInactive = 1;
constexpr std::int32_t kSliderThumb = 20;
constexpr std::int32_t kSliderThumbBorder = 2;

}  // namespace argus::gcs::theme

#endif  // ARGUS_GCS_THEME_THEME_TOKENS_HPP_
