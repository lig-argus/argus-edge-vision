# 산출물 목록

방산 SW 개발 프로세스(MIL-STD-498 기반 방위사업청 작성가이드) 산출물의 명칭과 저장소 안 파일 위치의 대응표이다. 위에서 아래로 상위 문서에서 하위 문서 순서다. 제출용 docx·hwp 가 필요하면 아래 Markdown 에서 변환하고, 변환한 파일은 저장소 밖(release 첨부, 공유 드라이브)에서 관리한다.

| 산출물 | 명칭 | 파일 |
|---|---|---|
| SSS | 체계/부체계 규격서 | [`docs/requirements/sss.md`](requirements/sss.md) |
| SSDD | 체계/부체계 설계기술서 | [`docs/design/ssdd.md`](design/ssdd.md) |
| SRS | 소프트웨어 요구사항명세서 | [`docs/requirements/srs.md`](requirements/srs.md) |
| SDD | 소프트웨어 설계기술서 | [`docs/design/sdd.md`](design/sdd.md) (클래스 상세는 [`docs/design/uml/`](design/uml/)) |
| ICD | 인터페이스 설계기술서 | [`docs/spec/icd.md`](spec/icd.md) (FC·통제기 MAVLink). 프로세스 사이는 [`docs/spec/process_interfaces.md`](spec/process_interfaces.md), 영상은 [`docs/spec/video.md`](spec/video.md) |
| STD | 소프트웨어 통합시험절차서 | [`docs/verification/std.md`](verification/std.md) |
| STR | 소프트웨어 통합시험결과서 | [`docs/verification/str.md`](verification/str.md) |

- 시험계획(STP)은 STD 1~3장으로 갈음한다.
