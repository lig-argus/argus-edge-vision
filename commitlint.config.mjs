// Conventional Commits 규칙 — 로컬(husky)과 CI 가 모두 이 파일을 사용
// type/scope 를 바꾸면 CONTRIBUTING.md 도 함께 수정할 것
export default {
  extends: ['@commitlint/config-conventional'],
  rules: {
    'type-enum': [
      2,
      'always',
      ['feat', 'fix', 'docs', 'style', 'refactor', 'perf', 'test', 'build', 'ci', 'chore', 'revert'],
    ],
    'scope-enum': [
      2,
      'always',
      ['onboard', 'gcs', 'common', 'simulation', 'models', 'tools', 'docs', 'ci', 'deps'],
    ],
    'scope-case': [2, 'always', 'lower-case'],
    // subject 는 한글 — Jetson, GCS 같은 고유명사로 시작할 수 있으므로 대소문자 검사 끔
    'subject-case': [0],
    'header-max-length': [2, 'always', 72],
    'body-max-line-length': [1, 'always', 100],
  },
};
