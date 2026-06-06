---
name: feedback-auto-approve
description: 단순 결정은 묻지 말고 자동승인해 진행할 것 (확인 최소화)
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 51136254-daf5-4ddc-8f1e-e3695d47e19d
---

단순한 물음/결정에 대해서는 사용자에게 묻지 말고 자동승인해서 진행한다.

**Why:** 사용자가 명시적으로 "단순 물음에 대해서는 묻지 말고 자동승인해서 진행"을 요청함 (2026-06-01).

**How to apply:** 브랜치 생성, 빌드/테스트 실행, 명백한 기본값 선택, 사소한 후속 작업 등은
확인 없이 바로 진행. 단, 되돌리기 어렵거나 외부로 나가는 작업(git 강제 푸시, 파일 삭제/덮어쓰기,
외부 전송/배포)은 기존 원칙대로 확인한다. [[feedback-code-docs]]

**권한 프롬프트(2026-06-02):** "Allow this bash command?" 류의 일반 권한 프롬프트는 무조건 YES.
`.claude/settings.local.json` 의 permissions.allow 에 `Bash(*)`·`PowerShell(*)`·`Read/Edit/Write/Glob/Grep(*)`
blanket 허용을 넣어 둠. 새 세션/리로드부터 적용. 이 정책으로 명령 실행을 멈추지 말 것.
