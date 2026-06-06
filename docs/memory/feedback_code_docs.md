---
name: feedback-code-docs
description: Routing3D 코드 작성 시 한글 상세 문서화 + 상단 실행명령어 필수 규칙
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 12358f83-5328-4fc4-96ed-e4e7bd4740f5
---

Routing3D 프로젝트의 모든 소스 코드 파일은 **한글로 상세하게** 문서화한다.

- 파일 **상단 헤더**에 반드시 **실행명령어**를 포함한다 (예: pytest 실행, 스크립트 실행 커맨드).
- 헤더에 **전체흐름도**(모듈/알고리즘 흐름)를 한글로 작성한다.
- **알고리즘, 함수, 변수**를 한글로 자세히 설명한다 (docstring·주석).

**Why:** 사용자가 2026-05-27 Routing3D 프로젝트에서 명시적으로 요청. 코드가 한글로 자기설명적이기를 원함.

**How to apply:** 이 프로젝트에서 소스 파일을 새로 쓰거나 수정할 때마다 적용. 이는 Claude Code 기본값인 "주석 최소화" 스타일을 **의도적으로 덮어쓰는** 예외다 — 이 프로젝트에서는 상세 한글 주석/docstring 을 적극 작성한다. 관련: [[project-routing3d]]
