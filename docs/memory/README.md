# Claude 메모리 스냅샷

이 폴더는 Claude Code 의 **영속 메모리**(사용자 홈
`~/.claude/projects/d--DINNO-DEV-AI-AutoRouting-Routing3D/memory/`)를 git 에 보관한
스냅샷이다. 원본은 repo 밖(사용자 홈)에 있어 PC/세션 간 공유가 안 되므로, 다른 PC·새
환경에서 동일한 컨텍스트(사용자 선호·프로젝트 작업로그·외부 시스템 참조)를 복원하려고
여기에 복사해 둔다.

- `MEMORY.md` — 인덱스(한 줄/메모리, 세션 시작 시 자동 로드)
- `feedback_*.md` — 작업 방식 피드백(문서화 규칙·자동승인·메일 금지)
- `project_routing3d.md` — 프로젝트 작업로그(시간순). **최신 정본은 repo `CLAUDE.md`**
- `reference_*.md` — 외부 시스템(DDW_AI_DB 장애물/씬 스키마) 참조

> 정본은 `CLAUDE.md` 와 git 이력. 이 스냅샷은 메모리 복원용 사본이며 자동 동기화되지 않으니,
> 메모리를 갱신하면 수동으로 다시 복사해 커밋한다.

## 복원 방법

```powershell
Copy-Item docs\memory\*.md `
  "$env:USERPROFILE\.claude\projects\d--DINNO-DEV-AI-AutoRouting-Routing3D\memory\" -Force
```
