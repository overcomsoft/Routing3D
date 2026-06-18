#!/usr/bin/env bash
# =========================================================================
#  sync-memory.sh — Claude 메모리 <-> 저장소 동기화 (Routing3D, bash/git-bash)
# =========================================================================
#  PowerShell 판(scripts/sync-memory.ps1)의 bash 동치. macOS/Linux/git-bash.
#
#  사용법 (저장소 어디서든):
#    bash scripts/sync-memory.sh import   # 저장소 정본 -> 라이브 (clone/pull 직후)
#    bash scripts/sync-memory.sh export   # 라이브 -> 저장소 정본 (commit 전)
#    bash scripts/sync-memory.sh status   # 비교만(복사 안 함)
#
#  라이브 메모리 경로는 저장소 절대경로에서 자동 계산
#  (드라이브문자 소문자 + ':' '\' '/' -> '-').
# =========================================================================
set -euo pipefail
MODE="${1:-status}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_MEM="$REPO_ROOT/.claude/memory"

# 저장소 절대경로를 Windows 표기로 환산해 인코딩(git-bash: /d/.. -> d:\..).
# 그 외 OS 에서는 경로 그대로 인코딩.
to_win() {
  local p="$1"
  if [[ "$p" =~ ^/([a-zA-Z])/(.*)$ ]]; then
    echo "${BASH_REMATCH[1]}:\\${BASH_REMATCH[2]//\//\\}"
  else
    echo "$p"
  fi
}
encode_path() {
  local p; p="$(to_win "$1")"
  # 드라이브문자 소문자화
  if [[ "$p" =~ ^([A-Za-z]): ]]; then
    p="$(echo "${p:0:1}" | tr 'A-Z' 'a-z')${p:1}"
  fi
  # ':' '\' '/' -> '-'
  echo "$p" | sed -E 's#[:\\/]#-#g'
}

HOME_DIR="${USERPROFILE:-$HOME}"
# git-bash 에서 USERPROFILE 은 C:\Users\.. 형태일 수 있으니 정규화
HOME_DIR="$(echo "$HOME_DIR" | sed -E 's#^([A-Za-z]):#/\L\1#; s#\\#/#g')"
ENC="$(encode_path "$REPO_ROOT")"
LIVE_MEM="$HOME_DIR/.claude/projects/$ENC/memory"

echo "저장소 정본 : $REPO_MEM"
echo "라이브 메모리: $LIVE_MEM"
echo ""

copy_md() { # $1=src $2=dst
  local src="$1" dst="$2"
  [[ -d "$src" ]] || { echo "원본 폴더가 없습니다: $src" >&2; exit 1; }
  mkdir -p "$dst"
  local n=0
  shopt -s nullglob
  for f in "$src"/*.md; do
    cp -f "$f" "$dst/"
    echo "  복사: $(basename "$f")"
    n=$((n+1))
  done
  echo "$n 개 파일 복사 완료."
}

case "$MODE" in
  import)
    echo "=== import: 저장소 정본 -> 라이브 ==="
    copy_md "$REPO_MEM" "$LIVE_MEM" ;;
  export)
    echo "=== export: 라이브 -> 저장소 정본 ==="
    copy_md "$LIVE_MEM" "$REPO_MEM"
    echo ""
    echo "이제 git add .claude/memory && git commit 으로 동기화하세요." ;;
  status)
    echo "=== status: 양쪽 비교(복사 안 함) ==="
    shopt -s nullglob
    declare -A R L
    for f in "$REPO_MEM"/*.md; do R["$(basename "$f")"]=$(wc -c <"$f"); done
    for f in "$LIVE_MEM"/*.md; do L["$(basename "$f")"]=$(wc -c <"$f"); done
    for n in $(printf '%s\n' "${!R[@]}" "${!L[@]}" | sort -u); do
      if [[ -n "${R[$n]:-}" && -n "${L[$n]:-}" ]]; then
        if [[ "${R[$n]}" == "${L[$n]}" ]]; then echo "  =  $n"
        else echo "  ~  $n  (정본 ${R[$n]}B / 라이브 ${L[$n]}B)"; fi
      elif [[ -n "${R[$n]:-}" ]]; then echo "  >  $n  (정본에만)"
      else echo "  <  $n  (라이브에만)"; fi
    done
    echo ""
    echo "범례: = 동일 / ~ 크기다름 / > 정본에만(import 필요) / < 라이브에만(export 필요)" ;;
  *)
    echo "사용법: $0 {import|export|status}" >&2; exit 2 ;;
esac
