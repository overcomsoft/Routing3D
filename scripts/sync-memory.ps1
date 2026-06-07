<#
=========================================================================
 sync-memory.ps1 — Claude 메모리 ↔ 저장소 동기화 (Routing3D)
=========================================================================
 목적:
   Claude Code 의 자동 메모리는 저장소 밖
   (~/.claude/projects/<인코딩된-경로>/memory/) 에 저장되어 GitHub 로
   따라오지 않는다. 이 스크립트는 저장소의 정본(docs/memory/) 과
   라이브 메모리 폴더를 양방향 동기화해 여러 PC 에서 메모리를 공유한다.

 사용법 (저장소 어디서든):
   # 새 PC 에서 clone/pull 직후 — 저장소 정본 → 라이브 (개발 시작 전)
   powershell -ExecutionPolicy Bypass -File scripts/sync-memory.ps1 import

   # 작업 중 메모리가 갱신된 뒤, commit 전 — 라이브 → 저장소 정본
   powershell -ExecutionPolicy Bypass -File scripts/sync-memory.ps1 export

   # 어디가 다른지 미리보기(복사 안 함)
   powershell -ExecutionPolicy Bypass -File scripts/sync-memory.ps1 status

 동작:
   - 라이브 폴더 경로는 저장소 절대경로에서 자동 계산
     (드라이브문자 소문자화 + ':' '\' '/' → '-').
   - *.md 만 복사(덮어쓰기). 안전을 위해 대상에만 있는 파일은 삭제하지 않고
     경고만 출력(-Mirror 지정 시 정본에 없는 라이브 파일 삭제).
=========================================================================
#>
param(
  [Parameter(Position = 0)]
  [ValidateSet('import', 'export', 'status')]
  [string]$Mode = 'status',
  [switch]$Mirror
)

$ErrorActionPreference = 'Stop'

# 저장소 루트 = 이 스크립트의 부모(scripts/)의 부모
# ($PSScriptRoot 는 -File 호출 환경에 따라 비어 있을 수 있어 MyInvocation 으로 폴백)
$ScriptDir = $PSScriptRoot
if ([string]::IsNullOrEmpty($ScriptDir)) { $ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path }
if ([string]::IsNullOrEmpty($ScriptDir)) { $ScriptDir = (Get-Location).Path }
$RepoRoot = (Resolve-Path (Join-Path $ScriptDir '..')).Path
$RepoMem  = Join-Path $RepoRoot 'docs\memory'

# 라이브 메모리 경로 계산: 드라이브문자 소문자 + 구분자(':','\','/') → '-'
$enc = $RepoRoot
if ($enc -match '^([A-Za-z]):') { $enc = $enc.Substring(0, 1).ToLower() + $enc.Substring(1) }
$enc = $enc -replace '[:\\/]', '-'
$LiveMem = Join-Path $env:USERPROFILE ".claude\projects\$enc\memory"

Write-Host "저장소 정본 : $RepoMem"
Write-Host "라이브 메모리: $LiveMem"
Write-Host ""

function Copy-Md($src, $dst) {
  if (-not (Test-Path $src)) { throw "원본 폴더가 없습니다: $src" }
  New-Item -ItemType Directory -Force -Path $dst | Out-Null
  $files = Get-ChildItem -Path $src -Filter '*.md' -File
  foreach ($f in $files) {
    Copy-Item -Path $f.FullName -Destination (Join-Path $dst $f.Name) -Force
    Write-Host "  복사: $($f.Name)"
  }
  Write-Host "$($files.Count) 개 파일 복사 완료."
  if ($Mirror) {
    $srcNames = $files.Name
    Get-ChildItem -Path $dst -Filter '*.md' -File |
      Where-Object { $srcNames -notcontains $_.Name } |
      ForEach-Object { Remove-Item $_.FullName -Force; Write-Host "  삭제(mirror): $($_.Name)" }
  } else {
    $srcNames = $files.Name
    Get-ChildItem -Path $dst -Filter '*.md' -File |
      Where-Object { $srcNames -notcontains $_.Name } |
      ForEach-Object { Write-Host "  [경고] 대상에만 존재(미삭제): $($_.Name) — 의도면 -Mirror" }
  }
}

switch ($Mode) {
  'import' {
    Write-Host "=== import: 저장소 정본 → 라이브 ==="
    Copy-Md $RepoMem $LiveMem
  }
  'export' {
    Write-Host "=== export: 라이브 → 저장소 정본 ==="
    Copy-Md $LiveMem $RepoMem
    Write-Host ""
    Write-Host "이제 git add docs/memory && git commit 으로 동기화하세요."
  }
  'status' {
    Write-Host "=== status: 양쪽 파일 비교(복사 안 함) ==="
    $r = @{}; if (Test-Path $RepoMem) { Get-ChildItem $RepoMem -Filter '*.md' -File | ForEach-Object { $r[$_.Name] = $_.Length } }
    $l = @{}; if (Test-Path $LiveMem) { Get-ChildItem $LiveMem -Filter '*.md' -File | ForEach-Object { $l[$_.Name] = $_.Length } }
    $all = ($r.Keys + $l.Keys | Sort-Object -Unique)
    foreach ($n in $all) {
      $inR = $r.ContainsKey($n); $inL = $l.ContainsKey($n)
      if ($inR -and $inL) {
        if ($r[$n] -eq $l[$n]) { Write-Host "  =  $n" }
        else { Write-Host "  ~  $n  (정본 $($r[$n])B / 라이브 $($l[$n])B)" }
      }
      elseif ($inR) { Write-Host "  >  $n  (정본에만)" }
      else { Write-Host "  <  $n  (라이브에만)" }
    }
    Write-Host ""
    Write-Host "범례: = 동일 / ~ 크기다름 / > 정본에만(import 필요) / < 라이브에만(export 필요)"
  }
}
