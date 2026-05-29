# Routing3D 실행 스크립트 (Windows PowerShell)
# =============================================================================
# [이 스크립트가 하는 일]
#   C++ 라우팅 엔진 CLI(routing3d_cli)를 (필요 시 빌드 후) 실행한다.
#   외부 의존성(OpenVDB/FCL/pybind11) 없이 코어만으로 빌드되므로 바로 돌아간다.
#
# [실행 예]
#   .\run.ps1                                  # 내장 데모(골든03: 5개 배관 순차) 실행
#   .\run.ps1 demo --out out.scene.txt         # 데모 + 결과를 scene.txt 로 저장
#   .\run.ps1 route --in scene.txt --out routed.scene.txt --mode multi
#   .\run.ps1 route --in scene.txt --mode single
#   .\run.ps1 summary --in scene.txt
# =============================================================================
param([Parameter(ValueFromRemainingArguments = $true)][string[]]$CliArgs)

$ErrorActionPreference = "Stop"
try { [Console]::OutputEncoding = [System.Text.Encoding]::UTF8 } catch {}  # 한글 출력.
$root = $PSScriptRoot
$buildDir = Join-Path $root "cpp\build"
$exe = Join-Path $buildDir "Release\routing3d_cli.exe"

# 빌드 디렉토리가 없으면 최초 구성(코어만; 무거운 옵션은 기본 OFF).
if (-not (Test-Path (Join-Path $buildDir "CMakeCache.txt"))) {
    Write-Host "[run] CMake 구성 중..." -ForegroundColor Cyan
    cmake -S (Join-Path $root "cpp") -B $buildDir -G "Visual Studio 17 2022" -A x64 | Out-Host
}

Write-Host "[run] routing3d_cli 빌드 중..." -ForegroundColor Cyan
cmake --build $buildDir --config Release --target routing3d_cli | Out-Host

if (-not (Test-Path $exe)) { throw "빌드 산출물을 찾을 수 없습니다: $exe" }

# 인자가 없으면 데모 실행.
if (-not $CliArgs -or $CliArgs.Count -eq 0) { $CliArgs = @("demo") }

Write-Host "[run] 실행: routing3d_cli $($CliArgs -join ' ')" -ForegroundColor Green
& $exe @CliArgs
exit $LASTEXITCODE
