# docx → PDF 변환 (MS Word COM) — Routing3D 보고서용
# =============================================================================
# [이 스크립트가 하는 일]
#   설치된 MS Word 를 COM 으로 띄워 .docx 를 PDF 로 내보낸다(reportlab/pandoc 불필요).
#
# [실행]  (프로젝트 루트에서)
#   .\.venv\Scripts\python.exe python_experiments/out/_gen_dev_report.py   # 먼저 docx 생성
#   powershell -ExecutionPolicy Bypass -File python_experiments/out/_docx_to_pdf.ps1 ^
#       -In docs/routing3d_dev_report.docx -Out docs/routing3d_dev_report.pdf
# =============================================================================
param(
    [Parameter(Mandatory = $true)][string]$In,
    [Parameter(Mandatory = $true)][string]$Out
)
$ErrorActionPreference = "Stop"
$inAbs = (Resolve-Path $In).Path
$outAbs = [System.IO.Path]::GetFullPath((Join-Path (Get-Location) $Out))

$word = New-Object -ComObject Word.Application
$word.Visible = $false
try {
    $doc = $word.Documents.Open($inAbs, $false, $true)  # ConfirmConversions=false, ReadOnly=true
    $wdFormatPDF = 17
    $doc.SaveAs([ref]$outAbs, [ref]$wdFormatPDF)
    $doc.Close($false)
    Write-Host "[pdf] 저장: $outAbs" -ForegroundColor Green
}
finally {
    $word.Quit()
    [System.Runtime.InteropServices.Marshal]::ReleaseComObject($word) | Out-Null
}
