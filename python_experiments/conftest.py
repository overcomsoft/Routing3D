"""pytest 공통 설정
================================================================================
이 파일 위치(python_experiments/)가 pytest 의 rootdir 이자 sys.path 진입점이 되어
`import routing3d_py` 가 동작한다.

[마커]
  db : 실제 PostgreSQL 연결이 필요한 통합 테스트. 연결 불가 시 테스트 내부에서
       자동 skip 된다. 명시적으로 제외하려면:
         ..\\.venv\\Scripts\\python.exe -m pytest -m "not db"
================================================================================
"""


def pytest_configure(config):
    """커스텀 마커를 등록해 'unknown marker' 경고를 방지한다."""
    config.addinivalue_line(
        "markers", "db: 실제 PostgreSQL 연결이 필요한 통합 테스트 (연결 불가 시 skip)"
    )
