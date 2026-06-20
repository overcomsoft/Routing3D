# Routing3D OpenVDB Cross-Platform Build

Routing3D now treats OpenVDB as the production occupancy backend when `USE_OPENVDB=ON`.
That option is ON by default and is required by default through `ROUTING3D_REQUIRE_OPENVDB=ON`.

## Dependencies

Use vcpkg on Windows, macOS, and Linux:

```bash
git clone https://github.com/microsoft/vcpkg.git
./vcpkg/bootstrap-vcpkg.sh
./vcpkg/vcpkg install openvdb:x64-linux
```

Windows PowerShell:

```powershell
git clone https://github.com/microsoft/vcpkg.git
.\vcpkg\bootstrap-vcpkg.bat
.\vcpkg\vcpkg install openvdb:x64-windows
$env:VCPKG_ROOT = "D:\vcpkg"
```

macOS:

```bash
./vcpkg/vcpkg install openvdb:arm64-osx
```

## Configure, Build, Test

Windows:

```powershell
cmake --preset windows-vs2022-x64-openvdb
cmake --build --preset windows-vs2022-x64-openvdb
ctest --preset windows-vs2022-x64-openvdb
```

Linux:

```bash
cmake --preset linux-ninja-x64-openvdb
cmake --build --preset linux-ninja-x64-openvdb
ctest --preset linux-ninja-x64-openvdb
```

macOS Apple Silicon:

```bash
cmake --preset macos-ninja-arm64-openvdb
cmake --build --preset macos-ninja-arm64-openvdb
ctest --preset macos-ninja-arm64-openvdb
```

## Legacy Fallback

For emergency comparison only:

```bash
cmake -S . -B build/legacy -DUSE_OPENVDB=OFF -DROUTING3D_REQUIRE_OPENVDB=OFF
```

The C API and C# wrapper should use the OpenVDB-enabled `routing3d_capi` binary for production.