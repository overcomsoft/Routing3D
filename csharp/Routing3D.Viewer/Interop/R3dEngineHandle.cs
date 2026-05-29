// 네이티브 엔진 핸들 SafeHandle — 누수/이중해제 방지
// =============================================================================
//   r3d_create() 가 돌려준 R3dEngine* 를 SafeHandle 로 감싸 GC/Dispose 시
//   r3d_destroy() 가 정확히 한 번 호출되게 한다.
// =============================================================================
using System;
using Microsoft.Win32.SafeHandles;

namespace Routing3D.Viewer.Interop
{
    internal sealed class R3dEngineHandle : SafeHandleZeroOrMinusOneIsInvalid
    {
        public R3dEngineHandle() : base(ownsHandle: true) { }

        public static R3dEngineHandle Create()
        {
            var h = new R3dEngineHandle();
            h.SetHandle(Native.r3d_create());
            return h;
        }

        protected override bool ReleaseHandle()
        {
            Native.r3d_destroy(handle);
            return true;
        }
    }
}
