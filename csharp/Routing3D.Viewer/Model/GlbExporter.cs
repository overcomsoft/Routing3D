// Model/GlbExporter.cs
// 자동라우팅 배관 메시 → GLB(glTF 2.0 Binary) 파일 직접 작성.
// 외부 NuGet 패키지 없음 — glTF 2.0 바이너리 스펙을 직접 구현.
//
// 좌표계 변환: Routing3D(Z-up, mm) → glTF(Y-up, m)
//   (x, y, z) → (x/1000, z/1000, -y/1000)
//   Blender · Unity · 온라인 뷰어(gltf.report 등)에서 바로 열린다.
//
// 출력 구조:
//   하나의 mesh → 유틸리티별 primitive 1개씩 + 각 색상 material
//   각 primitive: POSITION(VEC3 float) + NORMAL(VEC3 float) + indices(SCALAR uint)
// =============================================================================
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Windows.Media;

namespace Routing3D.Viewer.Model
{
    internal static class GlbExporter
    {
        // glTF constants
        private const uint MAGIC_GLTF  = 0x46546C67u;  // "glTF"
        private const uint CHUNK_JSON   = 0x4E4F534Au;  // "JSON"
        private const uint CHUNK_BIN    = 0x004E4942u;  // "BIN\0"
        private const int  CT_FLOAT     = 5126;          // GL_FLOAT
        private const int  CT_UINT      = 5125;          // GL_UNSIGNED_INT
        private const int  BT_ARRAY     = 34962;         // ARRAY_BUFFER  (positions/normals)
        private const int  BT_ELEMENT   = 34963;         // ELEMENT_ARRAY_BUFFER (indices)

        /// <summary>
        /// 유틸별 (label, color, positions[], normals[], indices[]) 목록을 GLB 파일로 저장.
        /// positions/normals 는 이미 glTF 좌표계(Y-up, m 단위)로 변환된 float 배열이어야 한다.
        /// </summary>
        public static void Write(string path,
            IList<(string label, Color color, float[] pos, float[] nrm, uint[] idx,
                   float[] pmin, float[] pmax)> parts)
        {
            if (parts.Count == 0)
                throw new InvalidOperationException("내보낼 메시가 없습니다.");

            // ── 1. 바이너리 버퍼 ─────────────────────────────────────────────────
            var binMs = new MemoryStream();
            using var bw = new BinaryWriter(binMs);

            var posOff = new (int off, int len)[parts.Count];
            var nrmOff = new (int off, int len)[parts.Count];
            var idxOff = new (int off, int len)[parts.Count];

            for (int i = 0; i < parts.Count; i++)
            {
                var (_, _, pos, nrm, idx, _, _) = parts[i];

                posOff[i] = WriteFloats(bw, binMs, pos);
                nrmOff[i] = nrm.Length > 0 ? WriteFloats(bw, binMs, nrm) : (-1, 0);
                idxOff[i] = WriteUints(bw, binMs, idx);
            }
            bw.Flush();
            byte[] bin = binMs.ToArray();

            // ── 2. JSON ──────────────────────────────────────────────────────────
            var sb = new StringBuilder();
            sb.Append('{');

            // asset
            sb.Append(@"""asset"":{""version"":""2.0"",""generator"":""Routing3D Viewer""},");
            sb.Append(@"""scene"":0,");
            sb.Append(@"""scenes"":[{""name"":""AutoRoutes"",""nodes"":[0]}],");
            sb.Append(@"""nodes"":[{""name"":""AutoRoutedPipes"",""mesh"":0}],");

            // materials
            sb.Append(@"""materials"":[");
            for (int i = 0; i < parts.Count; i++)
            {
                if (i > 0) sb.Append(',');
                var c  = parts[i].color;
                double r = c.R / 255.0, g = c.G / 255.0, b = c.B / 255.0;
                sb.Append($@"{{""name"":""{Esc(parts[i].label)}"",""pbrMetallicRoughness"":{{");
                sb.Append($@"""baseColorFactor"":[{r:F4},{g:F4},{b:F4},1.0],""metallicFactor"":0.25,""roughnessFactor"":0.55}}}}");
            }
            sb.Append("],");

            // mesh + primitives
            sb.Append(@"""meshes"":[{""name"":""Pipes"",""primitives"":[");
            int accCursor = 0;
            for (int i = 0; i < parts.Count; i++)
            {
                if (i > 0) sb.Append(',');
                bool hasNrm = parts[i].nrm.Length > 0;
                int posAcc = accCursor++;
                int nrmAcc = hasNrm ? accCursor++ : -1;
                int idxAcc = accCursor++;
                sb.Append($@"{{""attributes"":{{""POSITION"":{posAcc}");
                if (hasNrm) sb.Append($@",""NORMAL"":{nrmAcc}");
                sb.Append($@"}},""indices"":{idxAcc},""material"":{i},""mode"":4}}");
            }
            sb.Append("]}],");

            // accessors
            sb.Append(@"""accessors"":[");
            int bvCursor = 0;
            bool firstAcc = true;
            for (int i = 0; i < parts.Count; i++)
            {
                var (_, _, pos, nrm, idx, pmin, pmax) = parts[i];
                int nv = pos.Length / 3;
                bool hasNrm = nrm.Length > 0;

                Sep(sb, ref firstAcc);
                sb.Append($@"{{""bufferView"":{bvCursor++},""componentType"":{CT_FLOAT},""count"":{nv},""type"":""VEC3"",");
                sb.Append($@"""min"":[{pmin[0]:F6},{pmin[1]:F6},{pmin[2]:F6}],""max"":[{pmax[0]:F6},{pmax[1]:F6},{pmax[2]:F6}]}}");

                if (hasNrm)
                    sb.Append($@",{{""bufferView"":{bvCursor++},""componentType"":{CT_FLOAT},""count"":{nv},""type"":""VEC3""}}");

                sb.Append($@",{{""bufferView"":{bvCursor++},""componentType"":{CT_UINT},""count"":{idx.Length},""type"":""SCALAR""}}");
            }
            sb.Append("],");

            // bufferViews
            sb.Append(@"""bufferViews"":[");
            bool firstBv = true;
            for (int i = 0; i < parts.Count; i++)
            {
                bool hasNrm = parts[i].nrm.Length > 0;
                var (po, pl) = posOff[i];
                Sep(sb, ref firstBv);
                sb.Append($@"{{""buffer"":0,""byteOffset"":{po},""byteLength"":{pl},""target"":{BT_ARRAY}}}");

                if (hasNrm)
                {
                    var (no, nl) = nrmOff[i];
                    sb.Append($@",{{""buffer"":0,""byteOffset"":{no},""byteLength"":{nl},""target"":{BT_ARRAY}}}");
                }

                var (io, il) = idxOff[i];
                sb.Append($@",{{""buffer"":0,""byteOffset"":{io},""byteLength"":{il},""target"":{BT_ELEMENT}}}");
            }
            sb.Append("],");

            sb.Append($@"""buffers"":[{{""byteLength"":{bin.Length}}}]}}");

            // ── 3. GLB 헤더 + JSON 청크 + BIN 청크 ──────────────────────────────
            byte[] jsonRaw = Encoding.UTF8.GetBytes(sb.ToString());
            int jsonPad  = Pad4(jsonRaw.Length);
            int binPad   = Pad4(bin.Length);
            uint totalLen = (uint)(12 + 8 + jsonPad + 8 + binPad);

            using var fs = File.Create(path);
            using var fw = new BinaryWriter(fs, Encoding.UTF8, leaveOpen: false);

            fw.Write(MAGIC_GLTF);
            fw.Write(2u);           // version
            fw.Write(totalLen);

            // JSON chunk
            fw.Write((uint)jsonPad);
            fw.Write(CHUNK_JSON);
            fw.Write(jsonRaw);
            for (int i = jsonRaw.Length; i < jsonPad; i++) fw.Write((byte)0x20); // 공백 패딩

            // BIN chunk
            fw.Write((uint)binPad);
            fw.Write(CHUNK_BIN);
            fw.Write(bin);
            for (int i = bin.Length; i < binPad; i++) fw.Write((byte)0x00);      // 0 패딩
        }

        // ── 헬퍼 ─────────────────────────────────────────────────────────────────

        private static (int off, int len) WriteFloats(BinaryWriter bw, MemoryStream ms, float[] data)
        {
            int off = (int)ms.Position;
            foreach (var f in data) bw.Write(f);
            Align4(bw, ms);
            return (off, (int)ms.Position - off);
        }

        private static (int off, int len) WriteUints(BinaryWriter bw, MemoryStream ms, uint[] data)
        {
            int off = (int)ms.Position;
            foreach (var u in data) bw.Write(u);
            Align4(bw, ms);
            return (off, (int)ms.Position - off);
        }

        private static void Align4(BinaryWriter bw, MemoryStream ms)
        {
            int rem = (int)(ms.Position % 4);
            if (rem != 0)
                for (int i = rem; i < 4; i++) bw.Write((byte)0);
        }

        private static int Pad4(int n) => (n + 3) & ~3;

        private static void Sep(StringBuilder sb, ref bool first)
        {
            if (!first) sb.Append(',');
            first = false;
        }

        private static string Esc(string s)
            => s.Replace("\\", "\\\\").Replace("\"", "\\\"");
    }
}
