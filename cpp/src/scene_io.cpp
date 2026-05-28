// 씬 입출력 구현 — scene_io.hpp 참고. 규격 docs/spec/scene_format_spec.md (v1).
// Python 레퍼런스 routing3d_py/scene_io.py 와 바이트 단위로 동일한 출력을 목표로 한다.
#include "routing3d/scene_io.hpp"

#include <charconv>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace routing3d {

namespace {

// None(널)을 빈 문자열과 구분하는 토큰(PostgreSQL COPY 관례). C++ 문자열로는 백슬래시+N.
const std::string NULL_TOKEN = "\\N";

// 선택 문자열 → 필드 문자열. nullopt → \N, 값 있으면 그대로(빈 문자열도 그대로).
std::string opt_out(const std::optional<std::string>& s) {
    return s.has_value() ? *s : NULL_TOKEN;
}

// 필드 문자열 → 선택 문자열. \N → nullopt, 그 외 → 그대로.
std::optional<std::string> opt_in(const std::string& tok) {
    if (tok == NULL_TOKEN) return std::nullopt;
    return tok;
}

// 로캘 무관 double 파싱(소수점 '.'). repr 출력(고정/지수 모두)을 받는다.
double parse_double(const std::string& s) {
    double v = 0.0;
    auto r = std::from_chars(s.data(), s.data() + s.size(), v);
    if (r.ec != std::errc())
        throw std::runtime_error("scene.txt: float 파싱 실패: '" + s + "'");
    return v;
}

long long parse_ll(const std::string& s) {
    long long v = 0;
    std::from_chars(s.data(), s.data() + s.size(), v);
    return v;
}

int parse_int(const std::string& s) { return static_cast<int>(parse_ll(s)); }

// 텍스트를 줄 단위로 분리(개행 '\n' 기준, 각 줄의 뒤 '\r' 제거).
std::vector<std::string> split_lines(const std::string& text) {
    std::vector<std::string> out;
    std::string cur;
    for (char ch : text) {
        if (ch == '\n') {
            if (!cur.empty() && cur.back() == '\r') cur.pop_back();
            out.push_back(cur);
            cur.clear();
        } else {
            cur.push_back(ch);
        }
    }
    if (!cur.empty()) {
        if (cur.back() == '\r') cur.pop_back();
        out.push_back(cur);
    }
    return out;
}

// TAB 단일 분리(공백 분리 금지 — 이름에 공백 포함). 빈 필드 보존.
std::vector<std::string> split_tabs(const std::string& line) {
    std::vector<std::string> out;
    std::string cur;
    for (char ch : line) {
        if (ch == '\t') {
            out.push_back(cur);
            cur.clear();
        } else {
            cur.push_back(ch);
        }
    }
    out.push_back(cur);
    return out;
}

// 공백만 있는 줄인지(빈 줄 스킵용).
bool is_blank(const std::string& s) {
    for (char ch : s)
        if (ch != ' ' && ch != '\t' && ch != '\r' && ch != '\v' && ch != '\f') return false;
    return true;
}

}  // namespace

// =============================================================================
// 실수 표기: Python repr(float) 와 동일한 최단 왕복 표기 (계약 F4)
//   1) std::to_chars(scientific) 로 최단 유효숫자 + 지수를 얻고(Ryū = dtoa mode0 동일),
//   2) Python 규칙(decpt<=-4 또는 >16 이면 지수표기)으로 재포맷한다.
//   decpt = 소수점 왼쪽 자릿수 = (과학표기 지수 E) + 1.
// =============================================================================
std::string format_repr_double(double x) {
    if (std::isnan(x)) return "nan";
    if (std::isinf(x)) return x < 0 ? "-inf" : "inf";

    char buf[64];
    auto res = std::to_chars(buf, buf + sizeof(buf), x, std::chars_format::scientific);
    std::string sci(buf, res.ptr);  // 예: "-1.2345e+03", "5e+01", "0e+00"

    size_t i = 0;
    bool neg = false;
    if (sci[i] == '-') { neg = true; ++i; }

    std::string digits;
    digits.push_back(sci[i++]);                 // 첫 유효숫자.
    if (i < sci.size() && sci[i] == '.') {
        ++i;
        while (i < sci.size() && sci[i] != 'e') digits.push_back(sci[i++]);
    }
    ++i;                                          // 'e' 스킵.
    std::string exp_str = sci.substr(i);          // 예: "+03", "-05".
    if (!exp_str.empty() && exp_str[0] == '+')    // from_chars(int)는 '+'를 거부 → 제거.
        exp_str.erase(0, 1);
    const int E = parse_int(exp_str);             // 과학표기 지수.

    const int ndigits = static_cast<int>(digits.size());
    const int decpt = E + 1;

    std::string body;
    if (decpt <= -4 || decpt > 16) {
        // 지수표기: d[.ddd]e±XX (지수 최소 2자리, 부호 항상).
        body = digits.substr(0, 1);
        if (ndigits > 1) body += "." + digits.substr(1);
        const int e = E;
        const char esign = (e < 0) ? '-' : '+';
        std::string edig = std::to_string(std::abs(e));
        if (edig.size() < 2) edig = "0" + edig;
        body += "e";
        body += esign;
        body += edig;
    } else if (decpt <= 0) {
        body = "0.";
        body.append(static_cast<size_t>(-decpt), '0');
        body += digits;
    } else if (decpt >= ndigits) {
        body = digits;
        body.append(static_cast<size_t>(decpt - ndigits), '0');
        body += ".0";
    } else {
        body = digits.substr(0, static_cast<size_t>(decpt)) + "." +
               digits.substr(static_cast<size_t>(decpt));
    }
    return neg ? ("-" + body) : body;
}

// =============================================================================
// 쓰기 (SceneDoc → scene.txt 문자열). Python dumps_scene 과 동일 바이트.
// =============================================================================
std::string dumps_scene(const SceneDoc& doc) {
    std::ostringstream out;
    const auto ff = [](double v) { return format_repr_double(v); };

    out << "# Routing3D scene file \xe2\x80\x94 units: mm\n";  // — = U+2014 (UTF-8).
    out << "@format " << SCENE_FORMAT_TAG << "\n";
    out << "@version " << SCENE_FORMAT_VERSION << "\n\n";

    // ---- [grid]
    out << "[grid]\n";
    out << "cell_mm\t" << ff(doc.cell_mm) << "\n";
    out << "origin\t" << ff(doc.origin.x) << "\t" << ff(doc.origin.y) << "\t" << ff(doc.origin.z) << "\n";
    out << "shape\t" << doc.shape.i << "\t" << doc.shape.j << "\t" << doc.shape.k << "\n\n";

    // ---- [params]
    const RouteParams& p = doc.params;
    out << "[params]\n";
    out << "cell_mm\t" << ff(p.cell_mm) << "\n";
    out << "w_turn\t" << ff(p.w_turn) << "\n";
    out << "w_clear\t" << ff(p.w_clear) << "\n";
    out << "clearance_radius\t" << p.clearance_radius << "\n";
    out << "clearance_connectivity\t" << p.clearance_connectivity << "\n";
    out << "w_tier";
    for (const auto& [z, v] : p.w_tier)  // std::map 이라 키(z) 오름차순. 각 토큰 앞에 TAB.
        out << "\t" << z << ":" << ff(v);
    out << "\n\n";

    // ---- [obstacles]
    out << "[obstacles]\tcount=" << doc.obstacles.size() << "\n";
    out << "# minx\tminy\tminz\tmaxx\tmaxy\tmaxz\tost_type\tname\tobject_id\tddworks_type\n";
    for (const Obstacle& o : doc.obstacles) {
        out << ff(o.min_xyz.x) << "\t" << ff(o.min_xyz.y) << "\t" << ff(o.min_xyz.z) << "\t"
            << ff(o.max_xyz.x) << "\t" << ff(o.max_xyz.y) << "\t" << ff(o.max_xyz.z) << "\t"
            << opt_out(o.ost_type) << "\t" << opt_out(o.name) << "\t"
            << opt_out(o.object_id) << "\t" << opt_out(o.ddworks_type) << "\n";
    }
    out << "\n";

    // ---- [tasks]
    out << "[tasks]\tcount=" << doc.tasks.size() << "\n";
    out << "# sx\tsy\tsz\tgx\tgy\tgz\tutility\tutility_group\tstart_name\tend_name\tend_instance_guid\n";
    for (const RouteTask& t : doc.tasks) {
        out << ff(t.start_mm.x) << "\t" << ff(t.start_mm.y) << "\t" << ff(t.start_mm.z) << "\t"
            << ff(t.end_mm.x) << "\t" << ff(t.end_mm.y) << "\t" << ff(t.end_mm.z) << "\t"
            << opt_out(t.utility) << "\t" << opt_out(t.utility_group) << "\t"
            << opt_out(t.start_name) << "\t" << opt_out(t.end_name) << "\t"
            << opt_out(t.end_instance_guid) << "\n";
    }
    out << "\n";

    // ---- [results] (선택; None 아닌 결과 수만큼)
    size_t n_res = 0;
    for (const auto& r : doc.results)
        if (r.has_value()) ++n_res;
    if (n_res) {
        out << "[results]\tcount=" << n_res << "\n";
        for (size_t idx = 0; idx < doc.results.size(); ++idx) {
            const auto& opt = doc.results[idx];
            if (!opt.has_value()) continue;
            const SceneResult& r = *opt;
            out << "[result]\ttask=" << idx << "\n";
            out << "success\t" << (r.success ? 1 : 0) << "\n";
            out << "length_mm\t" << ff(r.length_mm) << "\n";
            out << "cost_mm\t" << ff(r.cost_mm) << "\n";
            out << "turns\t" << r.turns << "\n";
            out << "expanded_nodes\t" << r.expanded_nodes << "\n";
            out << "elapsed_ms\t" << ff(r.elapsed_ms) << "\n";
            if (r.path.has_value()) {
                out << "[path]\ttask=" << idx << "\tcount=" << r.path->size() << "\n";
                for (const Cell& c : *r.path)
                    out << c.i << "\t" << c.j << "\t" << c.k << "\n";
            }
            if (r.visited.has_value()) {
                out << "[visited]\ttask=" << idx << "\tcount=" << r.visited->size() << "\n";
                for (const Cell& c : *r.visited)
                    out << c.i << "\t" << c.j << "\t" << c.k << "\n";
            }
        }
        out << "\n";
    }

    return out.str();
}

void write_scene(const std::string& path, const SceneDoc& doc) {
    std::ofstream f(path, std::ios::binary);  // binary → \n 그대로(윈도우 CRLF 변환 방지).
    if (!f) throw std::runtime_error("scene.txt 쓰기 실패: " + path);
    const std::string text = dumps_scene(doc);
    f.write(text.data(), static_cast<std::streamsize>(text.size()));
}

// =============================================================================
// 읽기 (scene.txt 문자열 → SceneDoc). 단순 상태기계 파서(규격 §7).
// =============================================================================
SceneDoc loads_scene(const std::string& text) {
    SceneDoc doc;
    bool has_cell = false;

    std::map<std::string, std::vector<std::string>> params_kv;  // params 섹션 키→값들.
    std::map<int, std::map<std::string, std::string>> result_kv;
    std::map<int, std::vector<Cell>> path_by_task;
    std::map<int, std::vector<Cell>> visited_by_task;

    std::string section;
    int cur_task = -1;

    for (const std::string& line : split_lines(text)) {
        if (is_blank(line) || line[0] == '#') continue;

        if (line[0] == '@') {
            // 헤더 검증: @version 만 확인(불일치 시 거부).
            if (line.rfind("@version", 0) == 0) {
                std::istringstream is(line);
                std::string tag;
                int ver = 0;
                is >> tag >> ver;
                if (ver != SCENE_FORMAT_VERSION)
                    throw std::runtime_error("지원하지 않는 scene 버전: " + std::to_string(ver));
            }
            continue;
        }

        if (line[0] == '[') {
            std::vector<std::string> parts = split_tabs(line);
            const std::string& head = parts[0];                 // "[obstacles]"
            const size_t rb = head.find(']');
            section = head.substr(1, rb - 1);                   // "obstacles"
            std::map<std::string, std::string> attrs;
            for (size_t a = 1; a < parts.size(); ++a) {
                const size_t eq = parts[a].find('=');
                if (eq != std::string::npos)
                    attrs[parts[a].substr(0, eq)] = parts[a].substr(eq + 1);
            }
            if (section == "result" || section == "path" || section == "visited") {
                cur_task = parse_int(attrs["task"]);
                if (section == "result") result_kv[cur_task];           // 존재 표시(빈 맵).
                else if (section == "path") path_by_task[cur_task];     // 존재 표시(빈 목록).
                else visited_by_task[cur_task];
            }
            continue;
        }

        std::vector<std::string> cols = split_tabs(line);
        if (section == "grid") {
            if (cols[0] == "cell_mm") { doc.cell_mm = parse_double(cols[1]); has_cell = true; }
            else if (cols[0] == "origin")
                doc.origin = Vec3{parse_double(cols[1]), parse_double(cols[2]), parse_double(cols[3])};
            else if (cols[0] == "shape")
                doc.shape = Cell{parse_int(cols[1]), parse_int(cols[2]), parse_int(cols[3])};
        } else if (section == "params") {
            params_kv[cols[0]] = std::vector<std::string>(cols.begin() + 1, cols.end());
        } else if (section == "obstacles") {
            Obstacle o;
            o.min_xyz = Vec3{parse_double(cols[0]), parse_double(cols[1]), parse_double(cols[2])};
            o.max_xyz = Vec3{parse_double(cols[3]), parse_double(cols[4]), parse_double(cols[5])};
            o.ost_type = opt_in(cols[6]);
            o.name = opt_in(cols[7]);
            o.object_id = opt_in(cols[8]);
            o.ddworks_type = (cols.size() > 9) ? opt_in(cols[9]) : std::nullopt;
            doc.obstacles.push_back(std::move(o));
        } else if (section == "tasks") {
            RouteTask t;
            t.start_mm = Vec3{parse_double(cols[0]), parse_double(cols[1]), parse_double(cols[2])};
            t.end_mm = Vec3{parse_double(cols[3]), parse_double(cols[4]), parse_double(cols[5])};
            t.utility = opt_in(cols[6]);
            t.utility_group = opt_in(cols[7]);
            t.start_name = opt_in(cols[8]);
            t.end_name = opt_in(cols[9]);
            t.end_instance_guid = (cols.size() > 10) ? opt_in(cols[10]) : std::nullopt;
            doc.tasks.push_back(std::move(t));
        } else if (section == "result") {
            result_kv[cur_task][cols[0]] = (cols.size() > 1) ? cols[1] : "";
        } else if (section == "path") {
            path_by_task[cur_task].push_back(Cell{parse_int(cols[0]), parse_int(cols[1]), parse_int(cols[2])});
        } else if (section == "visited") {
            visited_by_task[cur_task].push_back(Cell{parse_int(cols[0]), parse_int(cols[1]), parse_int(cols[2])});
        }
    }

    // params 복원.
    auto pf = [&](const char* key, double dflt) {
        auto it = params_kv.find(key);
        return (it != params_kv.end() && !it->second.empty()) ? parse_double(it->second[0]) : dflt;
    };
    auto pi = [&](const char* key, int dflt) {
        auto it = params_kv.find(key);
        return (it != params_kv.end() && !it->second.empty()) ? parse_int(it->second[0]) : dflt;
    };
    doc.params.cell_mm = pf("cell_mm", has_cell ? doc.cell_mm : 50.0);
    doc.params.w_turn = pf("w_turn", 500.0);
    doc.params.w_clear = pf("w_clear", 10.0);
    doc.params.clearance_radius = pi("clearance_radius", 2);
    doc.params.clearance_connectivity = pi("clearance_connectivity", 6);
    doc.params.w_tier.clear();
    if (auto it = params_kv.find("w_tier"); it != params_kv.end()) {
        for (const std::string& tok : it->second) {
            const size_t colon = tok.find(':');
            if (colon == std::string::npos) continue;
            doc.params.w_tier[parse_int(tok.substr(0, colon))] = parse_double(tok.substr(colon + 1));
        }
    }

    // results 복원 (tasks 와 평행).
    doc.results.assign(doc.tasks.size(), std::nullopt);
    for (const auto& [idx, kv] : result_kv) {
        if (idx < 0 || idx >= static_cast<int>(doc.results.size())) continue;
        SceneResult r;
        auto get = [&](const char* k, const std::string& dflt) {
            auto i = kv.find(k);
            return i != kv.end() ? i->second : dflt;
        };
        r.success = get("success", "0") == "1";
        r.length_mm = parse_double(get("length_mm", "0.0"));
        r.cost_mm = parse_double(get("cost_mm", "0.0"));
        r.turns = parse_int(get("turns", "0"));
        r.expanded_nodes = parse_ll(get("expanded_nodes", "0"));
        r.elapsed_ms = parse_double(get("elapsed_ms", "0.0"));
        if (auto pit = path_by_task.find(idx); pit != path_by_task.end()) r.path = pit->second;
        if (auto vit = visited_by_task.find(idx); vit != visited_by_task.end()) r.visited = vit->second;
        doc.results[static_cast<size_t>(idx)] = std::move(r);
    }

    if (!has_cell) throw std::runtime_error("scene.txt 에 [grid] cell_mm 이 없습니다.");
    return doc;
}

SceneDoc read_scene(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("scene.txt 읽기 실패: " + path);
    std::ostringstream ss;
    ss << f.rdbuf();
    return loads_scene(ss.str());
}

// =============================================================================
// 점유맵 복원: grid 메타 + obstacles → Dense 점유맵 (퇴화 박스 스킵).
// =============================================================================
DenseOccupancy occupancy_from_doc(const SceneDoc& doc) {
    DenseOccupancy occ(doc.shape, doc.origin, doc.cell_mm);
    for (const Obstacle& o : doc.obstacles) {
        try {
            occ.add_box(AABB(o.min_xyz, o.max_xyz));
        } catch (const std::invalid_argument&) {
            continue;  // 두께 0(퇴화) 박스는 건너뛴다.
        }
    }
    return occ;
}

}  // namespace routing3d
