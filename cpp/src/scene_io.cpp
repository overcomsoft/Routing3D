// ???ÖÏ∂ú??Íµ¨ÌòÑ ??scene_io.hpp Ï∞∏Í≥†. Í∑úÍ≤© docs/spec/scene_format_spec.md (v1).
// Python ?àÌçº?∞Ïä§ routing3d_py/scene_io.py ?Ä Î∞îÏù¥???®ÏúÑÎ°??ôÏùº??Ï∂úÎ†•??Î™©ÌëúÎ°??úÎã§.
#include "routing3d/scene_io.hpp"

#include <charconv>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <cctype>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace routing3d {

namespace {

// None(????Îπ?Î¨∏Ïûê?¥Í≥º Íµ¨Î∂Ñ?òÎäî ?†ÌÅ∞(PostgreSQL COPY Í¥ÄÎ°Ä). C++ Î¨∏Ïûê?¥Î°ú??Î∞±Ïä¨?òÏãú+N.
const std::string NULL_TOKEN = "\\N";

// ?†ÌÉù Î¨∏Ïûê?????ÑÎìú Î¨∏Ïûê?? nullopt ??\N, Í∞??àÏúºÎ©?Í∑∏Î?Î°?Îπ?Î¨∏Ïûê?¥ÎèÑ Í∑∏Î?Î°?.
std::string opt_out(const std::optional<std::string>& s) {
    return s.has_value() ? *s : NULL_TOKEN;
}

// ?ÑÎìú Î¨∏Ïûê?????†ÌÉù Î¨∏Ïûê?? \N ??nullopt, Í∑?????Í∑∏Î?Î°?
std::optional<std::string> opt_in(const std::string& tok) {
    if (tok == NULL_TOKEN) return std::nullopt;
    return tok;
}

// Î°úÏ∫ò Î¨¥Í? double ?åÏã±(?åÏàò??'.'). repr Ï∂úÎ†•(Í≥†Ï†ï/ÏßÄ??Î™®Îëê)??Î∞õÎäî??
double parse_double(const std::string& s) {
    double v = 0.0;
    auto r = std::from_chars(s.data(), s.data() + s.size(), v);
    if (r.ec != std::errc())
        throw std::runtime_error("scene.txt: float ?åÏã± ?§Ìå®: '" + s + "'");
    return v;
}

long long parse_ll(const std::string& s) {
    long long v = 0;
    std::from_chars(s.data(), s.data() + s.size(), v);
    return v;
}

int parse_int(const std::string& s) { return static_cast<int>(parse_ll(s)); }

// ?çÏä§?∏Î? Ï§??®ÏúÑÎ°?Î∂ÑÎ¶¨(Í∞úÌñâ '\n' Í∏∞Ï?, Í∞?Ï§ÑÏùò ??'\r' ?úÍ±∞).
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

// TAB ?®Ïùº Î∂ÑÎ¶¨(Í≥µÎ∞± Î∂ÑÎ¶¨ Í∏àÏ? ???¥Î¶Ñ??Í≥µÎ∞± ?¨Ìï®). Îπ??ÑÎìú Î≥¥Ï°¥.
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

// Í≥µÎ∞±Îß??àÎäî Ï§ÑÏù∏ÏßÄ(Îπ?Ï§??§ÌÇµ??.
bool is_blank(const std::string& s) {
    for (char ch : s)
        if (ch != ' ' && ch != '\t' && ch != '\r' && ch != '\v' && ch != '\f') return false;
    return true;
}


struct JsonValue {
    enum Type { Null, Bool, Number, String, Array, Object } type = Null;
    bool b = false;
    double n = 0.0;
    std::string s;
    std::vector<JsonValue> a;
    std::map<std::string, JsonValue> o;
    const JsonValue& at(const std::string& key) const {
        auto it = o.find(key);
        if (it == o.end()) throw std::runtime_error("scene json: missing key: " + key);
        return it->second;
    }
};

class JsonParser {
public:
    explicit JsonParser(const std::string& text) : text_(text) {}
    JsonValue parse() {
        JsonValue v = parse_value();
        skip_ws();
        if (pos_ != text_.size()) throw std::runtime_error("scene json: trailing data");
        return v;
    }
private:
    void skip_ws() {
        while (pos_ < text_.size() && std::isspace(static_cast<unsigned char>(text_[pos_]))) ++pos_;
    }
    char peek() { skip_ws(); return pos_ < text_.size() ? text_[pos_] : '\0'; }
    char get() { if (pos_ >= text_.size()) throw std::runtime_error("scene json: unexpected eof"); return text_[pos_++]; }
    void expect(char ch) { skip_ws(); if (get() != ch) throw std::runtime_error("scene json: expected character"); }
    JsonValue parse_value() {
        skip_ws();
        char ch = peek();
        if (ch == '{') return parse_object();
        if (ch == '[') return parse_array();
        if (ch == '"') { JsonValue v; v.type = JsonValue::String; v.s = parse_string(); return v; }
        if (ch == 't' || ch == 'f') return parse_bool();
        if (ch == 'n') return parse_null();
        return parse_number();
    }
    JsonValue parse_null() {
        if (text_.compare(pos_, 4, "null") != 0) throw std::runtime_error("scene json: invalid null");
        pos_ += 4; return JsonValue{};
    }
    JsonValue parse_bool() {
        JsonValue v; v.type = JsonValue::Bool;
        if (text_.compare(pos_, 4, "true") == 0) { v.b = true; pos_ += 4; return v; }
        if (text_.compare(pos_, 5, "false") == 0) { v.b = false; pos_ += 5; return v; }
        throw std::runtime_error("scene json: invalid bool");
    }
    JsonValue parse_number() {
        skip_ws();
        size_t start = pos_;
        if (pos_ < text_.size() && text_[pos_] == '-') ++pos_;
        while (pos_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[pos_]))) ++pos_;
        if (pos_ < text_.size() && text_[pos_] == '.') { ++pos_; while (pos_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[pos_]))) ++pos_; }
        if (pos_ < text_.size() && (text_[pos_] == 'e' || text_[pos_] == 'E')) { ++pos_; if (pos_ < text_.size() && (text_[pos_] == '+' || text_[pos_] == '-')) ++pos_; while (pos_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[pos_]))) ++pos_; }
        JsonValue v; v.type = JsonValue::Number; v.n = parse_double(text_.substr(start, pos_ - start)); return v;
    }
    std::string parse_string() {
        expect('"');
        std::string out;
        while (true) {
            char ch = get();
            if (ch == '"') break;
            if (ch == '\\') {
                char esc = get();
                if (esc == '"' || esc == '\\' || esc == '/') out.push_back(esc);
                else if (esc == 'b') out.push_back('\b');
                else if (esc == 'f') out.push_back('\f');
                else if (esc == 'n') out.push_back('\n');
                else if (esc == 'r') out.push_back('\r');
                else if (esc == 't') out.push_back('\t');
                else throw std::runtime_error("scene json: unsupported string escape");
            } else out.push_back(ch);
        }
        return out;
    }
    JsonValue parse_array() {
        JsonValue v; v.type = JsonValue::Array; expect('[');
        if (peek() == ']') { get(); return v; }
        while (true) {
            v.a.push_back(parse_value());
            char ch = peek();
            if (ch == ']') { get(); break; }
            expect(',');
        }
        return v;
    }
    JsonValue parse_object() {
        JsonValue v; v.type = JsonValue::Object; expect('{');
        if (peek() == '}') { get(); return v; }
        while (true) {
            std::string key = parse_string();
            expect(':');
            v.o[key] = parse_value();
            char ch = peek();
            if (ch == '}') { get(); break; }
            expect(',');
        }
        return v;
    }
    const std::string& text_;
    size_t pos_ = 0;
};

std::string json_escape(const std::string& s) {
    std::string out = "\"";
    for (unsigned char ch : s) {
        if (ch == '"') out += "\\\"";
        else if (ch == '\\') out += "\\\\";
        else if (ch == '\n') out += "\\n";
        else if (ch == '\r') out += "\\r";
        else if (ch == '\t') out += "\\t";
        else if (ch < 0x20) out += " ";
        else out.push_back(static_cast<char>(ch));
    }
    out += "\"";
    return out;
}

std::string json_opt(const std::optional<std::string>& s) {
    return s.has_value() ? json_escape(*s) : std::string("null");
}

double jnum(const JsonValue& v) { if (v.type != JsonValue::Number) throw std::runtime_error("scene json: expected number"); return v.n; }
int jint(const JsonValue& v) { return static_cast<int>(jnum(v)); }
long long jll(const JsonValue& v) { return static_cast<long long>(jnum(v)); }
bool jbool(const JsonValue& v) { if (v.type != JsonValue::Bool) throw std::runtime_error("scene json: expected bool"); return v.b; }
std::optional<std::string> jopt(const JsonValue& v) { if (v.type == JsonValue::Null) return std::nullopt; if (v.type != JsonValue::String) throw std::runtime_error("scene json: expected string/null"); return v.s; }
Vec3 jvec3(const JsonValue& v) { return Vec3{jnum(v.a.at(0)), jnum(v.a.at(1)), jnum(v.a.at(2))}; }
Cell jcell(const JsonValue& v) { return Cell{jint(v.a.at(0)), jint(v.a.at(1)), jint(v.a.at(2))}; }
}  // namespace

// =============================================================================
// ?§Ïàò ?úÍ∏∞: Python repr(float) ?Ä ?ôÏùº??ÏµúÎã® ?ïÎ≥µ ?úÍ∏∞ (Í≥ÑÏïΩ F4)
//   1) std::to_chars(scientific) Î°?ÏµúÎã® ?†Ìö®?´Ïûê + ÏßÄ?òÎ? ?ªÍ≥†(Ry≈´ = dtoa mode0 ?ôÏùº),
//   2) Python Í∑úÏπô(decpt<=-4 ?êÎäî >16 ?¥Î©¥ ÏßÄ?òÌëúÍ∏??ºÎ°ú ?¨Ìè¨Îß∑Ìïú??
//   decpt = ?åÏàò???ºÏ™Ω ?êÎ¶ø??= (Í≥ºÌïô?úÍ∏∞ ÏßÄ??E) + 1.
// =============================================================================
std::string format_repr_double(double x) {
    if (std::isnan(x)) return "nan";
    if (std::isinf(x)) return x < 0 ? "-inf" : "inf";

    char buf[64];
    auto res = std::to_chars(buf, buf + sizeof(buf), x, std::chars_format::scientific);
    std::string sci(buf, res.ptr);  // ?? "-1.2345e+03", "5e+01", "0e+00"

    size_t i = 0;
    bool neg = false;
    if (sci[i] == '-') { neg = true; ++i; }

    std::string digits;
    digits.push_back(sci[i++]);                 // Ï≤??†Ìö®?´Ïûê.
    if (i < sci.size() && sci[i] == '.') {
        ++i;
        while (i < sci.size() && sci[i] != 'e') digits.push_back(sci[i++]);
    }
    ++i;                                          // 'e' ?§ÌÇµ.
    std::string exp_str = sci.substr(i);          // ?? "+03", "-05".
    if (!exp_str.empty() && exp_str[0] == '+')    // from_chars(int)??'+'Î•?Í±∞Î? ???úÍ±∞.
        exp_str.erase(0, 1);
    const int E = parse_int(exp_str);             // Í≥ºÌïô?úÍ∏∞ ÏßÄ??

    const int ndigits = static_cast<int>(digits.size());
    const int decpt = E + 1;

    std::string body;
    if (decpt <= -4 || decpt > 16) {
        // ÏßÄ?òÌëúÍ∏? d[.ddd]e¬±XX (ÏßÄ??ÏµúÏÜå 2?êÎ¶¨, Î∂Ä????ÉÅ).
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
// ?∞Í∏∞ (SceneDoc ??scene.txt Î¨∏Ïûê??. Python dumps_scene Í≥??ôÏùº Î∞îÏù¥??
// =============================================================================
std::string dumps_scene_legacy(const SceneDoc& doc) {
    std::ostringstream out;
    const auto ff = [](double v) { return format_repr_double(v); };

    out << "# Routing3D scene file \xe2\x80\x94 units: mm\n";  // ??= U+2014 (UTF-8).
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
    out << "w_corridor\t" << ff(p.w_corridor) << "\n";
    out << "w_heur\t" << ff(p.w_heur) << "\n";
    out << "w_heur_near\t" << ff(p.w_heur_near) << "\n";
    out << "clearance_radius\t" << p.clearance_radius << "\n";
    out << "clearance_connectivity\t" << p.clearance_connectivity << "\n";
    out << "corridor_radius\t" << p.corridor_radius << "\n";
    out << "w_tier";
    for (const auto& [z, v] : p.w_tier)
        out << "\t" << z << ":" << ff(v);
    out << "\n";
    out << "rack_levels";
    for (int z : p.rack_levels) out << "\t" << z;
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

    if (!doc.passthrough.empty()) {
        out << "[passthrough]\tcount=" << doc.passthrough.size() << "\n";
        out << "# minx\tminy\tminz\tmaxx\tmaxy\tmaxz\tost_type\tname\tobject_id\tddworks_type\n";
        for (const Obstacle& o : doc.passthrough) {
            out << ff(o.min_xyz.x) << "\t" << ff(o.min_xyz.y) << "\t" << ff(o.min_xyz.z) << "\t"
                << ff(o.max_xyz.x) << "\t" << ff(o.max_xyz.y) << "\t" << ff(o.max_xyz.z) << "\t"
                << opt_out(o.ost_type) << "\t" << opt_out(o.name) << "\t"
                << opt_out(o.object_id) << "\t" << opt_out(o.ddworks_type) << "\n";
        }
        out << "\n";
    }

    // ---- [tasks]
    out << "[tasks]\tcount=" << doc.tasks.size() << "\n";
    out << "# sx\tsy\tsz\tgx\tgy\tgz\tutility\tutility_group\tstart_name\tend_name\tend_instance_guid\tdiameter_mm\tgoal_dir\n";
    for (const RouteTask& t : doc.tasks) {
        out << ff(t.start_mm.x) << "\t" << ff(t.start_mm.y) << "\t" << ff(t.start_mm.z) << "\t"
            << ff(t.end_mm.x) << "\t" << ff(t.end_mm.y) << "\t" << ff(t.end_mm.z) << "\t"
            << opt_out(t.utility) << "\t" << opt_out(t.utility_group) << "\t"
            << opt_out(t.start_name) << "\t" << opt_out(t.end_name) << "\t"
            << opt_out(t.end_instance_guid) << "\t" << ff(t.diameter_mm) << "\t"
            << t.goal_dir << "\n";
    }
    out << "\n";

    // ---- [results] (?†ÌÉù; None ?ÑÎãå Í≤∞Í≥º ?òÎßå??
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


std::string dumps_scene(const SceneDoc& doc) {
    std::ostringstream out;
    const auto ff = [](double v) { return format_repr_double(v); };
    auto vec3 = [&](const Vec3& v) { out << "[" << ff(v.x) << "," << ff(v.y) << "," << ff(v.z) << "]"; };
    auto cell = [&](const Cell& c) { out << "[" << c.i << "," << c.j << "," << c.k << "]"; };
    auto obstacle = [&](const Obstacle& o) {
        out << "{\"min\":"; vec3(o.min_xyz); out << ",\"max\":"; vec3(o.max_xyz);
        out << ",\"ost_type\":" << json_opt(o.ost_type)
            << ",\"name\":" << json_opt(o.name)
            << ",\"object_id\":" << json_opt(o.object_id)
            << ",\"ddworks_type\":" << json_opt(o.ddworks_type) << "}";
    };
    out << "{\n";
    out << "  \"format\": \"" << SCENE_FORMAT_TAG << "\",\n";
    out << "  \"version\": " << SCENE_FORMAT_VERSION << ",\n";
    out << "  \"grid\": {\"cell_mm\": " << ff(doc.cell_mm) << ", \"origin\": "; vec3(doc.origin); out << ", \"shape\": "; cell(doc.shape); out << "},\n";
    const RouteParams& p = doc.params;
    out << "  \"params\": {\"cell_mm\": " << ff(p.cell_mm) << ", \"w_turn\": " << ff(p.w_turn)
        << ", \"w_clear\": " << ff(p.w_clear) << ", \"w_corridor\": " << ff(p.w_corridor)
        << ", \"w_heur\": " << ff(p.w_heur) << ", \"w_heur_near\": " << ff(p.w_heur_near)
        << ", \"clearance_radius\": " << p.clearance_radius << ", \"clearance_connectivity\": " << p.clearance_connectivity
        << ", \"corridor_radius\": " << p.corridor_radius << ", \"w_tier\": [";
    bool first = true;
    for (const auto& [z, w] : p.w_tier) { if (!first) out << ","; first = false; out << "{\"z\":" << z << ",\"weight\":" << ff(w) << "}"; }
    out << "], \"rack_levels\": [";
    for (size_t i = 0; i < p.rack_levels.size(); ++i) { if (i) out << ","; out << p.rack_levels[i]; }
    out << "]},\n";
    out << "  \"obstacles\": [";
    for (size_t i = 0; i < doc.obstacles.size(); ++i) { if (i) out << ","; obstacle(doc.obstacles[i]); }
    out << "],\n  \"passthrough\": [";
    for (size_t i = 0; i < doc.passthrough.size(); ++i) { if (i) out << ","; obstacle(doc.passthrough[i]); }
    out << "],\n  \"tasks\": [";
    for (size_t i = 0; i < doc.tasks.size(); ++i) {
        const RouteTask& t = doc.tasks[i]; if (i) out << ",";
        out << "{\"start\":"; vec3(t.start_mm); out << ",\"end\":"; vec3(t.end_mm);
        out << ",\"utility\":" << json_opt(t.utility)
            << ",\"utility_group\":" << json_opt(t.utility_group)
            << ",\"start_name\":" << json_opt(t.start_name)
            << ",\"end_name\":" << json_opt(t.end_name)
            << ",\"end_instance_guid\":" << json_opt(t.end_instance_guid)
            << ",\"diameter_mm\":" << ff(t.diameter_mm)
            << ",\"goal_dir\":" << t.goal_dir << "}";
    }
    out << "],\n  \"results\": [";
    for (size_t i = 0; i < doc.tasks.size(); ++i) {
        if (i) out << ",";
        if (i >= doc.results.size() || !doc.results[i].has_value()) { out << "null"; continue; }
        const SceneResult& r = *doc.results[i];
        out << "{\"success\":" << (r.success ? "true" : "false")
            << ",\"length_mm\":" << ff(r.length_mm) << ",\"cost_mm\":" << ff(r.cost_mm)
            << ",\"turns\":" << r.turns << ",\"expanded_nodes\":" << r.expanded_nodes
            << ",\"elapsed_ms\":" << ff(r.elapsed_ms) << ",\"fail\":" << r.fail << ",\"path\":";
        if (r.path.has_value()) { out << "["; for (size_t j = 0; j < r.path->size(); ++j) { if (j) out << ","; cell((*r.path)[j]); } out << "]"; } else out << "null";
        out << ",\"visited\":";
        if (r.visited.has_value()) { out << "["; for (size_t j = 0; j < r.visited->size(); ++j) { if (j) out << ","; cell((*r.visited)[j]); } out << "]"; } else out << "null";
        out << "}";
    }
    out << "]\n}\n";
    return out.str();
}
void write_scene(const std::string& path, const SceneDoc& doc) {
    std::ofstream f(path, std::ios::binary);  // binary ??\n Í∑∏Î?Î°??àÎèÑ??CRLF Î≥Ä??Î∞©Ï?).
    if (!f) throw std::runtime_error("scene json write failed: " + path);
    const std::string text = dumps_scene(doc);
    f.write(text.data(), static_cast<std::streamsize>(text.size()));
}

// =============================================================================
// ?ΩÍ∏∞ (scene.txt Î¨∏Ïûê????SceneDoc). ?®Ïàú ?ÅÌÉúÍ∏∞Í≥Ñ ?åÏÑú(Í∑úÍ≤© ¬ß7).
// =============================================================================
SceneDoc loads_scene_legacy(const std::string& text) {
    SceneDoc doc;
    bool has_cell = false;

    std::map<std::string, std::vector<std::string>> params_kv;  // params ?πÏÖò ?§‚ÜíÍ∞íÎì§.
    std::map<int, std::map<std::string, std::string>> result_kv;
    std::map<int, std::vector<Cell>> path_by_task;
    std::map<int, std::vector<Cell>> visited_by_task;

    std::string section;
    int cur_task = -1;

    for (const std::string& line : split_lines(text)) {
        if (is_blank(line) || line[0] == '#') continue;

        if (line[0] == '@') {
            // ?§Îçî Í≤ÄÏ¶? @version Îß??ïÏù∏(Î∂àÏùºÏπ???Í±∞Î?).
            if (line.rfind("@version", 0) == 0) {
                std::istringstream is(line);
                std::string tag;
                int ver = 0;
                is >> tag >> ver;
                if (ver < 1 || ver > SCENE_FORMAT_VERSION)
                    throw std::runtime_error("unsupported scene version: " + std::to_string(ver));
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
                if (section == "result") result_kv[cur_task];           // Ï°¥Ïû¨ ?úÏãú(Îπ?Îß?.
                else if (section == "path") path_by_task[cur_task];     // Ï°¥Ïû¨ ?úÏãú(Îπ?Î™©Î°ù).
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
        } else if (section == "obstacles" || section == "passthrough") {
            Obstacle o;
            o.min_xyz = Vec3{parse_double(cols[0]), parse_double(cols[1]), parse_double(cols[2])};
            o.max_xyz = Vec3{parse_double(cols[3]), parse_double(cols[4]), parse_double(cols[5])};
            o.ost_type = opt_in(cols[6]);
            o.name = opt_in(cols[7]);
            o.object_id = opt_in(cols[8]);
            o.ddworks_type = (cols.size() > 9) ? opt_in(cols[9]) : std::nullopt;
            if (section == "passthrough") doc.passthrough.push_back(std::move(o));
            else doc.obstacles.push_back(std::move(o));
        } else if (section == "tasks") {
            RouteTask t;
            t.start_mm = Vec3{parse_double(cols[0]), parse_double(cols[1]), parse_double(cols[2])};
            t.end_mm = Vec3{parse_double(cols[3]), parse_double(cols[4]), parse_double(cols[5])};
            t.utility = opt_in(cols[6]);
            t.utility_group = opt_in(cols[7]);
            t.start_name = opt_in(cols[8]);
            t.end_name = opt_in(cols[9]);
            t.end_instance_guid = (cols.size() > 10) ? opt_in(cols[10]) : std::nullopt;
            if (cols.size() > 11) t.diameter_mm = parse_double(cols[11]);
            if (cols.size() > 12) t.goal_dir = parse_int(cols[12]);
            doc.tasks.push_back(std::move(t));
        } else if (section == "result") {
            result_kv[cur_task][cols[0]] = (cols.size() > 1) ? cols[1] : "";
        } else if (section == "path") {
            path_by_task[cur_task].push_back(Cell{parse_int(cols[0]), parse_int(cols[1]), parse_int(cols[2])});
        } else if (section == "visited") {
            visited_by_task[cur_task].push_back(Cell{parse_int(cols[0]), parse_int(cols[1]), parse_int(cols[2])});
        }
    }

    // params Î≥µÏõê.
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
    doc.params.w_corridor = pf("w_corridor", 0.0);
    doc.params.w_heur = pf("w_heur", 1.0);
    doc.params.w_heur_near = pf("w_heur_near", 0.0);
    doc.params.clearance_radius = pi("clearance_radius", 2);
    doc.params.clearance_connectivity = pi("clearance_connectivity", 6);
    doc.params.corridor_radius = pi("corridor_radius", 1);
    doc.params.w_tier.clear();
    if (auto it = params_kv.find("w_tier"); it != params_kv.end()) {
        for (const std::string& tok : it->second) {
            const size_t colon = tok.find(':');
            if (colon == std::string::npos) continue;
            doc.params.w_tier[parse_int(tok.substr(0, colon))] = parse_double(tok.substr(colon + 1));
        }
    }

    // results Î≥µÏõê (tasks ?Ä ?âÌñâ).
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

    if (!has_cell) throw std::runtime_error("scene.txt ??[grid] cell_mm ???ÜÏäµ?àÎã§.");
    return doc;
}


SceneDoc loads_scene(const std::string& text) {
    size_t p0 = text.find_first_not_of(" \t\r\n");
    if (p0 == std::string::npos) throw std::runtime_error("scene json: empty input");
    if (text[p0] != '{') return loads_scene_legacy(text);

    JsonValue root = JsonParser(text).parse();
    SceneDoc doc;
    int version = jint(root.at("version"));
    if (version < 3 || version > SCENE_FORMAT_VERSION)
        throw std::runtime_error("unsupported scene json version: " + std::to_string(version));

    const JsonValue& grid = root.at("grid");
    doc.cell_mm = jnum(grid.at("cell_mm"));
    doc.origin = jvec3(grid.at("origin"));
    doc.shape = jcell(grid.at("shape"));

    const JsonValue& params = root.at("params");
    doc.params.cell_mm = jnum(params.at("cell_mm"));
    doc.params.w_turn = jnum(params.at("w_turn"));
    doc.params.w_clear = jnum(params.at("w_clear"));
    doc.params.w_corridor = jnum(params.at("w_corridor"));
    doc.params.w_heur = jnum(params.at("w_heur"));
    doc.params.w_heur_near = jnum(params.at("w_heur_near"));
    doc.params.clearance_radius = jint(params.at("clearance_radius"));
    doc.params.clearance_connectivity = jint(params.at("clearance_connectivity"));
    doc.params.corridor_radius = jint(params.at("corridor_radius"));
    doc.params.w_tier.clear();
    for (const JsonValue& item : params.at("w_tier").a) doc.params.w_tier[jint(item.at("z"))] = jnum(item.at("weight"));
    doc.params.rack_levels.clear();
    for (const JsonValue& item : params.at("rack_levels").a) doc.params.rack_levels.push_back(jint(item));

    auto read_obstacle = [](const JsonValue& v) {
        Obstacle o;
        o.min_xyz = jvec3(v.at("min")); o.max_xyz = jvec3(v.at("max"));
        o.ost_type = jopt(v.at("ost_type")); o.name = jopt(v.at("name"));
        o.object_id = jopt(v.at("object_id")); o.ddworks_type = jopt(v.at("ddworks_type"));
        return o;
    };
    for (const JsonValue& v : root.at("obstacles").a) doc.obstacles.push_back(read_obstacle(v));
    for (const JsonValue& v : root.at("passthrough").a) doc.passthrough.push_back(read_obstacle(v));

    for (const JsonValue& v : root.at("tasks").a) {
        RouteTask t;
        t.start_mm = jvec3(v.at("start")); t.end_mm = jvec3(v.at("end"));
        t.utility = jopt(v.at("utility")); t.utility_group = jopt(v.at("utility_group"));
        t.start_name = jopt(v.at("start_name")); t.end_name = jopt(v.at("end_name"));
        t.end_instance_guid = jopt(v.at("end_instance_guid"));
        t.diameter_mm = jnum(v.at("diameter_mm")); t.goal_dir = jint(v.at("goal_dir"));
        doc.tasks.push_back(std::move(t));
    }
    doc.results.assign(doc.tasks.size(), std::nullopt);
    const JsonValue& results = root.at("results");
    for (size_t i = 0; i < results.a.size() && i < doc.results.size(); ++i) {
        const JsonValue& v = results.a[i];
        if (v.type == JsonValue::Null) continue;
        SceneResult r;
        r.success = jbool(v.at("success")); r.length_mm = jnum(v.at("length_mm")); r.cost_mm = jnum(v.at("cost_mm"));
        r.turns = jint(v.at("turns")); r.expanded_nodes = jll(v.at("expanded_nodes")); r.elapsed_ms = jnum(v.at("elapsed_ms"));
        r.fail = jint(v.at("fail"));
        if (v.at("path").type != JsonValue::Null) { std::vector<Cell> path; for (const JsonValue& c : v.at("path").a) path.push_back(jcell(c)); r.path = path; }
        if (v.at("visited").type != JsonValue::Null) { std::vector<Cell> visited; for (const JsonValue& c : v.at("visited").a) visited.push_back(jcell(c)); r.visited = visited; }
        doc.results[i] = std::move(r);
    }
    return doc;
}
SceneDoc read_scene(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("scene json read failed: " + path);
    std::ostringstream ss;
    ss << f.rdbuf();
    return loads_scene(ss.str());
}

// =============================================================================
// ?êÏú†Îß?Î≥µÏõê: grid Î©îÌ? + obstacles ??Dense ?êÏú†Îß?(?¥Ìôî Î∞ïÏä§ ?§ÌÇµ).
// =============================================================================
DenseOccupancy occupancy_from_doc(const SceneDoc& doc) {
    DenseOccupancy occ(doc.shape, doc.origin, doc.cell_mm);
    for (const Obstacle& o : doc.obstacles) {
        try {
            occ.add_box(AABB(o.min_xyz, o.max_xyz));
        } catch (const std::invalid_argument&) {
            continue;  // ?êÍªò 0(?¥Ìôî) Î∞ïÏä§??Í±¥ÎÑà?¥Îã§.
        }
    }
    return occ;
}

// ?µÍ≥º Í∞ùÏ≤¥(doc.passthrough)ÎßåÏúºÎ°?Dense ?êÏú†Îß??ùÏÑ± ??Í∞Ä?úÌôî ?ÑÏö©.
DenseOccupancy occupancy_from_passthrough(const SceneDoc& doc) {
    DenseOccupancy occ(doc.shape, doc.origin, doc.cell_mm);
    for (const Obstacle& o : doc.passthrough) {
        try {
            occ.add_box(AABB(o.min_xyz, o.max_xyz));
        } catch (const std::invalid_argument&) {
            continue;  // ?êÍªò 0(?¥Ìôî) Î∞ïÏä§??Í±¥ÎÑà?¥Îã§.
        }
    }
    return occ;
}

}  // namespace routing3d
