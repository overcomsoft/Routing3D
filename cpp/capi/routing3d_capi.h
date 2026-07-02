// Routing3D ?쇱씠釉뚮윭由?C ABI ?ㅻ뜑 (routing3d_capi) ??Phase 3 (C#/HelixToolkit ?명꽣濡?
// =============================================================================
// [???뚯씪???섎뒗 ??
//   C++ ?쇱슦???붿쭊(occupancy/astar/cost/multi_route/scene_io)??C ABI(extern "C")濡?
//   ?몄텧??C#(P/Invoke)쨌?뚯씠??ctypes) ???대뼡 ?몄뒪?몃뱺 ?꾨줈?몄뒪濡??몄텧?섍쾶 ?쒕떎.
//   ?ㅺ퀎: docs/csharp_helix_interop_design.md.
//
// [ABI ?덉쟾 洹쒖튃]
//   1) C++ ?덉쇅??寃쎄퀎 諛뽰쑝濡??섍?吏 ?딅뒗????紐⑤뱺 諛섑솚? R3dStatus(?먮뒗 ?뺤닔/0)濡?蹂닿퀬.
//   2) STL/C++ 媛앹껜瑜??몄텧?섏? ?딅뒗????遺덊닾紐??몃뱾(R3dEngine*) + POD 援ъ“泥?+ 怨좎젙 諛곗뿴.
//   3) ?몄텧 洹쒖빟 cdecl. 援ъ“泥대뒗 blittable(怨좎젙 ?덉씠?꾩썐).
//   4) 肄쒕윭 ?좊떦 臾몄옄?댁? r3d_free_string ?쇰줈 ?댁젣. 寃쎈줈 諛곗뿴? 肄쒕윭 ?좊떦(2?④퀎).
//   5) 臾몄옄?댁? UTF-8(寃쎈줈 ?대쫫) ???몄뒪?몃뒗 UTF-8 濡?留덉꺃留곹븳??
//
// [鍮뚮뱶]  (?꾨줈?앺듃 猷⑦듃?먯꽌; 肄붿뼱? 留곹겕 ???몃? ?섏〈???녿뒗 ?⑥씪 DLL)
//   cmake -S cpp -B cpp/build -G "Visual Studio 17 2022" -A x64
//   cmake --build cpp/build --config Release --target routing3d_capi
//   # 異쒕젰臾? cpp/build/Release/routing3d_capi.dll (+ .lib import)
//
// [寃利?
//   ctest --test-dir cpp/build -C Release -R capi --output-on-failure
// =============================================================================
#ifndef ROUTING3D_CAPI_H
#define ROUTING3D_CAPI_H

#include <stdint.h>

#if defined(_WIN32)
#  if defined(ROUTING3D_CAPI_EXPORTS)
#    define R3D_API __declspec(dllexport)
#  else
#    define R3D_API __declspec(dllimport)
#  endif
#else
#  define R3D_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

// 諛섑솚 ?곹깭 肄붾뱶.
typedef enum {
    R3D_OK = 0,
    R3D_ERR_ARG = 1,      // ?섎せ???몄옄(null ?ъ씤?걔룹쓬?샕? 踰붿쐞)
    R3D_ERR_PARSE = 2,    // scene ?띿뒪???뚯떛 ?ㅽ뙣
    R3D_ERR_RUNTIME = 3,  // ?ㅽ뻾 以??덉쇅
    R3D_ERR_RANGE = 4     // ?몃뜳??醫뚰몴 踰붿쐞 ?ㅻ쪟 (pack20 ?쒓퀎 珥덇낵 ??
} R3dStatus;

// ?뺤쟻 踰꾩쟾 臾몄옄???댁젣 遺덊븘??.
R3D_API const char* r3d_version(void);
// 肄쒕윭 ?좊떦 臾몄옄???댁젣(r3d_route_scene_text / r3d_dump_scene_text 異쒕젰).
R3D_API void r3d_free_string(char* s);

// ---------------------------------------------------------------- Level 1: 臾몄옄??ABI
// ?낅젰 scene ?띿뒪??UTF-8) 瑜??쇱슦?낇븯怨?寃곌낵 scene ?띿뒪??UTF-8) 諛섑솚. out_scene_text ???댁젣 ?꾩슂.
//   mode     : "multi"(?쒖감, 異⑸룎?놁쓬) | "single"(?묒뾽蹂??낅┰).
//   priority : "longest"|"shortest"|"utility"|"original" (mode=multi ?꾩슜). 湲곕낯="longest".
R3D_API R3dStatus r3d_route_scene_text(const char* scene_text, const char* mode,
                                       const char* priority, char** out_scene_text);

// ---------------------------------------------------------------- Level 2: ?몃뱾 ABI
typedef struct R3dEngine R3dEngine;  // 遺덊닾紐??몃뱾.

// blittable POD (C# StructLayout.Sequential ? 1:1).
// [?낅젰 寃利???r3d_set_grid]
//   cell_mm > 0, nx/ny/nz > 0 : R3D_ERR_ARG
//   nx/ny/nz > 1,048,575 (pack20 20鍮꾪듃 ?곹븳) : R3D_ERR_RANGE
//   ??corridor.hpp pack20 ? 異뺣떦 20鍮꾪듃(理쒕? 2^20-1 = 1,048,575 ?).
//     8,000m / 50mm = 160,000 ?쇰줈 ?ㅼ젣 ?뚮옖??洹쒕え?먯꽑 異⑸텇?섎굹,
//     4mm ?댄븯 珥덈??멸꺽?먯뿉?쒕뒗 珥덇낵 ?꾪뿕 ??利됱떆 R3D_ERR_RANGE 濡?李⑤떒.
typedef struct {
    double cell_mm;
    double ox, oy, oz;  // origin (mm)
    int32_t nx, ny, nz;  // shape
} R3dGrid;

// [?낅젰 寃利???r3d_set_params]
//   cell_mm > 0 : R3D_ERR_ARG
//   w_turn / w_clear / w_corridor / w_heur / w_heur_near < 0 : R3D_ERR_ARG
//   clearance_radius < 0 : R3D_ERR_ARG
//   clearance_connectivity ??{0, 6, 26} : R3D_ERR_ARG  (0 ? 湲곕낯媛믪쑝濡?6 ?쇰줈 泥섎━)
typedef struct {
    double cell_mm, w_turn, w_clear;
    double w_corridor;               // ?뚮옉 諛붿씠?댁뒪 媛以묒튂(mm). 0=鍮꾪솢??湲곗〈 ?숈옉). >0=湲곗〈?ㅺ퀎 ?뚮옉 踰덈뱾留?
    double w_heur;                   // ?대━?ㅽ떛 媛以묒튂(weighted A*). 0/1=?쒖? A*. >1=紐⑺몴 吏???먯깋, ?쒓컙 鍮꾩턀??
    double w_heur_near;              // ?곸쓳 媛以?紐⑺몴洹쇱쿂 ?섎졃). (0,w_heur] 踰붿쐞 ?섎졃 媛以? 0=怨좎젙 w_heur(遺덈?).
    int32_t clearance_radius, clearance_connectivity;  // connectivity: 0(湲곕낯=6) | 6 | 26
    int32_t corridor_radius;         // ?뚮옉 ?쎌갹 諛섍꼍(?). 湲곕낯 1.
    int32_t rack_level_count;        // rack_levels ?ъ슜 媛쒖닔(0~8).
    int32_t rack_levels[8];          // ?좏샇 ??z-? ?몃뜳?? 理쒕? 8. ?대떦 ?덈꺼? ?뚮옉 硫댁젣.
} R3dParams;

typedef struct {
    int64_t large_grid_threshold;   // 0=default 5,000,000 cells.
    int64_t max_expansions;         // 0=env R3D_MAX_EXP/default.
    int64_t fallback_expansions;    // 0=max_expansions/env R3D_FALLBACK_EXP.
    int32_t hier_factor;            // 0=default 8.
    int32_t hier_radius;            // 0=default 2.
    int64_t hier_probe;             // 0=default 300,000.
    int32_t ripup_enabled;          // -1=env/default, 0=off, 1=on.
    int64_t cbs_expansions;         // 0=default/env R3D_CBS_EXP. CBS-lite per-route cap.
} R3dRuntimeOptions;

typedef struct {
    int32_t enabled;                // 0=off, nonzero=on.
    int32_t level;                  // 0=summary, 1=standard, 2=detail.
    int32_t sample_every;           // Expand/progress event interval. 0=default 1000.
    int32_t include_occupancy;      // Write occupancy summary/header.
    int32_t include_rejects;        // Reserved for detailed A* reject events.
    int32_t include_postprocess;    // Write unkink/min-straight postprocess events.
    int32_t max_events_per_task;    // 0=default 20000.
} R3dTraceOptions;

typedef struct {
    int32_t success;        // 1/0
    double length_mm;       // 湲고븯 湲몄씠
    double cost_mm;         // total cost including penalties
    int32_t turns;
    int64_t expanded_nodes;
    double elapsed_ms;
    int32_t path_len;       // 寃쎈줈 ? ????r3d_copy_path 踰꾪띁 ?ш린 ?곗텧??
    int32_t visited_len;    // 諛⑸Ц(?먯깋) ? ????r3d_copy_visited 踰꾪띁 ?ш린 ?곗텧?? 鍮꾪솢?깆씠硫?0.
    // ?ㅽ뙣 ?댁쑀(A1) ??success=0 ???뚮쭔 ?좏슚. 援ъ“泥??앹뿉 異붽?(湲곗〈 ?몄텧 ?덉쟾).
    // 0=None쨌1=StartBlocked쨌2=GoalBlocked쨌3=CorridorMiss쨌4=ExpansionLimit쨌5=GoalDirBlocked쨌6=NoPath.
    int32_t fail_reason;
} R3dResult;

R3D_API R3dEngine* r3d_create(void);
R3D_API void r3d_destroy(R3dEngine* e);
R3D_API R3dStatus r3d_load_scene_text(R3dEngine* e, const char* scene_text);

R3D_API R3dStatus r3d_set_grid(R3dEngine* e, const R3dGrid* g);
R3D_API R3dStatus r3d_set_params(R3dEngine* e, const R3dParams* p);
R3D_API R3dStatus r3d_set_runtime_options(R3dEngine* e, const R3dRuntimeOptions* opt);
R3D_API R3dStatus r3d_set_trace_options(R3dEngine* e, const R3dTraceOptions* opt);
R3D_API R3dStatus r3d_set_trace_file(R3dEngine* e, const char* path_utf8);
R3D_API R3dStatus r3d_flush_trace(R3dEngine* e);
// Return a UTF-8 JSON diagnostic report for build flags, scene counts, and effective handle options.
// The caller owns *out_json and must release it with r3d_free_string().
R3D_API R3dStatus r3d_get_runtime_report(const R3dEngine* e, char** out_json);
R3D_API R3dStatus r3d_add_obstacle(R3dEngine* e, double minx, double miny, double minz,
                                   double maxx, double maxy, double maxz);
// ?듦낵(pass-through) 媛앹껜 異붽?: ?먯쑀留?媛?쒗솕?? 寃쎈줈?먯깋 異⑸룎 ??곸씠 ?꾨떂.
R3D_API R3dStatus r3d_add_passthrough(R3dEngine* e, double minx, double miny, double minz,
                                      double maxx, double maxy, double maxz);
// ?묒뾽 異붽? ??task index(>=0) 諛섑솚, ?ㅽ뙣 ???뚯닔. utility/utility_group ? 鍮꾩슜 遺꾨쪟?먮쭔 ?ъ슜.
R3D_API int32_t r3d_add_task(R3dEngine* e, double sx, double sy, double sz,
                             double gx, double gy, double gz,
                             const char* utility, const char* utility_group);
// ?묒뾽 醫낅떒??媛깆떊(?명꽣?숉떚釉??ъ쭛).
R3D_API R3dStatus r3d_set_task_endpoints(R3dEngine* e, int32_t task,
                                         double sx, double sy, double sz,
                                         double gx, double gy, double gz);
// ?묒뾽 愿寃?mm) ?ㅼ젙 ???곗꽑?쒖쐞 "diameter"/"utility" ?뺣젹?먯꽌 '援듭? 諛곌? 癒쇱?' ?④낵瑜??몃떎.
// 誘몄꽕???먮뒗 0)?대㈃ 愿寃?臾댁떆(湲곗〈 嫄곕━ ?뺣젹怨??숈씪). route_multi/route_corridor_multi/route_ripup 怨듯넻.
R3D_API R3dStatus r3d_set_task_diameter(R3dEngine* e, int32_t task, double diameter_mm);
// ?묒뾽 紐⑺몴 吏꾩엯異??쒖빟(axis = NEIGHBORS_6 ?몃뜳??0..5 = +x,-x,+y,-y,+z,-z).
// A* 媛 紐⑺몴(end_mm)??洹?諛⑺뼢?쇰줈 吏꾩엯?댁빞留??꾨떖 ?몄젙. ?뺥듃 醫낅떒 ?ㅽ뀅 由щ뱶??異뺤쓣 二쇰㈃ 諛곌????쇱쭅??吏꾩엯.
// 留됲엳硫?臾댁젣??1???대갚(?곌껐 ?곗꽑). axis 媛 [0,5] 諛??먮뒗 誘몄꽕?뺤씠硫?-1(臾댁젣??, 湲곗〈 ?숈옉쨌怨⑤뱺 遺덈?.
R3D_API R3dStatus r3d_set_task_goal_dir(R3dEngine* e, int32_t task, int32_t axis);

// ?쇱슦??
R3D_API R3dStatus r3d_route_multi(R3dEngine* e, const char* priority);  // ?꾩껜 ?쒖감(異⑸룎?놁쓬)
R3D_API R3dStatus r3d_route_task(R3dEngine* e, int32_t task, R3dResult* out);  // ?⑥씪(?μ븷臾??꾩슜)
R3D_API R3dStatus r3d_route_task_anytime(R3dEngine* e, int32_t task,
                                          double initial_weight, double final_weight,
                                          double weight_step, double time_budget_ms,
                                          int64_t max_expansions, int32_t goal_dir,
                                          R3dResult* out, int32_t* out_iterations,
                                          int32_t* out_improvements);
// ?숈뒿???뚮옉 ?(ijk ?쇱쨷媛?諛곗뿴, 湲몄씠 n)???ㅼ젙?쒕떎(L2b ?뚰봽??諛붿씠?댁뒪). w_corridor>0(set_params)?대㈃
// route_multi 媛 ????ㅼ쓣 ?뚮옉 蹂댁긽?쇰줈 ?쇱븘 諛곌???洹?怨곸쑝濡??좊룄(湲곗〈?ㅺ퀎 ?ㅽ뀅/???곸뿉 ?곕씪媛湲?.
// n<=0 ?대굹 ijk==NULL ?대㈃ ?뚮옉??鍮꾩슫??湲곗〈 ?숈옉). ? 醫뚰몴???꾩옱 寃⑹옄(set_grid) 湲곗? (i,j,k).
R3D_API R3dStatus r3d_set_corridor_cells(R3dEngine* e, const int32_t* ijk, int32_t n);

// ?쇱슦??吏꾪뻾 肄쒕갚(cdecl). 酉곗뼱 吏꾪뻾 ?ㅼ씠?쇰줈洹몄슜 ??泥섎━ ?쒖꽌쨌?꾩껜/媛쒕퀎 吏꾪뻾?꽷룹꽦怨??ㅽ뙣쨌吏?쑣룰꼍濡쒕?
// ?ㅼ떆媛꾩뿉 ?뚮┛?? ABI ?덉쟾: ?쇱슦?낃낵 媛숈? ?ㅻ젅?쒖뿉?쒕쭔 ?몄텧. 肄쒕갚 ?덉쇅??寃쎄퀎瑜??섍린吏 留?寃?
//   user         : ?몄뒪??而⑦뀓?ㅽ듃 ?ъ씤??洹몃?濡??ш퀬 ?ㅻ떂).
//   phase        : 0=?먯깋 吏꾪뻾(泥섎━?곹깭 %), 1=諛곌? ?꾨즺(寃곌낵 吏??+ 寃쎈줈).
//   order_index  : 泥섎━ ?쒖꽌(0遺?? priority ?뺣젹 湲곗?).
//   task_index   : ?먮낯 ?묒뾽 ?몃뜳??get_result ? ?숈씪 留ㅽ븨).
//   success      : phase==1 ?먯꽌留??좏슚(1/0).
//   length_mm/turns/expanded_nodes/elapsed_ms : phase==1 ??寃곌낵 吏??phase==0 ? expanded 留??좏슚).
//   done/total   : 吏꾪뻾瑜?done = ?꾨즺 諛곌? ?? total = ?꾩껜).
//   progress01   : phase==0 ???먯깋 吏꾪뻾??0~1(?대━?ㅽ떛 洹쇱젒 湲곕컲). phase==1 ?대㈃ 1.0.
//   path_ijk/path_len : phase==1 ?깃났 ??寃쎈줈 ?((i,j,k) ?곗냽, path_len 媛?. ?놁쑝硫?NULL/0.
//                       ?ъ씤?곕뒗 肄쒕갚 ?몄텧 ?덉뿉?쒕쭔 ?좏슚 ??利됱떆 蹂듭궗??寃?
//   諛섑솚媛?      : 0=怨꾩냽, 0 ?꾨떂=痍⑥냼(abort). ?먯깋 以?phase 0, ??5留??뺤옣留덈떎)쨌諛곌? ?꾨즺(phase 1)
//                  留덈떎 寃?ы븯誘濡??몄뒪?멸? 痍⑥냼瑜??붿껌?섎㈃ ?꾩옱 諛곌? ?먯깋??利됱떆 以묐떒?섍퀬
//                  ?욎꽌 諛곌?? 泥섎━?섏? ?딆? 梨꾨줈 route_multi_progress 媛 R3D_OK 瑜??ъ쟾??諛섑솚?쒕떎
//                  (?대? ?꾨즺??諛곌? 寃곌낵??蹂댁〈). ?묐젰??cooperative) 痍⑥냼.
typedef int32_t(__cdecl* R3dProgressFn)(void* user, int32_t phase, int32_t order_index,
                                        int32_t task_index, int32_t success, double length_mm,
                                        int32_t turns, int64_t expanded_nodes, double elapsed_ms,
                                        int32_t done, int32_t total, double progress01,
                                        const int32_t* path_ijk, int32_t path_len);
// r3d_route_multi ? ?숈씪(?쒖감쨌異⑸룎?놁쓬)?대릺 諛곌?留덈떎 cb 瑜??몄텧?쒕떎. cb 媛 null?대㈃ 肄쒕갚 ?놁씠 ?숈옉.
R3D_API R3dStatus r3d_route_multi_progress(R3dEngine* e, const char* priority, R3dProgressFn cb,
                                           void* user);

// rip-up & reroute(Step 3.8): ?쒖감 踰좎씠?ㅻ씪???댄썑 留됲엺 諛곌???'媛濡쒕쭑? 湲곗〈 諛곌?'??
// ?щ같移섑빐 ?댁냼?쒕떎. 臾댁넀??梨꾪깮(?깃났 +1) 寃곗젙???뚭퀬由ъ쬁. 寃곌낵???먮낯 ?묒뾽
// ?몃뜳?ㅻ퀎濡?蹂댁〈(get_result/copy_path 留ㅽ븨 ?좎?). 0=?ㅽ뙣 ?묒뾽 ?놁쓬 ?몄뿉???곹깭肄붾뱶.
//   max_rounds : ?쇱슫???곹븳.  max_ripup : ??踰덉뿉 ?щ같移섑븷 諛곌? ???곹븳.
R3D_API R3dStatus r3d_route_ripup(R3dEngine* e, const char* priority, int32_t max_rounds,
                                  int32_t max_ripup);

// 怨꾩링 corridor ?쇱슦???⑥씪, 媛쒕퀎). Sparse ?먯쑀 + coarse 媛?대뱶?뭚ine tube(astar_hashed,
// ?댁떆 湲곕컲) ??8,000m 湲?珥덈???寃⑹옄?먯꽌 諛곗뿴 ?좊떦 ?놁씠 ?숈옉. ?묒뾽蹂??낅┰(異⑸룎 ?뚰뵾 ?놁쓬).
//   factor : coarse/fine ? 鍮꾩쑉(湲곕낯 16).  radius : corridor ?쎌갹 諛섍꼍(coarse ?).
// 鍮꾩슜?⑥닔(?꾪솚/?대━?대윴???щ떇) 誘몄쟻????洹좎씪 鍮꾩슜. 寃곌낵???묒뾽 ?몃뜳?ㅻ퀎濡?蹂댁〈.
R3D_API R3dStatus r3d_route_corridor(R3dEngine* e, int32_t factor, int32_t radius);

// ?쒖감 怨꾩링 corridor ?쇱슦?????寃⑹옄 + 諛곌? 媛?異⑸룎 ?뚰뵾). r3d_route_corridor ? ?숈씪 ?붿쭊.
// Sparse + astar_hashed(?댁떆 湲곕컲, ??? 諛곗뿴 誘명븷???대릺, priority ?쒖꽌濡?諛곌????쇱슦?낇븯怨?
// 源붾┛ 寃쎈줈瑜?mark_pipe(pipe_radius) 濡??먯쑀 異붽????ㅼ쓬 諛곌???洹멸쾬???쇳븯?꾨줉 ?쒕떎(異⑸룎 0).
//   factor : coarse/fine ? 鍮꾩쑉.  radius : corridor ?쎌갹 諛섍꼍(coarse ?).
//   priority : "longest"|"shortest"|"utility"|"original".  pipe_radius : 源붾┛ 諛곌? ?쎌갹 諛섍꼍(fine ?).
// 鍮꾩슜?⑥닔(?꾪솚/?대━?대윴???щ떇) 誘몄쟻????洹좎씪 鍮꾩슜. 寃곌낵???묒뾽 ?몃뜳?ㅻ퀎濡?蹂댁〈.
R3D_API R3dStatus r3d_route_corridor_multi(R3dEngine* e, int32_t factor, int32_t radius,
                                           const char* priority, int32_t pipe_radius);

// 寃곌낵/寃쎈줈 議고쉶.
R3D_API R3dStatus r3d_get_result(const R3dEngine* e, int32_t task, R3dResult* out);
// 寃쎈줈 ???buf(int32_t[3*buf_cells], (i,j,k) ?곗냽)??蹂듭궗. 諛섑솚=?ㅼ젣 蹂듭궗??? ??
R3D_API int32_t r3d_copy_path(const R3dEngine* e, int32_t task, int32_t* buf, int32_t buf_cells);
// 諛⑸Ц(?먯깋) ???buf ??蹂듭궗(媛?쒗솕 '諛⑸Ц留?). 諛섑솚=?ㅼ젣 蹂듭궗??? ?? 鍮꾪솢??誘몄쭛怨꾨㈃ 0.
R3D_API int32_t r3d_copy_visited(const R3dEngine* e, int32_t task, int32_t* buf, int32_t buf_cells);

// 諛⑸Ц ? 吏묎퀎 on/off (湲곕낯 on=1). off ?대㈃ 諛⑸Ц留??놁씠 ?쇱슦????visited_len=0, copy_visited=0.
// 酉곗뼱 諛⑸Ц留??④퀎?먯깋 鍮꾪솢????硫붾え由??덉빟???좎슜 ??誘몃━ 0 ?쇰줈 ?ㅼ젙 ???쇱슦?낇븳??
R3D_API R3dStatus r3d_set_collect_visited(R3dEngine* e, int32_t enabled);

// 諛곌? ?먯쑀 ?쎌갹 諛섍꼍(?) ?ㅼ젙 ??諛곌?-諛곌? 異⑸룎 ?뚰뵾(?듭뀡1). route_multi(_progress) 媛 源붾┛ 諛곌???
// 寃쎈줈 짹radius 6-?댁썐源뚯? ?먯쑀濡?留됱븘 ?ㅼ쓬 諛곌? 以묒떖?좎쓣 洹몃쭔???꾩슫???ㅼ젣 愿寃쎈낫????踰뚮━硫?寃뱀묠 諛⑹?).
// 0=湲곗〈 ?숈옉(寃쎈줈 ?留?. env R3D_PIPE_RADIUS 濡쒕룄 ?ㅼ젙 媛?? ?뚯닔??0 ?쇰줈 ?대옩??
R3D_API R3dStatus r3d_set_pipe_radius(R3dEngine* e, int32_t radius_cells);

// per-task 愿寃?諛섍꼍(B1) ?쒖꽦?? enabled!=0 ?대㈃ route_multi(_progress) 媛 媛?諛곌???diameter_mm ? cell_mm
// 濡?留덊궧 諛섍꼍???먮룞 ?곗텧(radius = clamp(ceil(d/cell)-1, 0, 8)). ?곗텧??湲濡쒕쾶 pipe_radius ?곗텧 梨낆엫??
// ?쒓굅?섍퀬, 媛??諛곌???援듭? 諛곌? 諛섍꼍?쇰줈 怨쇳뙣?밸릺??臾몄젣瑜??댁냼?쒕떎. 愿寃?誘몄긽(0)쨌OFF?대㈃ 湲濡쒕쾶 pipe_radius
// ?대갚(湲곗〈 ?숈옉쨌怨⑤뱺 遺덈?). env R3D_PER_TASK_RADIUS 濡쒕룄 以????덈떎(?ㅻ뱶由ъ뒪 A/B).
R3D_API R3dStatus r3d_set_per_task_radius(R3dEngine* e, int32_t enabled);

// C1 negotiated-congestion(CBS-lite, Phase C) 源딆씠 ?ㅼ젙. 0=OFF(?됰㈃ rip-up留뙿룰린蹂??숈옉쨌怨⑤뱺 遺덈?). >0 ?대㈃
// ?됰㈃ rip-up ?쇰줈???닿껐?섏? ?딅뒗 ?ㅽ뙣 諛곌???blocker 媛 ?щ같移섎? 紐??섎㈃ 洹?blocker ??blocker 源뚯? ?대떦 源딆씠留뚰겮
// ?뚭퀬?ㅼ뼱 ?묐낫?쒖폒 ?댁냼?쒕떎(conflict-based search 寃쎈웾??. 臾댁넀?ㅒ룰껐?뺤쟻. [0,3] ?대옩?? env R3D_CBS 濡?媛??
R3D_API R3dStatus r3d_set_cbs_depth(R3dEngine* e, int32_t depth);

// C2 肄붾꼫 理쒖냼諛섍꼍 諛곗닔(Phase C) ?ㅼ젙. ?섎낫 ?ъ씠 吏곸꽑(?④?)??(mult 횞 愿寃? 誘몃쭔?대㈃ ?쒖옉 遺덇?(吏㏃? ?④?) ??
// 寃쎈줈(?) ?④퀎?먯꽌 異⑸룎寃???섏뿉 ?몄젒 ??肄붾꼫瑜?吏곴탳 ?⑹튂湲곕줈 ?≪닔?쒕떎(爰얠엫 鍮꾩쬆媛???뚮쭔, ???앹젏 怨좎젙).
// 0=OFF(湲곗〈 ?숈옉쨌怨⑤뱺 遺덈?). 沅뚯옣 2.0(?섎낫 媛?吏곸꽑 ??2횞愿寃?. env R3D_MIN_STRAIGHT 濡?媛??
R3D_API R3dStatus r3d_set_min_straight(R3dEngine* e, double mult);

// 肄붾꼫 理쒖냼吏곸꽑(?덈? mm, ?섎뱶 ?쒖빟). >0 ?대㈃ A* ?먯깋??'??踰?爰얠씤 ????湲몄씠留뚰겮 吏곸쭊?섍린 ?꾩뿏 ?ㅼ떆
// 爰얠? 紐삵븯?꾨줉' 媛뺤젣?쒕떎 ???섎낫 媛?紐⑤뱺 吏곸꽑 援ш컙(?④?) ????湲몄씠. r3d_set_min_straight(愿寃?諛곗닔쨌?꾩쿂由?
// ?≪닔)? ?щ━ ?먯깋 ?④퀎???섎뱶 蹂댁옣?대ŉ 愿寃?臾닿?쨌??諛곌? ?곸슜(紐⑺몴 吏곸쟾 留덉?留??묒냽 援ш컙留?硫댁젣). ?濡쒕뒗
// ceil(mm/cell) 濡??섏궛. 0=OFF(湲곗〈 ?숈옉쨌怨⑤뱺 遺덈?). env R3D_MIN_STRAIGHT_MM ?쇰줈 ?ъ젙?? 沅뚯옣 100mm.
R3D_API R3dStatus r3d_set_min_straight_mm(R3dEngine* e, double mm);

// 諛곌?-諛곌? ?닿꺽(mm) ?ㅼ젙 ????諛곌? ?쇳꽣??嫄곕━ ??r1 + r2 + gap_mm 蹂댁옣(?쒕㈃ ?닿꺽 理쒖냼 gap_mm ?댁긽).
// 湲곗〈 留덊궧? ?쇳꽣?좎쓣 ~愿寃?d)留뚰겮留??꾩썙 ?쒕㈃??留욌떯?섎떎(寃뱀퀜 蹂댁엫). gap>0 ?대㈃ route_multi 硫붿씤 猷⑦봽媛
// 源붾┛ 諛곌???'routing 諛곌? 湲곗? ?쎌갹 諛섍꼍 = ceil((r_a+r_b+gap)/cell)'?쇰줈 留됱븘 ?뺥솗??r_a+r_b+gap ?닿꺽??
// 蹂댁옣?쒕떎(per-pipe 援ы쁽). 0=OFF(湲곗〈 ?숈옉쨌怨⑤뱺 遺덈?). 洹쒓꺽 沅뚯옣 60mm. env R3D_PIPE_GAP.
R3D_API R3dStatus r3d_set_pipe_gap(R3dEngine* e, double gap_mm);

// Segment A* opt-in. enabled!=0 tries straight-run expansion first, then falls back to existing A*.
// max_segment_cells clamps to the implementation range; 0 uses the default.
R3D_API R3dStatus r3d_set_segment_astar(R3dEngine* e, int32_t enabled, int32_t max_segment_cells);

// Octree-guided routing opt-in. Uses Octree Jump A* as a macro path, then fine A* inside a corridor.
// Applies to route_multi(_progress) escalation only; 0 disables it.
R3D_API R3dStatus r3d_set_octree_guide(R3dEngine* e, int32_t enabled, int32_t corridor_radius);

// TruckIn/Middle/Terminal split routing opt-in. trunk_z_mm<=0 selects rack/auto trunk height.
R3D_API R3dStatus r3d_set_route_split(R3dEngine* e, int32_t enabled, double trunk_z_mm);

// ?먯쑀(釉붾줉???) ?몃뜳?ㅻ? buf ??蹂듭궗(媛?쒗솕 '?먯쑀留?). ?꾩옱 doc ??obstacles 瑜?利됱꽍 voxelize.
// 諛섑솚=?ㅼ젣 蹂듭궗??? ?? buf_cells 媛 遺議깊븯硫?泥섏쓬 buf_cells 媛쒕쭔 蹂듭궗?섍퀬 洹몃쭔?붾떎.
// 珥?釉붾줉 ? ?섎? 誘몃━ ?뚮젮硫?buf=NULL, buf_cells=0 ?쇰줈 ?몄텧(珥???諛섑솚).
// [二쇱쓽] ????뚮옖???섏뼲 ?)?먯꽌 ?꾩껜 蹂듭궗??硫붾え由??쒓컙 ??컻 ?꾪뿕.
//        UI 誘몃━蹂닿린?먮뒗 r3d_copy_blocked_sampled ?ъ슜 沅뚯옣.
R3D_API int32_t r3d_copy_blocked(const R3dEngine* e, int32_t* buf, int32_t buf_cells);

// ???寃⑹옄 ?쒓컖?붿슜 ??blocked cell ??理쒕? max_cells 媛?洹좎씪 ?섑뵆留곹빐 buf ??蹂듭궗.
// ?꾩껜 媛쒖닔??r3d_copy_blocked(e, NULL, 0) 濡?議고쉶.
// 諛섑솚=?ㅼ젣 蹂듭궗 ? ???쨗ax_cells). max_cells?? ?먮뒗 buf=NULL ?대㈃ 0 諛섑솚.
// ????뚮옖???섏뼲 ?)?먯꽌 r3d_copy_blocked ?꾩껜 ?붿껌 ??????⑥닔濡??쒗븳???섎? 諛쏅뒗??
R3D_API int32_t r3d_copy_blocked_sampled(const R3dEngine* e, int32_t max_cells, int32_t* buf);

// ?듦낵 媛앹껜 ?먯쑀 ? ?몃뜳?ㅻ? buf ??蹂듭궗(媛?쒗솕 '?듦낵 ?먯쑀留?). r3d_copy_blocked ? ?숈씪 洹쒖빟.
R3D_API int32_t r3d_copy_passthrough(const R3dEngine* e, int32_t* buf, int32_t buf_cells);

// ?꾩옱 ?곹깭瑜?scene ?띿뒪??UTF-8)濡??ㅽ봽(Python ?붾쾭嫄?援먯감寃利?. out_text ???댁젣 ?꾩슂.
R3D_API R3dStatus r3d_dump_scene_text(const R3dEngine* e, char** out_text);

// ---- 媛蹂? ?ν듃由??쇱슦??----
// ?μ븷臾?AABB 瑜??ν듃由щ줈 ?됱씤?섍퀬 ?먯쑀怨듦컙?먯꽌 ????????踰덉뿉 ?먰봽?섎뒗
// ?곸쓳??A* 濡??⑥씪 諛곌????먯깋?쒕떎. 10mm ?댄븯 誘몄꽭寃⑹옄 理쒕떒寃쎈줈???좊━.
// task: r3d_add_task ?몃뜳?? max_exp: ?먯깋 ?곹븳(0=臾댁젣??.
// goal_dir: 吏꾩엯異?-1=臾댁젣?? 0..5=NEIGHBORS_6). out: 寃곌낵 POD.
R3D_API R3dStatus r3d_route_task_octree(R3dEngine* e, int32_t task,
                                        int64_t max_exp, int32_t goal_dir,
                                        R3dResult* out);

// ---- ?ν듃由?由ы봽 ?닿굅 (3D 媛?쒗솕?? ----
// ?붿쭊??濡쒕뱶????臾몄꽌濡??ν듃由щ? 鍮뚮뱶?섍퀬 紐⑤뱺 由ы봽 ?몃뱶瑜?buf ??梨꾩슫??
// x0_mm/y0_mm/z0_mm: 由ы봽 ?먯젏(mm). size_mm: 由ы봽 ??蹂 ?ш린(mm). state: 0=FREE, 1=BLOCKED.
// buf: ?몄텧???좊떦 諛곗뿴(maxCount). *out_count: ?ㅼ젣 梨꾩슫 媛쒖닔.
// R3D_ERR_ARG: e=null / scene 誘몃줈??/ buf=null / maxCount<=0
typedef struct {
    float x0_mm, y0_mm, z0_mm;   // 由ы봽 ?먯젏 (world mm)
    float size_mm;                 // 由ы봽 ??蹂 ?ш린 (mm)
    int32_t state;                 // 0=FREE, 1=BLOCKED
} R3dOctreeLeaf;

R3D_API R3dStatus r3d_enum_octree_leaves(R3dEngine* e,
                                          R3dOctreeLeaf* buf, int32_t maxCount,
                                          int32_t* out_count);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ROUTING3D_CAPI_H
