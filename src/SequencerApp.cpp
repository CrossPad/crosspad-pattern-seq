/**
 * @file SequencerApp.cpp
 * @brief Sequencer -- the loop recorder, on a screen of its own.
 *
 * Everything that used to slide in from the Sampler's "Advanced" strip: tempo,
 * click, the scene's length and grid, x2 and /2, the scene picker, held ERASE
 * and the transport -- around a view of what is recorded. The pattern is drawn
 * as sixteen lanes, one per pad, across the sounding scene's length; every
 * note a tick in its pad's colour, brighter the harder it was hit, and a
 * playhead running over them. Hold ERASE, hit a pad, and its lane empties.
 *
 * BOUNCE plays the sounding scene once, from its bar one, and writes its pads
 * to a clip in the open song project, on the track picked -- where Arrange
 * lays it out with the others.
 *
 * The pads stay the Sampler's: no pad logic here, so they play the kit and
 * record exactly as they do from any other screen. The loop keeps running
 * when the app closes; it lives in the sequencer, not in this screen.
 */

#include "crosspad-sequencer/SequencerApp.hpp"

// `class App` lives in a different header per platform, but both derive from
// crosspad_gui::AppBase<App>, so the lifecycle calls used here are identical.
#if __has_include("pc_stubs/PcApp.hpp")
  #include "pc_stubs/PcApp.hpp"
#else
  #include "crosspad-gui/app/App.hpp"
#endif

#include "crosspad/app/AppRegistrar.hpp"
#include "crosspad/event/EventData.hpp"
#include "crosspad/event/IEventBus.hpp"
#include "crosspad/kit/IKitManager.hpp"
#include "crosspad/kit/KitInfo.hpp"
#include "crosspad/platform/PlatformServices.hpp"
#include "crosspad/sequencer/SequencerController.hpp"
#include "crosspad/song/ISongEngine.hpp"
#include "crosspad-gui/widgets/gui_helpers.h"
#include "crosspad-gui/widgets/spinbox_helpers.h"

#include "lvgl.h"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

// Encoder navigation is an embedded-only concern -- the simulator drives the UI
// with a pointer. Snapshot whichever group was active before the app opened and
// restore that, rather than a global launcher group that goes stale whenever the
// launcher is rebuilt.
#ifdef ESP_PLATFORM
extern "C" {
  extern lv_indev_t* indev_encoderTop;
}
#define SEQ_USE_ENCODER 1
#endif

namespace {

constexpr uint32_t kTickMs = 50;
constexpr int      kLanes  = 16;

const lv_color_t kBg     = lv_color_hex(0x16181B);
const lv_color_t kPanel  = lv_color_hex(0x20242A);
const lv_color_t kLine   = lv_color_hex(0x2C3138);
const lv_color_t kBarLn  = lv_color_hex(0x4A515B);
const lv_color_t kText   = lv_color_hex(0xE8EAED);
const lv_color_t kDim    = lv_color_hex(0x7B828B);
const lv_color_t kInk    = lv_color_hex(0x16181B);
const lv_color_t kAccent = lv_color_hex(0xC6FF47);
const lv_color_t kRec    = lv_color_hex(0xFF5C5C);
const lv_color_t kPlay   = lv_color_hex(0x4FE0B0);
const lv_color_t kAmber  = lv_color_hex(0xFFC857);
const lv_color_t kHead   = lv_color_hex(0xFFFFFF);
/// The tempo field while a host's MIDI clock drives the tempo.
const lv_color_t kHostClock = lv_color_hex(0x4FC3F7);

lv_obj_t* s_root = nullptr;
lv_obj_t* s_grid = nullptr;
lv_obj_t* s_bpm = nullptr;
lv_obj_t* s_clickBtn = nullptr;
lv_obj_t* s_preBtn = nullptr;
lv_obj_t* s_barsLabel = nullptr;
lv_obj_t* s_quantBtn = nullptr;
lv_obj_t* s_quantLabel = nullptr;
lv_obj_t* s_sceneLabel = nullptr;
lv_obj_t* s_eraseBtn = nullptr;
lv_obj_t* s_recBtn = nullptr;
lv_obj_t* s_dubBtn = nullptr;
lv_obj_t* s_playBtn = nullptr;
lv_obj_t* s_stopBtn = nullptr;
lv_timer_t* s_timer = nullptr;
bool s_bpmSyncing = false;   // a sync must not read back as an edit
/* Subscribed on the first open and kept: the clock publishes only changes, so
 * a screen opened while the host already drives must find the flag current. */
crosspad::SubscriptionHandle s_tempoSub = nullptr;
std::atomic<bool> s_tempoFromHost{false};

/* The bounce window: track picker, then progress, then the result. */
lv_obj_t* s_bounceBox = nullptr;
lv_obj_t* s_bounceText = nullptr;
lv_obj_t* s_bounceBar = nullptr;
lv_obj_t* s_bounceBtn = nullptr;     // Cancel while running, OK after
bool      s_bouncing = false;

/* What the grid was last drawn from; redrawn whole when it changes. The
 * playhead is the only thing that moves while a scene plays, and only its
 * two columns (where it was, where it is) are redrawn for it: the whole grid
 * every tick kept the LVGL task busy most of the time on the core the card
 * writers share. */
uint32_t s_drawn = 0;
int32_t  s_headX = -1;   // screen x of the playhead as drawn, -1 = none

#ifdef SEQ_USE_ENCODER
lv_group_t* s_encGroup = nullptr;
lv_group_t* s_prevEncGroup = nullptr;
lv_group_t* s_bounceGroup = nullptr;
#endif

crosspad::SequencerController* seq()
{
    return crosspad::getPlatformServices().sequencer;
}

crosspad::KitInfo* current_kit()
{
    auto* km = crosspad::getKitManager();
    return km ? km->getCurrentKit() : nullptr;
}

bool running()
{
    auto* sq = seq();
    return sq && sq->getState() == crosspad::SequencerState::Running;
}

// ── pattern grid ───────────────────────────────────────────────────────────

void fill(lv_layer_t* layer, int32_t x1, int32_t y1, int32_t x2, int32_t y2,
          lv_color_t c, lv_opa_t opa)
{
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_color = c;
    d.bg_opa = opa;
    d.radius = 0;
    lv_area_t a = {x1, y1, x2, y2};
    lv_draw_rect(layer, &d, &a);
}

/// Drawn straight into the layer rather than built from objects: a full scene
/// is 192 notes, and 192 LVGL objects rebuilt on every hit would cost more
/// than the pattern is worth.
void grid_draw_cb(lv_event_t* e)
{
    auto* sq = seq();
    lv_layer_t* layer = lv_event_get_layer(e);
    lv_obj_t* obj = static_cast<lv_obj_t*>(lv_event_get_target(e));
    if (!sq || !layer) return;

    lv_area_t box;
    lv_obj_get_content_coords(obj, &box);
    const int32_t w = lv_area_get_width(&box);
    const int32_t h = lv_area_get_height(&box);
    const int32_t laneH = h / kLanes;
    const int32_t x0 = box.x1 + 4;          // after the pad-colour strip
    const int32_t gw = w - 4;
    if (laneH < 2 || gw < 16) return;

    const uint8_t scene = sq->getActiveScene();
    const int bars = sq->getSceneBars(scene) ? sq->getSceneBars(scene) : 1;
    const int beats = bars * 4;
    crosspad::KitInfo* kit = current_kit();

    /* Lanes: the pad's own colour as a strip at the left edge, so a note's
     * colour can be matched to its row without a label. */
    for (int p = 0; p < kLanes; ++p) {
        const int32_t y = box.y1 + p * laneH;
        const lv_color_t pc = kit ? lv_color_hex(kit->pads[p].color) : kDim;
        fill(layer, box.x1, y, box.x1 + 2, y + laneH - 2, pc, LV_OPA_COVER);
        if (p % 4 == 3 && p < kLanes - 1) {
            fill(layer, x0, y + laneH - 1, x0 + gw - 1, y + laneH - 1, kLine, LV_OPA_COVER);
        }
    }
    /* Beats faint, bars strong: the grid a take is snapped to. */
    for (int b = 0; b <= beats; ++b) {
        const int32_t x = x0 + (int32_t)((int64_t)gw * b / beats);
        fill(layer, x, box.y1, x, box.y1 + laneH * kLanes - 1,
             b % 4 == 0 ? kBarLn : kLine, LV_OPA_COVER);
    }

    const int32_t noteW = gw / (beats * 4) > 2 ? gw / (beats * 4) - 1 : 2;
    const uint16_t n = sq->getSceneEventCount(scene);
    for (uint16_t i = 0; i < n; ++i) {
        float at = 0.0f;
        uint8_t pad = 0, vel = 0;
        if (!sq->getSceneEvent(scene, i, at, pad, vel) || pad >= kLanes) continue;
        const int32_t x = x0 + (int32_t)(at * gw);
        const int32_t y = box.y1 + pad * laneH;
        const lv_color_t pc = kit ? lv_color_hex(kit->pads[pad].color) : kAccent;
        const lv_opa_t opa = (lv_opa_t)(110 + (vel > 127 ? 127 : vel) * 145 / 127);
        fill(layer, x, y, x + noteW - 1, y + laneH - 2, pc, opa);
    }

    if (s_headX >= 0) fill(layer, s_headX, box.y1, s_headX + 1, box.y1 + laneH * kLanes - 1, kHead, LV_OPA_COVER);
}

/// Where the playhead belongs now, in screen x; -1 when stopped.
int32_t head_x()
{
    auto* sq = seq();
    if (!sq || !s_grid || !running()) return -1;
    lv_area_t box;
    lv_obj_get_content_coords(s_grid, &box);
    const int32_t gw = lv_area_get_width(&box) - 4;
    return box.x1 + 4 + (int32_t)(sq->getProgress() * gw);
}

void invalidate_head(int32_t x)
{
    if (x < 0 || !s_grid) return;
    lv_area_t box;
    lv_obj_get_content_coords(s_grid, &box);
    lv_area_t a = {x, box.y1, x + 1, box.y2};
    lv_obj_invalidate_area(s_grid, &a);
}

/// Everything the grid shows that is not the playhead.
uint32_t pattern_shape()
{
    auto* sq = seq();
    if (!sq) return 0;
    const uint8_t s = sq->getActiveScene();
    return ((uint32_t)s << 24) ^ ((uint32_t)sq->getSceneBars(s) << 16) ^
           sq->getSceneEventCount(s) ^ ((uint32_t)sq->getSceneCount() << 20);
}

// ── state on the buttons ───────────────────────────────────────────────────

/* Only what changed: setting a style, even to the value it has, refreshes the
 * object and redraws it, and sync_ui() runs every tick. */
void paint(lv_obj_t* b, lv_color_t bg, bool dark)
{
    if (!b) return;
    if (!lv_color_eq(lv_obj_get_style_bg_color(b, 0), bg)) lv_obj_set_style_bg_color(b, bg, 0);
    lv_obj_t* l = lv_obj_get_child(b, 0);
    const lv_color_t ink = dark ? kInk : kText;
    if (l && !lv_color_eq(lv_obj_get_style_text_color(l, 0), ink)) lv_obj_set_style_text_color(l, ink, 0);
}

void set_text(lv_obj_t* l, const char* text)
{
    if (l && std::strcmp(lv_label_get_text(l), text) != 0) lv_label_set_text(l, text);
}

void sync_ui()
{
    auto* sq = seq();
    if (!sq) return;

    const bool run = running();
    const auto mode = sq->getMode();
    const bool rec = run && mode == crosspad::SequencerMode::Record;
    const bool dub = run && mode == crosspad::SequencerMode::Overdub;
    paint(s_recBtn,  rec ? kRec : kPanel, rec);
    paint(s_dubBtn,  dub ? kRec : kPanel, dub);

    /* Counting in: the take's button counts down the beats left, amber, and
     * turns red on its beat one. */
    const uint8_t left = sq->getCountInLeft();
    lv_obj_t* waiting = !left ? nullptr
                      : (mode == crosspad::SequencerMode::Overdub ? s_dubBtn : s_recBtn);
    char buf[24];
    if (waiting) {
        paint(waiting, kAmber, true);
        std::snprintf(buf, sizeof(buf), "%u", (unsigned)left);
        set_text(lv_obj_get_child(waiting, 0), buf);
    }
    if (waiting != s_recBtn) set_text(lv_obj_get_child(s_recBtn, 0), "REC");
    if (waiting != s_dubBtn) set_text(lv_obj_get_child(s_dubBtn, 0), "DUB");
    paint(s_preBtn, sq->getCountIn() ? kAmber : kPanel, sq->getCountIn());
    paint(s_playBtn, (run && !rec && !dub) ? kPlay : kPanel, run && !rec && !dub);
    paint(s_stopBtn, kPanel, false);
    paint(s_clickBtn, sq->getClickActive() ? kAmber : kPanel, sq->getClickActive());
    if (!sq->isErasing()) paint(s_eraseBtn, kPanel, false);

    const bool off = sq->getQuantization() == crosspad::Quantization::Off;
    paint(s_quantBtn, off ? kAmber : kPanel, off);
    const char* q = "1/16";
    switch (sq->getQuantization()) {
        case crosspad::Quantization::Quarter:      q = "1/4";  break;
        case crosspad::Quantization::Eighth:       q = "1/8";  break;
        case crosspad::Quantization::ThirtySecond: q = "1/32"; break;
        case crosspad::Quantization::Off:          q = "free"; break;
        default: break;
    }
    set_text(s_quantLabel, q);

    const uint8_t active = sq->getActiveScene();
    const uint8_t pending = sq->getPendingScene();
    const uint8_t bars = sq->getSceneBars(active) ? sq->getSceneBars(active) : 1;
    std::snprintf(buf, sizeof(buf), "%ub", (unsigned)bars);
    set_text(s_barsLabel, buf);

    /* The scene you asked for, amber until the loop end hands it over. */
    if (pending != active) {
        std::snprintf(buf, sizeof(buf), "Seq %u > %u", (unsigned)(active + 1), (unsigned)(pending + 1));
    } else {
        std::snprintf(buf, sizeof(buf), "Seq %u / %u", (unsigned)(active + 1), (unsigned)sq->getSceneCount());
    }
    set_text(s_sceneLabel, buf);
    const lv_color_t sceneInk = pending != active ? kAmber : kText;
    if (!lv_color_eq(lv_obj_get_style_text_color(s_sceneLabel, 0), sceneInk)) {
        lv_obj_set_style_text_color(s_sceneLabel, sceneInk, 0);
    }

    if (s_bpm && !lv_obj_has_state(s_bpm, LV_STATE_EDITED)) {
        const int32_t shown = (int32_t)lroundf(sq->getBpm());
        if (lv_spinbox_get_value(s_bpm) != shown) {
            s_bpmSyncing = true;
            lv_spinbox_set_value(s_bpm, shown);
            s_bpmSyncing = false;
        }
        const lv_color_t field = s_tempoFromHost.load(std::memory_order_relaxed) ? kHostClock : kPanel;
        if (!lv_color_eq(lv_obj_get_style_bg_color(s_bpm, 0), field)) {
            lv_obj_set_style_bg_color(s_bpm, field, 0);
        }
    }
}

void bounce_progress();

void tick_cb(lv_timer_t*)
{
    if (s_bouncing) bounce_progress();
    sync_ui();
    const uint32_t shape = pattern_shape();
    const int32_t x = head_x();
    if (shape != s_drawn) {
        s_drawn = shape;
        s_headX = x;
        if (s_grid) lv_obj_invalidate(s_grid);
    } else if (x != s_headX) {
        invalidate_head(s_headX);
        s_headX = x;
        invalidate_head(s_headX);
    }
}

// ── actions ────────────────────────────────────────────────────────────────

void back_cb(lv_event_t* e)
{
    auto* app = static_cast<App*>(lv_event_get_user_data(e));
    if (app) app->destroyAppAsync();
}

/// The tempo goes to the clock at once -- the pattern is rescaled with it, so
/// a take keeps its place in the bar -- and sequences.json carries it with
/// the scenes, which is what a kit loads its tempo back from.
void bpm_cb(lv_event_t* e)
{
    auto* sq = seq();
    if (!sq || s_bpmSyncing) return;
    /* No set_cursor_pos()/set_step() here: both rewrite the text, which sends
     * VALUE_CHANGED straight back into this handler. The cursor is pinned on
     * focus and click instead. */
    auto* field = static_cast<lv_obj_t*>(lv_event_get_target(e));
    if (s_tempoFromHost.load(std::memory_order_relaxed)) {
        /* The DAW's clock owns the tempo: put the field back and touch neither
         * the kit nor sequences.json. */
        s_bpmSyncing = true;
        lv_spinbox_set_value(field, (int32_t)lroundf(sq->getBpm()));
        s_bpmSyncing = false;
        return;
    }
    const int32_t v = lv_spinbox_get_value(field);
    if ((int32_t)lroundf(sq->getBpm()) == v) return;
    sq->setBpm((float)v);
    if (auto* kit = current_kit()) kit->bpm = (uint16_t)v;
}

/// A bar of click before a take started from a stop, so the first hit lands
/// on beat one instead of on the button press.
void pre_cb(lv_event_t*)
{
    if (auto* sq = seq()) sq->setCountIn(!sq->getCountIn());
    sync_ui();
}

void click_cb(lv_event_t*)
{
    if (auto* sq = seq()) sq->setClickActive(!sq->getClickActive());
    sync_ui();
}

void bars_cb(lv_event_t*)
{
    auto* sq = seq();
    if (!sq) return;
    const uint8_t s = sq->getActiveScene();
    const uint8_t bars = sq->getSceneBars(s);
    sq->setSceneBars(s, (bars >= 8) ? 1 : (uint8_t)(bars ? bars * 2 : 1));
    sync_ui();
}

/// Off, 1/4, 1/8, 1/16, 1/32 and round. Off is one press away because a part
/// with feel in it is ruined by the same snap that flatters most playing.
void quant_cb(lv_event_t*)
{
    auto* sq = seq();
    if (!sq) return;
    switch (sq->getQuantization()) {
        case crosspad::Quantization::Off:       sq->setQuantization(crosspad::Quantization::Quarter);      break;
        case crosspad::Quantization::Quarter:   sq->setQuantization(crosspad::Quantization::Eighth);       break;
        case crosspad::Quantization::Eighth:    sq->setQuantization(crosspad::Quantization::Sixteenth);    break;
        case crosspad::Quantization::Sixteenth: sq->setQuantization(crosspad::Quantization::ThirtySecond); break;
        default:                                sq->setQuantization(crosspad::Quantization::Off);          break;
    }
    sync_ui();
}

void double_cb(lv_event_t*) { if (auto* sq = seq()) sq->doubleScene(sq->getActiveScene()); sync_ui(); }
void halve_cb(lv_event_t*)  { if (auto* sq = seq()) sq->halveScene(sq->getActiveScene()); sync_ui(); }

void scene_step(int d)
{
    auto* sq = seq();
    if (!sq) return;
    const uint8_t n = sq->getSceneCount();
    if (!n) return;
    sq->selectScene((uint8_t)((sq->getPendingScene() + n + d) % n));
    sync_ui();
}
void scene_prev_cb(lv_event_t*) { scene_step(-1); }
void scene_next_cb(lv_event_t*) { scene_step(+1); }

/// Minus drops the scene on show, plus appends an empty one and goes to it.
/// The last scene never goes -- a kit has to have somewhere to record.
void scene_del_cb(lv_event_t*) { if (auto* sq = seq()) sq->removeScene(sq->getActiveScene()); sync_ui(); }
void scene_add_cb(lv_event_t*) { if (auto* sq = seq()) sq->addScene(); sync_ui(); }

/// Held, not toggled: hold it (the encoder's button or a finger) and hit
/// pads -- each pad hit leaves the sounding scene. Letting go ends it.
void erase_cb(lv_event_t* e)
{
    const lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_PRESSED && code != LV_EVENT_RELEASED && code != LV_EVENT_PRESS_LOST) return;
    const bool on = code == LV_EVENT_PRESSED;
    if (auto* sq = seq()) sq->setErasing(on);
    paint(s_eraseBtn, on ? kRec : kPanel, on);
}

/// A second REC closes the take without stopping the loop, which is what you
/// want after playing the part in.
void rec_cb(lv_event_t*)
{
    auto* sq = seq();
    if (!sq) return;
    if (sq->isRecording()) sq->playback();
    else                   sq->record();
    sync_ui();
}
void dub_cb(lv_event_t*)  { if (auto* sq = seq()) sq->overdub();  sync_ui(); }
void play_cb(lv_event_t*) { if (auto* sq = seq()) sq->playback(); sync_ui(); }
void stop_cb(lv_event_t*) { if (auto* sq = seq()) sq->stop();     sync_ui(); }

// ── bounce ─────────────────────────────────────────────────────────────────

crosspad::ISongEngine* songs() { return crosspad::getPlatformServices().songEngine; }

void bounce_close()
{
    if (!s_bounceBox) return;
#ifdef SEQ_USE_ENCODER
    if (indev_encoderTop && s_encGroup) lv_indev_set_group(indev_encoderTop, s_encGroup);
    if (s_bounceGroup) { lv_group_delete(s_bounceGroup); s_bounceGroup = nullptr; }
#endif
    lv_obj_delete_async(s_bounceBox);
    s_bounceBox = s_bounceText = s_bounceBar = s_bounceBtn = nullptr;
    s_bouncing = false;
}

void bounce_close_cb(lv_event_t*)
{
    if (s_bouncing) {
        if (auto* se = songs()) se->cancelBounce();   // the result closes it
        return;
    }
    bounce_close();
}

lv_obj_t* bounce_button(lv_obj_t* parent, const char* text, lv_event_cb_t cb, void* ud)
{
    lv_obj_t* b = lv_button_create(parent);
    lv_obj_set_size(b, LV_PCT(100), 26);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, kPanel, 0);
    lv_obj_set_style_outline_color(b, kHead, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(b, 2, LV_STATE_FOCUS_KEY);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_style_text_color(l, kText, 0);
    lv_label_set_text(l, text);
    lv_obj_center(l);
#ifdef SEQ_USE_ENCODER
    if (s_bounceGroup) lv_group_add_obj(s_bounceGroup, b);
#endif
    return b;
}

/// Where the take is going, then how far it got.
void bounce_track_cb(lv_event_t* e)
{
    auto* se = songs();
    auto* sq = seq();
    if (!se || !sq || !s_bounceBox) return;
    const uint8_t track = (uint8_t)(intptr_t)lv_event_get_user_data(e);
    const uint8_t scene = sq->getActiveScene();

    lv_obj_clean(s_bounceBox);
#ifdef SEQ_USE_ENCODER
    if (s_bounceGroup) lv_group_remove_all_objs(s_bounceGroup);
#endif
    s_bounceText = lv_label_create(s_bounceBox);
    lv_obj_set_style_text_color(s_bounceText, kText, 0);
    lv_obj_set_width(s_bounceText, LV_PCT(100));
    lv_label_set_long_mode(s_bounceText, LV_LABEL_LONG_WRAP);
    lv_label_set_text_fmt(s_bounceText, "Bouncing Seq %u to track %u...",
                          (unsigned)(scene + 1), (unsigned)(track + 1));
    s_bounceBar = lv_bar_create(s_bounceBox);
    lv_obj_set_size(s_bounceBar, LV_PCT(100), 10);
    lv_bar_set_range(s_bounceBar, 0, 1000);
    lv_obj_set_style_bg_color(s_bounceBar, kPanel, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_bounceBar, kRec, LV_PART_INDICATOR);
    s_bounceBtn = bounce_button(s_bounceBox, "Cancel", bounce_close_cb, nullptr);
#ifdef SEQ_USE_ENCODER
    if (s_bounceGroup) lv_group_focus_obj(s_bounceBtn);
#endif

    /* The take needs a ring in PSRAM, and PSRAM is what the UI runs out of.
     * Decoded icons are the part of it that costs nothing to give back: they
     * are read again the next time they are drawn. */
    lv_image_cache_drop(nullptr);
    s_bouncing = se->bounceScene(scene, track);
    if (!s_bouncing) {
        lv_label_set_text(s_bounceText, "Could not start: a bounce or the song is already running.");
        lv_label_set_text(lv_obj_get_child(s_bounceBtn, 0), "OK");
    }
}

void bounce_progress()
{
    auto* se = songs();
    if (!se || !s_bounceText) return;
    const crosspad::BounceStatus st = se->bounceStatus();
    if (st.framesTotal) {
        lv_bar_set_value(s_bounceBar, (int32_t)((uint64_t)st.framesDone * 1000 / st.framesTotal), LV_ANIM_OFF);
    }
    if (st.state == crosspad::BounceState::Running) return;
    s_bouncing = false;
    if (st.state == crosspad::BounceState::Done) {
        lv_bar_set_value(s_bounceBar, 1000, LV_ANIM_OFF);
        lv_obj_set_style_bg_color(s_bounceBar, kPlay, LV_PART_INDICATOR);
        lv_label_set_text_fmt(s_bounceText, "Done: %s is on its track in Arrange.%s%s", st.file.c_str(),
                              st.error.empty() ? "" : "\nNote: ", st.error.c_str());
    } else {
        lv_label_set_text_fmt(s_bounceText, "Bounce failed: %s", st.error.c_str());
    }
    lv_label_set_text(lv_obj_get_child(s_bounceBtn, 0), "OK");
}

void bounce_cb(lv_event_t*)
{
    auto* se = songs();
    auto* sq = seq();
    if (!se || !sq || s_bounceBox) return;

    s_bounceBox = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_bounceBox);
    lv_obj_set_size(s_bounceBox, LV_PCT(100), LV_PCT(100));
    lv_obj_add_flag(s_bounceBox, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_style_bg_color(s_bounceBox, kBg, 0);
    lv_obj_set_style_bg_opa(s_bounceBox, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_bounceBox, 8, 0);
    lv_obj_set_style_pad_row(s_bounceBox, 5, 0);
    lv_obj_set_flex_flow(s_bounceBox, LV_FLEX_FLOW_COLUMN);
#ifdef SEQ_USE_ENCODER
    s_bounceGroup = lv_group_create();
    if (indev_encoderTop) lv_indev_set_group(indev_encoderTop, s_bounceGroup);
#endif

    std::string project;
    {
        std::lock_guard<std::recursive_mutex> l(se->songMutex());
        project = se->song().name;
    }
    lv_obj_t* t = lv_label_create(s_bounceBox);
    lv_obj_set_style_text_color(t, kText, 0);
    lv_obj_set_width(t, LV_PCT(100));
    lv_label_set_long_mode(t, LV_LABEL_LONG_WRAP);
    lv_label_set_text_fmt(t, "Bounce Seq %u (%u bars) into \"%s\" -- on which track?",
                          (unsigned)(sq->getActiveScene() + 1),
                          (unsigned)sq->getSceneBars(sq->getActiveScene()),
                          project.empty() ? "no card" : project.c_str());
    for (int i = 0; i < crosspad::AudioSong::kTracks; ++i) {
        char label[16];
        lv_snprintf(label, sizeof(label), "Track %d", i + 1);
        bounce_button(s_bounceBox, label, bounce_track_cb, (void*)(intptr_t)i);
    }
    bounce_button(s_bounceBox, "Cancel", bounce_close_cb, nullptr);
}

// ── building ───────────────────────────────────────────────────────────────

lv_obj_t* make_button(lv_obj_t* parent, int32_t w, int32_t h, const char* text,
                      lv_event_cb_t cb, void* ud = nullptr)
{
    lv_obj_t* b = lv_button_create(parent);
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, kPanel, 0);
    lv_obj_set_style_outline_color(b, kHead, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(b, 2, LV_STATE_FOCUS_KEY);
    if (cb) lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(l, kText, 0);
    lv_label_set_text(l, text);
    lv_obj_center(l);
#ifdef SEQ_USE_ENCODER
    if (s_encGroup) lv_group_add_obj(s_encGroup, b);
#endif
    return b;
}

lv_obj_t* make_row(lv_obj_t* parent, int32_t h)
{
    lv_obj_t* r = lv_obj_create(parent);
    lv_obj_remove_style_all(r);
    lv_obj_set_size(r, LV_PCT(100), h);
    lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(r, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(r, 3, 0);
    lv_obj_remove_flag(r, LV_OBJ_FLAG_SCROLLABLE);
    return r;
}

/// Tempo: focus it, press to edit, turn a BPM at a time, press again to
/// leave. One BPM per detent always -- a spinbox steps by whichever digit its
/// cursor stands on, so the cursor is pinned to the last one.
lv_obj_t* make_bpm(lv_obj_t* parent)
{
    auto* sq = seq();
    const int32_t start = sq ? (int32_t)lroundf(sq->getBpm()) : 120;
    s_bpmSyncing = true;
    lv_obj_t* sp = lv_CreateStandardSpinbox(parent,
                                            (int32_t)crosspad::SequencerLimits::MIN_BPM,
                                            (int32_t)crosspad::SequencerLimits::MAX_BPM,
                                            start > 0 ? start : 120, bpm_cb,
#ifdef SEQ_USE_ENCODER
                                            s_encGroup,
#else
                                            nullptr,
#endif
                                            nullptr, &lv_font_montserrat_14);
    s_bpmSyncing = false;
    lv_obj_set_size(sp, 40, 22);
    lv_obj_set_style_pad_all(sp, 0, 0);
    lv_obj_set_style_radius(sp, 6, 0);
    lv_obj_set_style_bg_color(sp, kPanel, 0);
    lv_obj_set_style_border_width(sp, 0, 0);
    lv_obj_set_style_text_color(sp, kText, 0);
    lv_obj_set_style_text_align(sp, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_bg_opa(sp, LV_OPA_TRANSP, LV_PART_CURSOR);
    lv_obj_set_style_text_color(sp, kText, LV_PART_CURSOR);
    lv_obj_set_style_bg_color(sp, kAccent, LV_STATE_EDITED);
    lv_obj_set_style_text_color(sp, kInk, LV_STATE_EDITED);
    lv_obj_set_style_outline_color(sp, kHead, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(sp, 2, LV_STATE_FOCUS_KEY);
    lv_spinbox_set_cursor_pos(sp, 0);
    lv_spinbox_set_step(sp, 1);
    lv_obj_remove_flag(sp, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(sp, LV_DIR_NONE);
    if (lv_obj_t* l = lv_textarea_get_label(sp)) {
        lv_obj_set_width(l, LV_PCT(100));
        lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(l, LV_ALIGN_CENTER, 0, 0);
    }
    /* Pinning the cursor rewrites the text and sends VALUE_CHANGED, so it is
     * done as a sync: what comes back is ours, not a tempo somebody dialled. */
    auto pin = [](lv_event_t* e) {
        lv_obj_t* s = static_cast<lv_obj_t*>(lv_event_get_target(e));
        const bool was = s_bpmSyncing;
        s_bpmSyncing = true;
        lv_spinbox_set_cursor_pos(s, 0);
        lv_spinbox_set_step(s, 1);
        s_bpmSyncing = was;
    };
    lv_obj_add_event_cb(sp, pin, LV_EVENT_FOCUSED, nullptr);
    lv_obj_add_event_cb(sp, pin, LV_EVENT_CLICKED, nullptr);
    return sp;
}

} // namespace

lv_obj_t* Sequencer_create(lv_obj_t* parent, App* app)
{
#ifdef SEQ_USE_ENCODER
    if (indev_encoderTop) s_prevEncGroup = lv_indev_get_group(indev_encoderTop);
    if (!s_encGroup) s_encGroup = lv_group_create();
    if (indev_encoderTop) lv_indev_set_group(indev_encoderTop, s_encGroup);
#endif

    s_root = lv_obj_create(parent);
    lv_obj_set_size(s_root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_root, kBg, 0);
    lv_obj_set_style_border_width(s_root, 0, 0);
    lv_obj_set_style_radius(s_root, 0, 0);
    lv_obj_set_style_pad_all(s_root, 5, 0);
    lv_obj_set_style_pad_row(s_root, 4, 0);
    lv_obj_set_flex_flow(s_root, LV_FLEX_FLOW_COLUMN);
    lv_obj_remove_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);

    // -- what the scene is counted in --
    lv_obj_t* top = make_row(s_root, 22);
    make_button(top, 30, 22, LV_SYMBOL_LEFT, back_cb, app);
    s_bpm = make_bpm(top);
    if (!s_tempoSub) {
        /* Event-bus task: only remember who drives the tempo; sync_ui() on
         * the LVGL timer shows it. */
        s_tempoSub = crosspad::getEventBus().subscribe(crosspad::EventType::TempoChanged,
            [](const void* data, crosspad::EventType, void*) {
                const auto* d = static_cast<const crosspad::TempoChangedData*>(data);
                s_tempoFromHost.store(d->source == crosspad::EventSource::MidiInput,
                                      std::memory_order_relaxed);
            }, nullptr);
    }
    s_clickBtn = make_button(top, 26, 22, "M", click_cb);
    s_preBtn = make_button(top, 30, 22, "PRE", pre_cb);
    lv_obj_t* barsBtn = make_button(top, 30, 22, "4b", bars_cb);
    s_barsLabel = lv_obj_get_child(barsBtn, 0);
    s_quantBtn = make_button(top, 36, 22, "1/16", quant_cb);
    s_quantLabel = lv_obj_get_child(s_quantBtn, 0);
    make_button(top, 30, 22, "x2", double_cb);
    make_button(top, 30, 22, "/2", halve_cb);

    // -- which scene, and ERASE held beside it --
    lv_obj_t* scenes = make_row(s_root, 22);
    make_button(scenes, 26, 22, LV_SYMBOL_LEFT, scene_prev_cb);
    s_sceneLabel = lv_label_create(scenes);
    lv_obj_set_width(s_sceneLabel, 64);
    lv_obj_set_style_text_align(s_sceneLabel, LV_TEXT_ALIGN_CENTER, 0);
    make_button(scenes, 26, 22, LV_SYMBOL_RIGHT, scene_next_cb);
    make_button(scenes, 26, 22, LV_SYMBOL_MINUS, scene_del_cb);
    make_button(scenes, 26, 22, LV_SYMBOL_PLUS, scene_add_cb);
    lv_obj_t* gap = lv_obj_create(scenes);
    lv_obj_remove_style_all(gap);
    lv_obj_set_flex_grow(gap, 1);
    make_button(scenes, 50, 22, "BOUNCE", bounce_cb);
    s_eraseBtn = make_button(scenes, 60, 22, "ERASE", nullptr);
    lv_obj_add_event_cb(s_eraseBtn, erase_cb, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_event_cb(s_eraseBtn, erase_cb, LV_EVENT_RELEASED, nullptr);
    lv_obj_add_event_cb(s_eraseBtn, erase_cb, LV_EVENT_PRESS_LOST, nullptr);

    // -- the pattern --
    s_grid = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_grid);
    lv_obj_set_width(s_grid, LV_PCT(100));
    lv_obj_set_flex_grow(s_grid, 1);
    lv_obj_set_style_bg_color(s_grid, kPanel, 0);
    lv_obj_set_style_bg_opa(s_grid, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(s_grid, 6, 0);
    lv_obj_set_style_pad_all(s_grid, 3, 0);
    lv_obj_remove_flag(s_grid, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_grid, grid_draw_cb, LV_EVENT_DRAW_MAIN_END, nullptr);

    // -- transport --
    lv_obj_t* tr = make_row(s_root, 28);
    s_recBtn  = make_button(tr, 74, 28, "REC", rec_cb);
    s_dubBtn  = make_button(tr, 74, 28, "DUB", dub_cb);
    s_playBtn = make_button(tr, 74, 28, LV_SYMBOL_PLAY, play_cb);
    s_stopBtn = make_button(tr, 74, 28, LV_SYMBOL_STOP, stop_cb);

    sync_ui();
    s_drawn = pattern_shape();
    s_headX = -1;

#ifdef SEQ_USE_ENCODER
    if (s_encGroup) lv_group_focus_obj(s_playBtn);
#endif

    s_timer = lv_timer_create(tick_cb, kTickMs, nullptr);

    if (app) {
        app->setOnShow([](lv_obj_t*) {
            if (s_timer) lv_timer_resume(s_timer);
#ifdef SEQ_USE_ENCODER
            if (indev_encoderTop && s_encGroup) lv_indev_set_group(indev_encoderTop, s_encGroup);
#endif
        });
        app->setOnHide([](lv_obj_t*) {
            if (s_timer) lv_timer_pause(s_timer);
            /* Erase must never outlive the button holding it. */
            if (auto* sq = seq()) sq->setErasing(false);
#ifdef SEQ_USE_ENCODER
            if (indev_encoderTop && s_prevEncGroup) lv_indev_restore_group(indev_encoderTop, s_prevEncGroup);
#endif
        });
    }

    return s_root;
}

void Sequencer_destroy(lv_obj_t* root)
{
    if (s_timer) {
        lv_timer_delete(s_timer);
        s_timer = nullptr;
    }
    /* A screen closed with erase still held must not leave every later hit
     * erasing instead of playing. The loop itself keeps running, and so does
     * a bounce: it finishes on its own and lands in the song. */
    if (auto* sq = seq()) sq->setErasing(false);
#ifdef SEQ_USE_ENCODER
    if (s_bounceGroup) { lv_group_delete(s_bounceGroup); s_bounceGroup = nullptr; }
#endif
    s_bounceBox = s_bounceText = s_bounceBar = s_bounceBtn = nullptr;
    s_bouncing = false;
    s_root = s_grid = s_bpm = s_clickBtn = s_preBtn = s_barsLabel = nullptr;
    s_quantBtn = s_quantLabel = s_sceneLabel = s_eraseBtn = nullptr;
    s_recBtn = s_dubBtn = s_playBtn = s_stopBtn = nullptr;

#ifdef SEQ_USE_ENCODER
    // Hand navigation back before the group dies, or the launcher comes back
    // unnavigable and only a reboot fixes it.
    if (indev_encoderTop && s_prevEncGroup) lv_indev_restore_group(indev_encoderTop, s_prevEncGroup);
    if (s_encGroup) {
        lv_group_delete(s_encGroup);
        s_encGroup = nullptr;
    }
    s_prevEncGroup = nullptr;
#endif

    lv_obj_delete_async(root);
}

// Auto-registration: the app shows up in the launcher with no wiring anywhere
// else. No pad logic: the pads stay the Sampler's, so they play and record here
// exactly as they do everywhere else.
REGISTER_APP(Sequencer, nullptr, "sequencer.png",
             Sequencer_create, Sequencer_destroy,
             nullptr, nullptr, nullptr, 0)
