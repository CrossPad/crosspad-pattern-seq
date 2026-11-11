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
#include "crosspad/instrument/InstrumentPadLogic.hpp"
#include "crosspad/instrument/InstrumentPlayState.hpp"
#include "crosspad/event/EventData.hpp"
#include "crosspad/event/IEventBus.hpp"
#include "crosspad/instrument/IInstrumentHost.hpp"
#include "crosspad/music/MusicTheory.hpp"
#include "crosspad/pad/PadPerformance.hpp"
#include "crosspad/kit/IKitManager.hpp"
#include "crosspad/kit/KitInfo.hpp"
#include "crosspad/pad/IPadLogicHandler.hpp"
#include "crosspad/pad/PadManager.hpp"
#include "crosspad/platform/PlatformServices.hpp"
#include "crosspad/sampletools/ISampleTools.hpp"
#include "crosspad/sequencer/SequencerController.hpp"
#include "crosspad/transport/SequencerTransport.hpp"
#include "crosspad/transport/SongTransport.hpp"
#include "crosspad/song/ArrangeFocus.hpp"
#include "crosspad/song/ISongEngine.hpp"
#include "crosspad-gui/components/mode_overlay.h"
#include "crosspad-gui/components/transport_bar.h"
#include "crosspad-gui/components/ui_toast.h"
#include "crosspad-gui/widgets/gui_helpers.h"

#include "lvgl.h"
/* lv_image_cache_drop(): lvgl.h stopped exporting the cache in 9.4. */
#include "src/misc/cache/instance/lv_image_cache.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

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

/* ── set_text_fmt() is not used here, and this is why ─────────────
 *
 * It frees the label's old text and allocates the new one, but it does not
 * revert the dots of a truncated label first -- lv_label_set_text() does
 * exactly that, one line in, "in case text == label->text". So when a long
 * line is replaced by a shorter one, LVGL still holds the offset the dots were
 * written at, and on the next refresh it puts the four saved characters back at
 * that offset -- past the end of the new, shorter buffer. The heap block next
 * door loses its tail canary, and the crash lands minutes later on whatever
 * frees that block: an assert deep inside LVGL, with nothing in the backtrace
 * to say who wrote the bytes. It read as "Arrange crashes when I click a
 * block", because this screen is the one that formats a line of changing length
 * on every click.
 *
 * Formatting it ourselves and handing lv_label_set_text() a finished string is
 * both the safe way and what every other screen in this firmware does. */
static void set_text_fmt(lv_obj_t* label, const char* fmt, ...)
{
    if (!label || !fmt) return;
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    lv_label_set_text(label, buf);
}

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
const lv_color_t kGridBg = lv_color_hex(0x1B1E22);   ///< darker than a button
const lv_color_t kWave   = lv_color_hex(0x9CD639);   ///< a note, the export's one colour
const lv_color_t kPlayhead = lv_color_hex(0x00FF88); ///< mint, like the transport bar at rest
constexpr int32_t kGridInset = 6;                    ///< the export's first line, inside the panel
constexpr int32_t kNoteW = 5;                        ///< the export's mark
constexpr int32_t kNoteH = 6;
constexpr int32_t kMinStepPx = 6;                    ///< below this a step grid is a smear
const lv_color_t kHead   = lv_color_hex(0xFFFFFF);
/// The tempo field while a host's MIDI clock drives the tempo.
const lv_color_t kHostClock = lv_color_hex(0x4FC3F7);

lv_obj_t* s_root = nullptr;
lv_obj_t* s_grid = nullptr;
lv_obj_t* s_bpm = nullptr;
lv_obj_t* s_syncBtn = nullptr;
lv_obj_t* s_levelsBtn = nullptr;   ///< the scene window's 16 LEVELS
lv_obj_t* s_clearBtn = nullptr;
bool     s_clearArmed = false;           ///< CLEAR pressed once; the second press clears
uint32_t s_clearArmedAt = 0;
constexpr uint32_t kClearArmMs = 3000;   ///< how long the first press waits for the second
lv_obj_t* s_preBtn = nullptr;
/* The first two soft keys, opened from Arrange or on its own: the way to the
 * arrangement, and the way into the sound the notes play through. */
lv_obj_t* s_arrBtn = nullptr;
lv_obj_t* s_editBtn = nullptr;
lv_obj_t* s_barsLabel = nullptr;
lv_obj_t* s_quantBtn = nullptr;
lv_obj_t* s_quantLabel = nullptr;
lv_obj_t* s_sceneLabel = nullptr;
/* The scene name is a button: the design's two rows are full to 314 px and
 * the screen it was drawn for had fewer controls than this one, so the four
 * that no longer fit -- the tempo, ADD, DEL, SRC and SYNC -- live behind it.
 * A window for a set of settings is this screen's own idiom already (SWING,
 * BOUNCE), so it is not a new one to learn. */
lv_obj_t* s_sceneBtn = nullptr;
lv_obj_t* s_sceneMenu = nullptr;
lv_group_t* s_sceneGroup = nullptr;
lv_obj_t* s_eraseBtn = nullptr;
lv_obj_t* s_recBtn = nullptr;
lv_obj_t* s_dubBtn = nullptr;
lv_obj_t* s_playBtn = nullptr;
lv_obj_t* s_stopBtn = nullptr;
lv_timer_t* s_timer = nullptr;
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

/* The swing window is on every platform; only its encoder group is not. */
lv_obj_t*   s_swingBox  = nullptr;      ///< the swing window, over the screen
lv_obj_t*   s_swingText = nullptr;      ///< what it is set to, in words
lv_obj_t*   s_swingBtn  = nullptr;      ///< SWING on the top row
lv_obj_t*   s_swingVal  = nullptr;      ///< the amount, turned with the encoder
lv_obj_t*   s_swingChip[7] = {};        ///< the sampler's steps, 50 to 75

/* SRC: what the pads play, and the window that picks it. */
lv_obj_t* s_srcBtn   = nullptr;         ///< SRC on the transport row
lv_obj_t* s_srcLabel = nullptr;         ///< its text: KIT, or the instrument
lv_obj_t* s_srcBox   = nullptr;         ///< the picker, over the screen
lv_obj_t* s_srcList  = nullptr;         ///< the scrolling part of it
lv_obj_t* s_srcNoteLbl = nullptr;       ///< a failure or "loading", under the list
lv_obj_t* s_srcOctLbl  = nullptr;       ///< the base note
lv_obj_t* s_srcChordLbl = nullptr;      ///< what a pad makes: one note, or a chord
lv_obj_t* s_srcClose = nullptr;         ///< below the list; the group is
                                        ///< rebuilt around it on every fill,
                                        ///< so it keeps its place in the order
std::vector<std::string> s_srcLib;      ///< instrument folder names
bool s_srcListing = false;              ///< a folder request of ours is out

#ifdef SEQ_USE_ENCODER
lv_group_t* s_encGroup = nullptr;
lv_group_t* s_prevEncGroup = nullptr;
lv_group_t* s_bounceGroup = nullptr;
lv_group_t* s_srcGroup = nullptr;
lv_group_t* s_swingGroup = nullptr;
#endif

crosspad::SequencerController* seq()
{
    return crosspad::getPlatformServices().sequencer;
}

bool running()
{
    auto* sq = seq();
    return sq && sq->getState() == crosspad::SequencerState::Running;
}

// ── what the pads play, and so what a take records ─────────────────────────

/* The Sequencer has no sound of its own: it records whatever the pads make,
 * and a bounce captures the pads bus (SongEngine::runBounce), so whichever of
 * these is playing is what lands on the track. Until now that was always the
 * kit. SRC also offers the instruments Pitch Play saves, so a melodic part can
 * be recorded and bounced here rather than played by hand into the Sampler.
 *
 * The pitched engine and the kits' streamer share one arena, so an instrument
 * is *borrowed* through IInstrumentHost: taking it stops the kits' engine,
 * giving it back reloads the kit. Hence release() on the way out -- leave it
 * borrowed and the Sampler comes back silent. The one exception is a bounce
 * still running when the screen closes: it is capturing the pads, so the
 * engine has to stay until it is done.
 *
 * The pads need a logic of their own in this mode (the Sampler's plays kit
 * pads, not notes), pushed on the way in and popped on the way out. This app
 * registers with plain REGISTER_APP and no padLogicName, so the push is ours
 * to make -- an app declaring one must not. */

constexpr uint8_t kSeqPads = 16;
constexpr int      kSeqChordMax = 6;    ///< the widest voicing MusicTheory builds
constexpr const char* kSrcPadLogic = "SeqPitch";

/* A pad plays a chord, not only its own note. The mapping is
 * crosspad::MusicTheory's -- the same one Pitch Play plays through, so a
 * triad here is the triad there -- over a chromatic scale rooted on the pad
 * range's base: the Sequencer has an octave control and no key of its own,
 * and inventing a second place to set one would be two to keep in step.
 *
 * In PSRAM: PlaySettings is ~150 bytes and this board boots with a couple of
 * hundred to spare. */
#ifdef ESP_PLATFORM
  #include "esp_attr.h"
  #define SEQ_SLOW_BSS EXT_RAM_BSS_ATTR
#else
  #define SEQ_SLOW_BSS
#endif

SEQ_SLOW_BSS crosspad::PlaySettings s_ps;

enum class Src : uint8_t { Kit, Inst };

Src         s_src = Src::Kit;
std::string s_instPath;                  ///< logical path of the chosen instrument
std::string s_instName;                  ///< what SRC and the window show
std::string s_srcNote;                   ///< a failure or a progress line, for the window
uint32_t    s_playGen = 0;               ///< InstrumentPlayState::generation last copied

/* Pitch Play's map, not one of our own.
 *
 * The instrument source *is* Pitch Play's instrument, so a pad here has to
 * make the note Pitch Play says it makes -- its key, scale, transpose and
 * chord included. This used to build a chromatic map from a base note of its
 * own with its own chord index, which meant the same pattern sounded one way
 * under the pads and another way recorded, and two places to set the same
 * thing. Both are gone: core's InstrumentPlayState carries a padMidiMap and
 * padChordNotes that MusicTheory::rebuildPadMap() has already filled, and
 * this copies them.
 *
 * Copied rather than read through: onPadPress runs on the pad task and the
 * publisher is the UI thread, so a snapshot taken here, on the app tick, is
 * what the pads read. */
void seq_rebuild_pads()
{
    s_ps = crosspad::instrumentPlayState().play;
    s_playGen = crosspad::instrumentPlayState().generation;
}

std::atomic<bool> s_instReady{false};    ///< the bank is built and the pads may play it
bool        s_padLogicPushed = false;

crosspad::IInstrumentHost*   inst_host() { return crosspad::getPlatformServices().instrumentHost; }
crosspad::ISampleTools*      tools()     { return crosspad::getPlatformServices().sampleTools; }

void note_name(int note, char* buf, size_t n)
{
    static const char* kNames[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    if (note < 0 || note > 127) { std::snprintf(buf, n, "--"); return; }
    std::snprintf(buf, n, "%s%d", kNames[note % 12], note / 12 - 1);
}

std::shared_ptr<crosspad::InstrumentPadLogic> s_srcPadLogic;   // core owns the behaviour

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

/// The V5 grid, drawn straight into the layer rather than built from objects.
///
/// The export lays out 12 rows x 16 boxes plus 17 line objects -- 209 LVGL
/// objects, tens of kB of the PSRAM the UI lives in on this board, rebuilt on
/// every hit. The look is the export's to the pixel; the way it gets on
/// screen is not, and nothing about that is visible.
///
/// Sixteen lanes, not the mockup's twelve: there are sixteen pads, and four
/// of them would have nowhere to show. ROWH follows from that -- 134 px of
/// grid over 16 lanes is 8, where the export's 12 lanes took 10.
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
    if (w < 32 || h < 16) return;

    const uint8_t scene = sq->getActiveScene();
    const int bars  = sq->getSceneBars(scene) ? sq->getSceneBars(scene) : 1;
    const int beats = bars * 4;

    /* The export's own geometry: the first line 6 px inside the grid, and a
     * step every 18.5 px. It drew 16 steps of a bar; a scene here can be up
     * to eight bars, so the step is the scene's beat and the division stays
     * the export's -- strong on the bar, hairline between. */
    const int32_t x0 = box.x1 + kGridInset;
    const int32_t gw = w - kGridInset * 2;
    const int32_t rowH = h / kLanes;
    if (rowH < 3 || gw < 16) return;

    /* A line per step, strong every fourth, exactly as the export draws it --
     * at one bar that is its sixteen steps and seventeen lines. A longer
     * scene would put them 4 px apart and the grid would read as a smear, so
     * below kMinStepPx the division falls back to the beat. */
    int divs = beats * 4;
    if (divs > 0 && gw / divs < kMinStepPx) divs = beats;
    if (divs < 1) divs = 1;
    const int strongEvery = (divs == beats) ? 4 : 4;   // a bar, or a beat
    for (int b = 0; b <= divs; ++b) {
        const int32_t x = x0 + (int32_t)((int64_t)gw * b / divs);
        const bool strong = (b % strongEvery) == 0;
        fill(layer, x, box.y1, x + (strong ? 1 : 0), box.y1 + h - 1,
             kLine, strong ? LV_OPA_COVER : 150);
    }

    /* Pad 1 at the bottom, the way the pads sit under your hand. */
    auto laneY = [&](int pad) { return box.y1 + (kLanes - 1 - pad) * rowH; };

    const uint16_t n = sq->getSceneEventCount(scene);
    for (uint16_t i = 0; i < n; ++i) {
        float at = 0.0f;
        uint8_t pad = 0, vel = 0;
        if (!sq->getSceneEvent(scene, i, at, pad, vel) || pad >= kLanes) continue;
        /* One colour, one size, like the export: the row says which pad, so
         * the mark only has to say that a hit is there. A note held longer
         * trails a bar as long as it was held, round the loop end if it
         * reaches past it. */
        const int32_t x = x0 + (int32_t)(at * gw);
        const int32_t y = laneY(pad) + (rowH - kNoteH) / 2;
        const int32_t len = (int32_t)(sq->getSceneEventLength(scene, i) * gw);
        if (len > kNoteW) {
            const int32_t end = x + len;
            fill(layer, x, y, std::min(end, x0 + gw) - 1, y + kNoteH - 1, kWave, LV_OPA_50);
            if (end > x0 + gw) fill(layer, x0, y, x0 + (end - x0 - gw) - 1, y + kNoteH - 1, kWave, LV_OPA_50);
        }
        fill(layer, x, y, x + kNoteW - 1, y + kNoteH - 1, kWave, LV_OPA_COVER);
    }

    if (s_headX >= 0) {
        fill(layer, s_headX, box.y1, s_headX + 1, box.y1 + h - 1,
             kPlayhead, LV_OPA_COVER);
    }
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
    const uint16_t n = sq->getSceneEventCount(s);
    /* A length lands at the release, after its note is already drawn. */
    uint32_t held = 0;
    for (uint16_t i = 0; i < n; ++i) held = held * 31u + (uint32_t)(sq->getSceneEventLength(s, i) * 65536.0f);
    return ((uint32_t)s << 24) ^ ((uint32_t)sq->getSceneBars(s) << 16) ^
           n ^ ((uint32_t)sq->getSceneCount() << 20) ^ held;
}

// ── state on the buttons ───────────────────────────────────────────────────

/* Only what changed: setting a style, even to the value it has, refreshes the
 * object and redraws it, and sync_ui() runs every tick. */
void paint(lv_obj_t* b, lv_color_t bg, bool dark)
{
    if (!b) return;
    if (!lv_color_eq(lv_obj_get_style_bg_color(b, LV_PART_MAIN), bg)) lv_obj_set_style_bg_color(b, bg, 0);
    lv_obj_t* l = lv_obj_get_child(b, 0);
    const lv_color_t ink = dark ? kInk : kText;
    if (l && !lv_color_eq(lv_obj_get_style_text_color(l, LV_PART_MAIN), ink)) lv_obj_set_style_text_color(l, ink, 0);
}

void set_text(lv_obj_t* l, const char* text)
{
    if (l && std::strcmp(lv_label_get_text(l), text) != 0) lv_label_set_text(l, text);
}

/* ── an Arrange track's seq: is this screen one, and in what colour ───────
 * Opened from Arrange, this screen is one track of a song; opened on its own
 * it is the groove sequencer it has always been. The actions that go with it
 * are further down, by scene_step(). */
bool in_track()
{
    const crosspad::ArrangeFocus& f = crosspad::arrangeFocus();
    return f.held() && f.seq;
}

lv_color_t track_ink()
{
    return crosspad::arrangeFocus().source == crosspad::SeqSource::Pitch
        ? lv_color_hex(0xA77BF0)      // Pitch Play's purple
        : kAmber;                     // the Sampler's amber
}

crosspad::ISongEngine* songs();

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
    /* REC and DUB moved to the transport bar, so these are null here now and
     * only the count-in above still writes to one when a screen has them. */
    if (s_recBtn && waiting != s_recBtn) set_text(lv_obj_get_child(s_recBtn, 0), "REC");
    if (s_dubBtn && waiting != s_dubBtn) set_text(lv_obj_get_child(s_dubBtn, 0), "DUB");
    paint(s_preBtn, sq->getCountIn() ? kAmber : kPanel, sq->getCountIn());
    /* The amount on the button: a feel you cannot see the value of is one you
     * cannot put back where it was. */
    if (s_swingBtn) {
        const uint8_t sw = sq->getSwing();
        paint(s_swingBtn, sw ? kAmber : kPanel, sw != 0);
        /* The amount always on the key, the way the export writes it: 78 px
         * holds "SWING 100%" in font 10, and a feel you cannot see the value
         * of is one you cannot put back where it was. */
        char sb[16];
        std::snprintf(sb, sizeof(sb), "SWING %u%%", (unsigned)sw);
        set_text(lv_obj_get_child(s_swingBtn, 0), sb);
    }
    paint(s_playBtn, (run && !rec && !dub) ? kPlay : kPanel, run && !rec && !dub);
    paint(s_stopBtn, kPanel, false);
    /* CLEAR asked once: red, and saying what the second press does, until it
     * is pressed again or the moment passes. */
    if (s_clearArmed && lv_tick_elaps(s_clearArmedAt) > kClearArmMs) s_clearArmed = false;
    if (s_clearBtn) {
        paint(s_clearBtn, s_clearArmed ? kRec : kPanel, s_clearArmed);
        set_text(lv_obj_get_child(s_clearBtn, 0), s_clearArmed ? "SURE?" : "CLEAR");
    }
    if (auto* se = crosspad::getPlatformServices().songEngine) {
        bool sync = false;
        {
            std::lock_guard<std::recursive_mutex> l(se->songMutex());
            sync = se->song().syncSeq;
        }
        paint(s_syncBtn, sync ? kAccent : kPanel, sync);
    }
    if (!sq->isErasing()) paint(s_eraseBtn, kPanel, false);

    const bool off = sq->getQuantization() == crosspad::Quantization::Off;
    paint(s_quantBtn, off ? kAmber : kPanel, off);
    const char* q = "1/16";
    switch (sq->getQuantization()) {
        case crosspad::Quantization::Quarter:          q = "1/4";   break;
        case crosspad::Quantization::Eighth:           q = "1/8";   break;
        case crosspad::Quantization::EighthTriplet:    q = "1/8T";  break;
        case crosspad::Quantization::SixteenthTriplet: q = "1/16T"; break;
        case crosspad::Quantization::ThirtySecond:     q = "1/32";  break;
        case crosspad::Quantization::Off:              q = "free";  break;
        default: break;
    }
    set_text(s_quantLabel, q);

    const uint8_t active = sq->getActiveScene();
    const uint8_t pending = sq->getPendingScene();
    const uint8_t bars = sq->getSceneBars(active) ? sq->getSceneBars(active) : 1;
    std::snprintf(buf, sizeof(buf), "%ub", (unsigned)bars);
    set_text(s_barsLabel, buf);

    /* In an Arrange track's context the name is the track, because that is
     * what the player is working on -- which scene it plays is the track's
     * business and shows in Arrange's lane. The colour is the sound it plays
     * through, the same amber or purple the track's badge carries there. */
    if (in_track()) {
        /* Which block of the track: the bar it starts on. */
        const auto* se = crosspad::getPlatformServices().songEngine;
        const int32_t at = se ? se->seqEditBar() : -1;
        const uint32_t bar = at >= 0 ? (uint32_t)at : crosspad::arrangeFocus().bar;
        std::snprintf(buf, sizeof(buf), "T%u B%u", (unsigned)(crosspad::arrangeFocus().track + 1), (unsigned)(bar + 1));
        set_text(s_sceneLabel, buf);
        const lv_color_t ink = track_ink();
        if (!lv_color_eq(lv_obj_get_style_text_color(s_sceneLabel, LV_PART_MAIN), ink)) {
            lv_obj_set_style_text_color(s_sceneLabel, ink, 0);
        }
    } else
    /* The scene you asked for, amber until the loop end hands it over. */
    if (pending != active) {
        /* No spaces around the separators, as the design writes it. It also
         * bounds the label: eight scenes is the most there can be, so the
         * widest this gets is "Seq 8>8" -- seven characters, which montserrat
         * 14 draws in about 49 px of the 56 the design gives the name. With
         * the spaces it was nine and overflowed the chip into the arrows. */
        std::snprintf(buf, sizeof(buf), "Seq %u>%u", (unsigned)(active + 1), (unsigned)(pending + 1));
    } else {
        std::snprintf(buf, sizeof(buf), "Seq %u/%u", (unsigned)(active + 1), (unsigned)sq->getSceneCount());
    }
    if (!in_track()) {
        set_text(s_sceneLabel, buf);
        const lv_color_t sceneInk = pending != active ? kAmber : kText;
        if (!lv_color_eq(lv_obj_get_style_text_color(s_sceneLabel, LV_PART_MAIN), sceneInk)) {
            lv_obj_set_style_text_color(s_sceneLabel, sceneInk, 0);
        }
    }

    if (s_bpm) {
        std::snprintf(buf, sizeof(buf), "%03d", (int)lroundf(sq->getBpm()));
        set_text(lv_obj_get_child(s_bpm, 0), buf);
        /* Blue while a host's MIDI clock drives the tempo. */
        const lv_color_t field = s_tempoFromHost.load(std::memory_order_relaxed) ? kHostClock : kPanel;
        if (!lv_color_eq(lv_obj_get_style_bg_color(s_bpm, LV_PART_MAIN), field)) {
            lv_obj_set_style_bg_color(s_bpm, field, 0);
        }
    }
}

void bounce_progress();
void src_poll();

void tick_cb(lv_timer_t*)
{
    /* What changed in the scene goes back to the block it is. */
    if (in_track()) {
        if (auto* se = crosspad::getPlatformServices().songEngine) se->syncSeqEdit();
    }
    if (s_bouncing) bounce_progress();
    src_poll();
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

/// The tempo goes to the clock at once -- the pattern is rescaled with it, so
/// a take keeps its place in the bar -- and sequences.json carries it with
/// the scenes. It is the groove's, not the kit's: another kit plays it as is.
void bpm_cb(lv_event_t* e)
{
    auto* sq = seq();
    lv_obj_t* b = static_cast<lv_obj_t*>(lv_event_get_current_target(e));
    if (!sq || !b) return;
    lv_group_t* g = lv_obj_get_group(b);
    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        /* The encoder's second click, or a tap: leave edit mode, or enter it. */
        if (g) {
            const bool editing = lv_group_get_editing(g) && lv_group_get_focused(g) == b;
            lv_group_focus_obj(b);
            lv_group_set_editing(g, !editing);
        }
        return;
    }
    if (code == LV_EVENT_FOCUSED || code == LV_EVENT_DEFOCUSED) {
        /* Let go of (a click, a long press, focus moving on): the tempo is the
         * instrument's, and the open song takes it now. Nothing to do when
         * nothing was turned. */
        if (!(g && lv_group_get_editing(g) && lv_group_get_focused(g) == b)) crosspad::transportCommitTempo();
        return;
    }
    const uint32_t key = lv_event_get_key(e);
    const int step = key == LV_KEY_RIGHT || key == LV_KEY_UP ? 1 : (key == LV_KEY_LEFT || key == LV_KEY_DOWN ? -1 : 0);
    /* The DAW's clock owns the tempo while it drives: a turn changes
     * nothing, and sequences.json is left alone. */
    if (!step || s_tempoFromHost.load(std::memory_order_relaxed)) return;
    const int32_t v = std::max<int32_t>((int32_t)crosspad::SequencerLimits::MIN_BPM,
                      std::min<int32_t>((int32_t)crosspad::SequencerLimits::MAX_BPM,
                                        (int32_t)lroundf(sq->getBpm()) + step));
    crosspad::transportSetTempo((float)v);
    sync_ui();
}

/// A bar of click before a take started from a stop, so the first hit lands
/// on beat one instead of on the button press.
void pre_cb(lv_event_t*)
{
    if (auto* sq = seq()) sq->setCountIn(!sq->getCountIn());
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

/* The design gives the length a - and a + of its own beside the readout,
 * which beats one button that cycles 1-2-4-8 and wraps: a shorter scene is
 * one press back, not three forward. */
void len_dn_cb(lv_event_t*)
{
    auto* sq = seq();
    if (!sq) return;
    const uint8_t sc = sq->getActiveScene();
    const uint8_t bars = sq->getSceneBars(sc);
    if (bars > 1) sq->setSceneBars(sc, (uint8_t)(bars / 2));
    sync_ui();
}

void len_up_cb(lv_event_t*)
{
    auto* sq = seq();
    if (!sq) return;
    const uint8_t sc = sq->getActiveScene();
    const uint8_t bars = sq->getSceneBars(sc);
    if (bars < 8) sq->setSceneBars(sc, (uint8_t)(bars ? bars * 2 : 1));
    sync_ui();
}

/// 1/4, 1/8, 1/8T, 1/16, 1/16T, 1/32, free, and round again. The triplets sit
/// where they belong by density -- three to the beat is between two and four --
/// so one way round the button is coarse to fine and nothing jumps back. Free
/// stands one press past the finest, because a part with feel in it is ruined by
/// the same snap that flatters most playing.
void quant_cb(lv_event_t*)
{
    auto* sq = seq();
    if (!sq) return;
    using Q = crosspad::Quantization;
    static const Q kRound[] = {Q::Quarter, Q::Eighth, Q::EighthTriplet, Q::Sixteenth,
                               Q::SixteenthTriplet, Q::ThirtySecond, Q::Off};
    constexpr size_t kCount = sizeof(kRound) / sizeof(kRound[0]);
    const Q now = sq->getQuantization();
    size_t i = 0;
    while (i < kCount && kRound[i] != now) ++i;
    sq->setQuantization(kRound[i + 1 < kCount ? i + 1 : 0]);   // nothing known: back to 1/4
    sync_ui();
}

void double_cb(lv_event_t*) { if (auto* sq = seq()) sq->doubleScene(sq->getActiveScene()); sync_ui(); }

/// CLEAR: every note of the scene on show goes; the scene stays, with its
/// length and its feel. Asked twice, as DEL is in the pad editor -- a whole
/// pattern is too much to lose to one stray press.
void clear_cb(lv_event_t*)
{
    auto* sq = seq();
    if (!sq) return;
    if (!s_clearArmed) {
        s_clearArmed = true;
        s_clearArmedAt = lv_tick_get();
    } else {
        s_clearArmed = false;
        sq->eraseSeq((int8_t)sq->getActiveScene());
    }
    sync_ui();
}
void halve_cb(lv_event_t*)  { if (auto* sq = seq()) sq->halveScene(sq->getActiveScene()); sync_ui(); }

/* ── an Arrange track's seq ────────────────────────────────────────────────
 * Opened from Arrange (its SEQ card, or CLIP EDIT on a seq block), this
 * screen is one block of one track of a song: the name says which track and
 * which bar the block starts on, the first two keys are the way back to the
 * arrangement and the way into the sound it plays. The block's notes are put
 * in the track's own scene and looped there on their own; what is recorded,
 * erased, doubled or cleared goes back to the block as it changes
 * (ISongEngine::syncSeqEdit), and Arrange draws it from there. On a bar with
 * no block, the first notes made here become one. The scene is the block's,
 * so it is not stepped or added to from here.
 *
 * With nothing handed over, none of this applies and the screen is the groove
 * sequencer it has always been. */
/// The block this screen edits, into the track's scene. One too long for a
/// scene is left to Arrange, and the screen is the groove sequencer instead.
void open_track_block()
{
    if (!in_track()) return;
    auto* se = songs();
    if (!se) return;
    crosspad::ArrangeFocus& f = crosspad::arrangeFocus();
    if (se->openSeqEdit((uint8_t)f.track, f.bar, f.scene)) return;
    ui_toast_show("That block is longer than a scene holds (8 bars, 192 notes): DUB or cut it in Arrange.",
                  3500, UI_TOAST_INFO);
    f.clear();
}

/// Opened from the MODE map (or anywhere but Arrange) on a song with a seq
/// track that plays the kit: this screen is that track's block under
/// Arrange's playhead, as CLIP EDIT would have opened it -- or the block the
/// first notes make there. On its own it would record into whatever scene was
/// up, and Arrange would show the track empty and play nothing. A
/// song with no such track leaves the groove sequencer.
void adopt_song_track()
{
    if (in_track()) return;
    auto* se = songs();
    auto* sq = seq();
    if (!se || !sq) return;
    crosspad::ArrangeFocus& f = crosspad::arrangeFocus();
    bool moved = false;
    {
        std::lock_guard<std::recursive_mutex> l(se->songMutex());
        auto& song = se->song();
        /* The armed one: what REC in Arrange takes onto, of up to two kits. */
        const int track = song.armedSeqTrack();
        if (track < 0 || song.tracks[track].source != crosspad::SeqSource::Kit) return;
        auto& t = song.tracks[track];
        /* A scene the sequencer does not have: one of its own again, as
         * Arrange does before it hands a track over. */
        if (t.scene >= sq->getSceneCount()) {
            /* The one added, not the active one: while the transport runs
             * that is still another track's until the loop ends. */
            const bool added = sq->addScene();
            const int scene = song.sceneForNewSeq(sq->getSceneCount(), added);
            if (scene < 0) return;
            t.scene = (uint8_t)scene;
            moved = true;
        }
        f.clear();
        f.track  = track;
        f.seq    = true;
        f.source = t.source;
        f.scene  = t.scene;
        f.bar    = crosspad::songTransport().startBar();
    }
    if (moved) se->commit();
    sq->selectScene(f.scene);
}

/// Back to the arrangement. The track is left as it is -- its notes are in
/// the scene, which is where they live -- and nothing is handed over any
/// more; opened again, the screen finds the song's track itself
/// (adopt_song_track()).
void arr_back_cb(lv_event_t*)
{
    if (auto* se = songs()) se->closeSeqEdit();
    crosspad::arrangeFocus().clear();
    /* No Arrange in this firmware: the bar is refreshed anyway, so the keys
     * paint from a transport that nobody handed over. */
    if (!crosspad_gui::mode_overlay_open_app("Arrange")) crosspad_gui::transportbar_refresh();
}

/// EDIT: the sound this track's notes play through, in the app that makes it.
/// The focus is kept, so that screen knows it was reached from a track.
/// BOUNCE on a track: the question is asked over the arrangement it changes
/// (UX 2000 section 7, screen 6), so Arrange is opened to ask it.
void track_bounce_cb(lv_event_t*)
{
    if (!in_track()) return;
    crosspad::arrangeFocus().bounce = true;
    if (!crosspad_gui::mode_overlay_open_app("Arrange")) crosspad::arrangeFocus().clear();
}

void edit_src_cb(lv_event_t*)
{
    /* A kit track's kit is the Sampler's once the kits' engine holds it
     * (Arrange gives it the engine as CLIP EDIT opens here): before, the
     * Sampler would edit the track's neighbour's. */
    if (in_track() && crosspad::arrangeFocus().source == crosspad::SeqSource::Kit) {
        auto* se = songs();
        std::string kit;
        bool live = false;
        if (se) {
            std::lock_guard<std::recursive_mutex> l(se->songMutex());
            const auto& song = se->song();
            const int t = crosspad::arrangeFocus().track;
            live = song.liveKit == t;
            kit = song.seqRoute(t) == 0 ? song.tracks[t].kit : std::string();
        }
        crosspad::IKitManager* km = crosspad::getKitManager();
        const crosspad::KitInfo* cur = km ? km->getCurrentKit() : nullptr;
        if (!live || (!kit.empty() && (!cur || cur->path != kit))) {
            ui_toast_show("The track's kit is still loading -- EDIT again in a moment.", 3000, UI_TOAST_INFO);
            return;
        }
    }
    /* A track plays what Arrange's "plays with" card said; on its own, this
     * screen plays its own source -- the kit, or the instrument it borrowed. */
    const bool pitch = in_track() ? crosspad::arrangeFocus().source == crosspad::SeqSource::Pitch
                                  : s_src == Src::Inst;
    crosspad_gui::mode_overlay_open_app(pitch ? "PitchPlay" : "Sampler");
}

void scene_step(int d)
{
    auto* sq = seq();
    if (!sq || in_track()) return;
    const uint8_t n = sq->getSceneCount();
    if (!n) return;
    const uint8_t next = (uint8_t)((sq->getPendingScene() + n + d) % n);
    sq->selectScene(next);
    sync_ui();
}
void scene_prev_cb(lv_event_t*) { scene_step(-1); }
void scene_next_cb(lv_event_t*) { scene_step(+1); }

/// Minus drops the scene on show, plus appends an empty one and goes to it.
/// The last scene never goes -- a kit has to have somewhere to record.
void scene_del_cb(lv_event_t*) { if (auto* sq = seq(); sq && !in_track()) sq->removeScene(sq->getActiveScene()); sync_ui(); }
void scene_add_cb(lv_event_t*) { if (auto* sq = seq(); sq && !in_track()) sq->addScene(); sync_ui(); }

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

// ── transport, alone or with the song ──────────────────────────────────────

crosspad::ISongEngine* songs() { return crosspad::getPlatformServices().songEngine; }


void sync_cb(lv_event_t*)
{
    auto* se = songs();
    if (!se) return;
    {
        std::lock_guard<std::recursive_mutex> l(se->songMutex());
        se->song().syncSeq = !se->song().syncSeq;
    }
    se->commit();
    sync_ui();
}


/* PLAY, STOP, REC and DUB used to be four buttons on this screen, with the
 * song-synced start, the counted-in take, the second REC that closes a take
 * and the stop that takes the song with it. They are in the transport bar
 * now, and those four rules went with them into
 * crosspad::SequencerTransport -- not dropped. start_with_song() went too.
 */

// ── bounce ─────────────────────────────────────────────────────────────────

void bounce_close()
{
    if (!s_bounceBox) return;
#ifdef SEQ_USE_ENCODER
    if (indev_encoderTop && s_encGroup) lv_indev_set_group(indev_encoderTop, s_encGroup);
    lv_group_delete_deferred(s_bounceGroup);          // its OK/Cancel click is still being read
    s_bounceGroup = nullptr;
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

/* Both window makers hang callbacks that are written below them, and they use
 * the row helper the main screen is built with. */
void swing_val_cb(lv_event_t* e);
void swing_set_cb(lv_event_t* e);
lv_obj_t* make_row(lv_obj_t* parent, int32_t h);

/// The amount, as a button the encoder turns: the idiom the tempo uses, so the
/// gesture is the one already learned on the screen behind.
lv_obj_t* swing_value(lv_obj_t* parent)
{
    lv_obj_t* b = lv_CreateEditableButton(parent);
    lv_obj_set_size(b, 56, 26);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, kPanel, 0);
    lv_obj_set_style_bg_color(b, kAccent, LV_STATE_EDITED);
    lv_obj_set_style_text_color(b, kText, 0);
    lv_obj_set_style_text_color(b, kInk, LV_STATE_EDITED);
    lv_obj_set_style_outline_color(b, kHead, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(b, 2, LV_STATE_FOCUS_KEY);
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
    lv_obj_add_flag(l, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_center(l);
    lv_obj_add_event_cb(b, swing_val_cb, LV_EVENT_KEY, nullptr);
    lv_obj_add_event_cb(b, swing_val_cb, LV_EVENT_CLICKED, nullptr);
#ifdef SEQ_USE_ENCODER
    if (s_swingGroup) lv_group_add_obj(s_swingGroup, b);
#endif
    return b;
}

/// One of the sampler's steps: a chip in a row, not a row of its own -- seven
/// of them would not fit the screen and a step is one number.
lv_obj_t* swing_chip(lv_obj_t* parent, const char* text, void* intensity)
{
    lv_obj_t* b = lv_button_create(parent);
    lv_obj_set_size(b, 40, 24);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_set_style_radius(b, 5, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, kPanel, 0);
    lv_obj_set_style_outline_color(b, kHead, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(b, 2, LV_STATE_FOCUS_KEY);
    lv_obj_add_event_cb(b, swing_set_cb, LV_EVENT_CLICKED, intensity);
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(l, kText, 0);
    lv_label_set_text(l, text);
    lv_obj_center(l);
#ifdef SEQ_USE_ENCODER
    if (s_swingGroup) lv_group_add_obj(s_swingGroup, b);
#endif
    return b;
}

/// A row of the swing window. Its own maker rather than the bounce one, which
/// adds itself to that window's group.
lv_obj_t* swing_button(lv_obj_t* parent, const char* text, lv_event_cb_t cb, void* ud)
{
    lv_obj_t* b = lv_button_create(parent);
    lv_obj_set_size(b, LV_PCT(100), 24);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, kPanel, 0);
    lv_obj_set_style_outline_color(b, kHead, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(b, 2, LV_STATE_FOCUS_KEY);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(l, kText, 0);
    lv_label_set_text(l, text);
    lv_obj_center(l);
#ifdef SEQ_USE_ENCODER
    if (s_swingGroup) lv_group_add_obj(s_swingGroup, b);
#endif
    return b;
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
    set_text_fmt(s_bounceText, "Bouncing Seq %u to track %u...",
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
        set_text_fmt(s_bounceText, "Done: %s is on its track in Arrange.%s%s", st.file.c_str(),
                              st.error.empty() ? "" : "\nNote: ", st.error.c_str());
    } else {
        set_text_fmt(s_bounceText, "Bounce failed: %s", st.error.c_str());
    }
    lv_label_set_text(lv_obj_get_child(s_bounceBtn, 0), "OK");
}

// ── 16 LEVELS: the Sampler's, from here ────────────────────────────────────

/* So a part can be recorded in it: one pad of the kit at sixteen loudnesses or
 * sixteen semitones. Off, velocity, pitch, off -- the way the Sampler's own
 * button goes round, and the same mode: the Sampler's pads are the ones this
 * screen plays. A take keeps what each hit sounded as (PadPerformance), so the
 * part plays the same once the mode is off again. Only with the kit as the
 * source -- an instrument's pads are notes already. */
crosspad::IPadPerformance* pad_performance()
{
    return crosspad::getPlatformServices().padPerformance;
}

void levels_sync()
{
    if (!s_levelsBtn) return;
    auto* perf = pad_performance();
    const bool usable = perf && s_src == Src::Kit;
    const crosspad::PadLevels lv = usable ? perf->levels() : crosspad::PadLevels::Off;
    char buf[24];
    switch (lv) {
        case crosspad::PadLevels::Velocity:
            std::snprintf(buf, sizeof(buf), "16 VEL  PAD %u", (unsigned)perf->levelsSource() + 1);
            break;
        case crosspad::PadLevels::Pitch:
            std::snprintf(buf, sizeof(buf), "16 PITCH  PAD %u", (unsigned)perf->levelsSource() + 1);
            break;
        default:
            std::snprintf(buf, sizeof(buf), "16 LEVELS");
            break;
    }
    set_text(lv_obj_get_child(s_levelsBtn, 0), buf);
    paint(s_levelsBtn, lv != crosspad::PadLevels::Off ? kAccent : kPanel, lv != crosspad::PadLevels::Off);
    lv_obj_set_style_opa(s_levelsBtn, usable ? LV_OPA_COVER : LV_OPA_40, 0);
}

void levels_cb(lv_event_t*)
{
    auto* perf = pad_performance();
    if (!perf || s_src != Src::Kit) return;
    using L = crosspad::PadLevels;
    const L now = perf->levels();
    perf->setLevels(now == L::Off ? L::Velocity : (now == L::Velocity ? L::Pitch : L::Off));
    levels_sync();
}

// ── SRC: the kit, or an instrument borrowed from Pitch Play ────────────────

void src_sync()
{
    levels_sync();             // usable only while the pads play the kit
    if (!s_srcLabel) return;
    if (s_src == Src::Kit) { set_text(s_srcLabel, "KIT"); return; }
    if (!s_instReady.load()) { set_text(s_srcLabel, "..."); return; }
    /* Eight characters of font 10 fit the 48 px button; a longer name is cut
     * with a dot, so a cut reads as one rather than as a misspelling. */
    char buf[16];
    if (s_instName.size() <= 8) std::snprintf(buf, sizeof(buf), "%s", s_instName.c_str());
    else                        std::snprintf(buf, sizeof(buf), "%.7s.", s_instName.c_str());
    set_text(s_srcLabel, buf);
}

/// The pads stop playing the kit and start playing notes. Idempotent.
void src_take_pads()
{
    if (s_padLogicPushed) return;
    auto& pm = crosspad::getPadManager();
    if (!s_srcPadLogic) {
        s_srcPadLogic = std::make_shared<crosspad::InstrumentPadLogic>();
        pm.registerPadLogic(kSrcPadLogic, s_srcPadLogic);
    }
    pm.pushPadLogic(kSrcPadLogic);
    s_padLogicPushed = true;
}

/// And back. Pop before anything unregisters the handler, or the pop lands on
/// a name that no longer resolves and leaves the pads with no logic at all.
void src_drop_pads()
{
    if (!s_padLogicPushed) return;
    crosspad::getPadManager().popPadLogic();
    s_padLogicPushed = false;
}

void src_fill();

/// Give the engine back and let the kit load again.
void src_to_kit()
{
    src_drop_pads();
    s_instReady.store(false);
    if (s_srcPadLogic) s_srcPadLogic->setReady(false);
    s_src = Src::Kit;
    s_instPath.clear();
    s_instName.clear();
    s_srcNote.clear();
    if (auto* h = inst_host()) h->release();
    src_sync();
    src_fill();
}

/// Borrow the engine for @p path. The load runs on the host's worker; the
/// pads stay quiet until src_poll() sees Ready.
void src_to_inst(const std::string& path, const std::string& name)
{
    auto* h = inst_host();
    if (!h) { s_srcNote = "No instrument engine on this build."; src_fill(); return; }
    if (!h->requestOpen(path)) { s_srcNote = "Busy -- try again."; src_fill(); return; }
    s_src = Src::Inst;
    s_instPath = path;
    s_instName = name;
    s_instReady.store(false);
    if (s_srcPadLogic) s_srcPadLogic->setReady(false);
    s_srcNote = "Loading " + name + "...";
    src_take_pads();          // before Ready: a hit now is silent, not a kit pad
    src_sync();
    src_fill();
}

/// Host and listing, from the app tick.
void src_poll()
{
    /* Pitch Play gives the engine back to the kits whenever it closes --
     * every time, whatever else was playing through it. Leaving it from this
     * screen's EDIT therefore pulled the instrument out from under a pattern
     * that was playing it, and the next bar came out of the kit's pads
     * instead. The source is this screen's, so this screen takes it back: on
     * the edge where the bank goes away, and once, so a load that cannot
     * succeed is not retried every tick. */
    if (s_src == Src::Inst && !s_instPath.empty()) {
        auto* h = inst_host();
        const bool gone = h && h->state() == crosspad::IInstrumentHost::State::Idle;
        static bool wasGone = false;
        if (gone && !wasGone) {
            s_instReady.store(false);
            if (s_srcPadLogic) s_srcPadLogic->setReady(false);
            if (h->requestOpen(s_instPath)) s_srcNote = "Taking " + s_instName + " back ...";
        }
        wasGone = gone;
    }

    /* Pitch Play may have been opened and changed while this app sat behind
     * it, so the map is re-read whenever the published generation moves --
     * not only when this window is open, because the pads play from it. */
    if (crosspad::instrumentPlayState().generation != s_playGen) {
        seq_rebuild_pads();
        if (s_srcBox) src_fill();
    }

    if (s_srcListing) {
        auto* st = tools();
        if (st) {
            const crosspad::ToolJob job = st->job();
            if (job == crosspad::ToolJob::Ready || job == crosspad::ToolJob::Failed) {
                s_srcLib.clear();
                if (job == crosspad::ToolJob::Ready) {
                    const auto f = st->folder();
                    for (size_t i = 0; i < f.folders; ++i) s_srcLib.push_back(f.name(i));
                }
                s_srcListing = false;
                src_fill();
            }
        } else {
            s_srcListing = false;
        }
    }

    if (s_src != Src::Inst) return;
    auto* h = inst_host();
    if (!h) return;
    const auto state = h->state();
    if (state == crosspad::IInstrumentHost::State::Ready) {
        if (s_srcPadLogic) s_srcPadLogic->setReady(true);
        if (!s_instReady.exchange(true)) {
            const std::string msg = h->message();
            s_srcNote = msg.empty() ? std::string() : msg;
            src_sync();
            src_fill();
        }
    } else if (state == crosspad::IInstrumentHost::State::Failed) {
        const std::string why = h->message();
        src_to_kit();
        s_srcNote = why.empty() ? std::string("Could not load that instrument.") : why;
        src_fill();
    }
}

void src_close()
{
    if (!s_srcBox) return;
#ifdef SEQ_USE_ENCODER
    /* KIT is a row of the scene window, which stays open under the picker:
     * the encoder goes back to it, not to the screen hidden beneath both --
     * a blind click there could reach CLEAR. */
    lv_group_t* back = (s_sceneMenu && s_sceneGroup) ? s_sceneGroup : s_encGroup;
    if (indev_encoderTop && back) lv_indev_set_group(indev_encoderTop, back);
    lv_group_delete_deferred(s_srcGroup);   // the click that closed it is still being read
    s_srcGroup = nullptr;
#endif
    lv_obj_delete_async(s_srcBox);
    s_srcBox = s_srcList = s_srcNoteLbl = s_srcOctLbl = s_srcChordLbl = nullptr;
    s_srcClose = nullptr;
    sync_ui();
}

void src_close_cb(lv_event_t*) { src_close(); }

void src_pick_cb(lv_event_t* e)
{
    const auto idx = (size_t)(uintptr_t)lv_event_get_user_data(e);
    if (idx == 0) { src_to_kit(); return; }
    const size_t i = idx - 1;
    if (i >= s_srcLib.size()) return;
    auto* h = inst_host();
    if (!h) return;
    const std::string name = s_srcLib[i];
    src_to_inst(h->libraryRoot() + "/" + name + "/" + crosspad::IInstrumentHost::kFileName, name);
}



lv_obj_t* src_button(lv_obj_t* parent, const char* text, lv_event_cb_t cb, void* ud, bool on)
{
    lv_obj_t* b = lv_button_create(parent);
    lv_obj_set_size(b, LV_PCT(100), 24);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, on ? kAccent : kPanel, 0);
    lv_obj_set_style_outline_color(b, kHead, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(b, 2, LV_STATE_FOCUS_KEY);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(l, on ? kInk : kText, 0);
    lv_label_set_text(l, text);
    lv_obj_center(l);
#ifdef SEQ_USE_ENCODER
    if (s_srcGroup) lv_group_add_obj(s_srcGroup, b);
#endif
    return b;
}

/// Rebuild the window's contents. Safe when it is closed -- the source can
/// change from the poll, and then there is only the SRC label to update.
void src_fill()
{
    src_sync();
    if (!s_srcList) return;
    /* Only the list is rebuilt. Deleting its buttons takes them out of the
     * group by itself, so nothing else has to be removed -- an earlier
     * lv_group_remove_all_objs() here took OCT and Close out with them and
     * the encoder could never reach them again. */
    lv_obj_clean(s_srcList);

    src_button(s_srcList, "KIT -- the loaded kit", src_pick_cb, (void*)(uintptr_t)0,
               s_src == Src::Kit);
    for (size_t i = 0; i < s_srcLib.size(); ++i) {
        char buf[40];
        std::snprintf(buf, sizeof(buf), LV_SYMBOL_KEYBOARD " %.32s", s_srcLib[i].c_str());
        src_button(s_srcList, buf, src_pick_cb, (void*)(uintptr_t)(i + 1),
                   s_src == Src::Inst && s_instName == s_srcLib[i]);
    }
    if (s_srcLib.empty() && !s_srcListing) {
        lv_obj_t* l = lv_label_create(s_srcList);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(l, kDim, 0);
        lv_obj_set_width(l, LV_PCT(100));
        lv_label_set_long_mode(l, LV_LABEL_LONG_WRAP);
        lv_label_set_text(l, "No instruments yet -- make one in Pitch Play and SAVE it.");
    }

    if (s_srcOctLbl) {
        char n0[8], n1[8], buf[32];
        note_name(s_ps.padMidiMap[0], n0, sizeof(n0));
        note_name(s_ps.padMidiMap[kSeqPads - 1], n1, sizeof(n1));
        std::snprintf(buf, sizeof(buf), "%s - %s", n0, n1);
        set_text(s_srcOctLbl, buf);
    }
    if (s_srcChordLbl) {
        const std::string& cn = crosspad::instrumentPlayState().chordName;
        set_text(s_srcChordLbl, cn.empty() ? "--" : cn.c_str());
    }
    if (s_srcNoteLbl) set_text(s_srcNoteLbl, s_srcNote.c_str());

#ifdef SEQ_USE_ENCODER
    /* The list's new buttons were appended to the group; move the three
     * below it back behind them so turning the encoder still walks the
     * window top to bottom. */
    if (s_srcGroup) {
        for (lv_obj_t* o : {s_srcClose}) {
            if (!o) continue;
            lv_group_remove_obj(o);
            lv_group_add_obj(s_srcGroup, o);
        }
    }
#endif
}

void src_cb(lv_event_t*)
{
    if (s_srcBox) { src_close(); return; }

    s_srcBox = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_srcBox);
    lv_obj_set_size(s_srcBox, LV_PCT(100), LV_PCT(100));
    lv_obj_add_flag(s_srcBox, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_style_bg_color(s_srcBox, kBg, 0);
    lv_obj_set_style_bg_opa(s_srcBox, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_srcBox, 8, 0);
    lv_obj_set_style_pad_row(s_srcBox, 4, 0);
    lv_obj_set_flex_flow(s_srcBox, LV_FLEX_FLOW_COLUMN);
    lv_obj_remove_flag(s_srcBox, LV_OBJ_FLAG_SCROLLABLE);

#ifdef SEQ_USE_ENCODER
    s_srcGroup = lv_group_create();
    if (indev_encoderTop) lv_indev_set_group(indev_encoderTop, s_srcGroup);
#endif

    lv_obj_t* title = lv_label_create(s_srcBox);
    lv_obj_set_style_text_color(title, kText, 0);
    lv_label_set_text(title, "What the pads record");

    s_srcList = lv_obj_create(s_srcBox);
    lv_obj_remove_style_all(s_srcList);
    lv_obj_set_width(s_srcList, LV_PCT(100));
    lv_obj_set_flex_grow(s_srcList, 1);
    lv_obj_set_flex_flow(s_srcList, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(s_srcList, 3, 0);

    /* What Pitch Play is set to: the pads' range and what a pad makes. Read
     * only, and it used to be two rows of -/+ here. They are gone on purpose:
     * the instrument source is Pitch Play's instrument, so a second place to
     * set the key, the base note and the chord could only disagree with the
     * first -- and did, the same pattern sounding one way under the pads and
     * another recorded. Set it in Pitch Play; this says what it is. */
    lv_obj_t* info = make_row(s_srcBox, 24);
    lv_obj_t* il = lv_label_create(info);
    lv_obj_set_style_text_font(il, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(il, kDim, 0);
    lv_label_set_text(il, "Pitch Play");
    s_srcOctLbl = lv_label_create(info);
    lv_obj_set_flex_grow(s_srcOctLbl, 1);
    lv_obj_set_style_text_font(s_srcOctLbl, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_align(s_srcOctLbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(s_srcOctLbl, kAccent, 0);
    lv_label_set_text(s_srcOctLbl, "");
    s_srcChordLbl = lv_label_create(info);
    lv_obj_set_style_text_font(s_srcChordLbl, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_align(s_srcChordLbl, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_color(s_srcChordLbl, kAccent, 0);
    lv_label_set_text(s_srcChordLbl, "");

    s_srcNoteLbl = lv_label_create(s_srcBox);
    lv_obj_set_style_text_font(s_srcNoteLbl, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(s_srcNoteLbl, kAmber, 0);
    lv_obj_set_width(s_srcNoteLbl, LV_PCT(100));
    lv_label_set_long_mode(s_srcNoteLbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(s_srcNoteLbl, "");

    lv_obj_t* close = lv_button_create(s_srcBox);
    lv_obj_set_size(close, LV_PCT(100), 24);
    lv_obj_set_style_radius(close, 6, 0);
    lv_obj_set_style_shadow_width(close, 0, 0);
    lv_obj_set_style_bg_color(close, kPanel, 0);
    s_srcClose = close;
    lv_obj_add_event_cb(close, src_close_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_center(lv_label_create(close));
    lv_label_set_text(lv_obj_get_child(close, 0), "Close");
#ifdef SEQ_USE_ENCODER
    if (s_srcGroup) { lv_group_add_obj(s_srcGroup, close); lv_group_focus_obj(close); }
#endif

    /* Ask the card what is in the library; src_poll() fills the list when the
     * answer comes. The window opens with whatever was last read. */
    if (auto* st = tools()) {
        if (auto* h = inst_host()) {
            if (st->requestFolder(st->cardRoot() + h->libraryRoot())) s_srcListing = true;
        }
    }
    src_fill();
}

// ── swing ──────────────────────────────────────────────────────────────────

/* Swing is late, not moved: every second subdivision sounds a share of one
 * later, the pattern keeps its straight positions, and turning it off puts the
 * feel back exactly where it was. It is applied on the way out (see
 * PatternSequencer::swung), and a take recorded while it is on is pulled back
 * to the straight grid first -- the player aims at what they hear.
 *
 * Its own window because it is two settings, not one: how much, and which
 * subdivision. A button that only cycled through amounts would make the second
 * one unreachable, and a hold-to-edit knob would hide both. */

void swing_close()
{
    if (!s_swingBox) return;
#ifdef SEQ_USE_ENCODER
    if (indev_encoderTop && s_encGroup) lv_indev_set_group(indev_encoderTop, s_encGroup);
    lv_group_delete_deferred(s_swingGroup);   // the click that closed it is still being read
    s_swingGroup = nullptr;
#endif
    lv_obj_delete_async(s_swingBox);
    s_swingBox = s_swingText = s_swingVal = nullptr;
    for (auto& c : s_swingChip) c = nullptr;
    sync_ui();
}

void swing_close_cb(lv_event_t*) { swing_close(); }

/* The steps a sampler's swing knob clicks through, 50 % to 75 %, and what each
 * is on the 0..100 scale: intensity = (sampler - 50) * 4. The triplet one is
 * the exception -- those machines print 66 % and mean two thirds of the way
 * through the pair, which is 67 here, so the chip says 66 and sets the feel
 * rather than the rounding. */
struct SwingStep { uint8_t sampler; uint8_t intensity; };
const SwingStep kSwingSteps[7] = {
    {50, 0}, {54, 16}, {58, 32}, {62, 48}, {66, 67}, {70, 80}, {75, 100},
};

/// What it is set to, in the two numbers a player might be thinking in.
void swing_paint()
{
    auto* sq = seq();
    if (!sq) return;
    const uint8_t pct = sq->getSwing();
    const uint8_t per = sq->getSwingPerBeat();
    if (s_swingVal) {
        char v[8];
        std::snprintf(v, sizeof(v), "%u", (unsigned)pct);
        set_text(lv_obj_get_child(s_swingVal, 0), v);
    }
    if (s_swingText) {
        if (!pct) {
            set_text(s_swingText, "straight -- every subdivision on its own line");
        } else {
            set_text_fmt(s_swingText, "1/%u   .   sampler %u %%%s",
                         (unsigned)(per * 4), (unsigned)(50 + pct / 4),
                         pct >= 66 && pct <= 68 ? "   .   triplet feel" : "");
        }
    }
    /* The step it is standing on, lit. Turned off a step, nothing is lit --
     * which is the honest answer. */
    for (int i = 0; i < 7; ++i) {
        if (!s_swingChip[i]) continue;
        const bool on = kSwingSteps[i].intensity == pct;
        lv_obj_set_style_bg_color(s_swingChip[i], on ? kAmber : kPanel, 0);
        if (lv_obj_t* l = lv_obj_get_child(s_swingChip[i], 0)) {
            lv_obj_set_style_text_color(l, on ? kInk : kText, 0);
        }
    }
}

/// The amount, turned: focus it, press to take it, a point per detent, press
/// again to let go -- the same way the tempo is turned on the screen behind.
void swing_val_cb(lv_event_t* e)
{
    auto* sq = seq();
    lv_obj_t* b = static_cast<lv_obj_t*>(lv_event_get_current_target(e));
    if (!sq || !b) return;
    lv_group_t* g = lv_obj_get_group(b);
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        if (g) {
            const bool editing = lv_group_get_editing(g) && lv_group_get_focused(g) == b;
            lv_group_focus_obj(b);
            lv_group_set_editing(g, !editing);
        }
        return;
    }
    const uint32_t key = lv_event_get_key(e);
    const int step = key == LV_KEY_RIGHT || key == LV_KEY_UP ? 1
                   : (key == LV_KEY_LEFT || key == LV_KEY_DOWN ? -1 : 0);
    if (!step) return;
    const int now = (int)sq->getSwing() + step;
    sq->setSwing((uint8_t)(now < 0 ? 0 : (now > 100 ? 100 : now)));
    swing_paint();
}

void swing_set_cb(lv_event_t* e)
{
    auto* sq = seq();
    if (!sq) return;
    sq->setSwing((uint8_t)(intptr_t)lv_event_get_user_data(e));
    swing_paint();
}

void swing_unit_cb(lv_event_t* e)
{
    auto* sq = seq();
    if (!sq) return;
    sq->setSwingPerBeat(sq->getSwingPerBeat() == 2 ? 4 : 2);
    swing_paint();
    /* The button says which one it is, so it has to be repainted too. */
    if (lv_obj_t* b = static_cast<lv_obj_t*>(lv_event_get_current_target(e))) {
        if (lv_obj_t* l = lv_obj_get_child(b, 0)) {
            set_text_fmt(l, "Swing the 1/%u", (unsigned)(sq->getSwingPerBeat() * 4));
        }
    }
}

void swing_cb(lv_event_t*)
{
    auto* sq = seq();
    if (!sq || s_swingBox) return;

    s_swingBox = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_swingBox);
    lv_obj_set_size(s_swingBox, LV_PCT(100), LV_PCT(100));
    lv_obj_add_flag(s_swingBox, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_style_bg_color(s_swingBox, kBg, 0);
    lv_obj_set_style_bg_opa(s_swingBox, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_swingBox, 8, 0);
    lv_obj_set_style_pad_row(s_swingBox, 4, 0);
    lv_obj_set_flex_flow(s_swingBox, LV_FLEX_FLOW_COLUMN);
    lv_obj_remove_flag(s_swingBox, LV_OBJ_FLAG_SCROLLABLE);
#ifdef SEQ_USE_ENCODER
    s_swingGroup = lv_group_create();
    if (indev_encoderTop) lv_indev_set_group(indev_encoderTop, s_swingGroup);
#endif

    /* Whose it is: swing is the scene's own, and in a song the scene is the
     * track's -- every track has its feel. */
    lv_obj_t* title = lv_label_create(s_swingBox);
    lv_obj_set_style_text_color(title, kText, 0);
    if (in_track()) set_text_fmt(title, "SWING  -  track %d", crosspad::arrangeFocus().track + 1);
    else            set_text_fmt(title, "SWING  -  Seq %u", (unsigned)(sq->getActiveScene() + 1));

    s_swingText = lv_label_create(s_swingBox);
    lv_obj_set_style_text_font(s_swingText, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(s_swingText, kAmber, 0);
    lv_label_set_long_mode(s_swingText, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(s_swingText, LV_PCT(100));
    lv_label_set_text(s_swingText, "");

    /* The amount, big, with the scale beside it: one knob from nothing to as
     * far as those machines go. */
    lv_obj_t* amount = make_row(s_swingBox, 30);
    lv_obj_t* tag = lv_label_create(amount);
    lv_obj_set_style_text_font(tag, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(tag, kDim, 0);
    lv_label_set_text(tag, "AMOUNT");
    s_swingVal = swing_value(amount);
    lv_obj_t* of = lv_label_create(amount);
    lv_obj_set_style_text_font(of, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(of, kDim, 0);
    lv_label_set_text(of, "of 100");

    char unit[20];
    lv_snprintf(unit, sizeof(unit), "Swing the 1/%u", (unsigned)(sq->getSwingPerBeat() * 4));
    swing_button(s_swingBox, unit, swing_unit_cb, nullptr);

    /* The sampler's own steps, in a row: the numbers printed on those machines,
     * so a groove learned on one lands here by name. */
    lv_obj_t* steps = make_row(s_swingBox, 26);
    for (int i = 0; i < 7; ++i) {
        char label[8];
        lv_snprintf(label, sizeof(label), "%u", (unsigned)kSwingSteps[i].sampler);
        s_swingChip[i] = swing_chip(steps, label, (void*)(intptr_t)kSwingSteps[i].intensity);
    }

    swing_button(s_swingBox, LV_SYMBOL_OK "  Done", swing_close_cb, nullptr);

    swing_paint();
#ifdef SEQ_USE_ENCODER
    if (s_swingGroup) lv_group_focus_obj(s_swingVal);
#endif
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
    int offered = 0;   // the tracks in use and the next free one, never fewer than four
    {
        std::lock_guard<std::recursive_mutex> l(se->songMutex());
        project = se->song().name;
        offered = std::min(crosspad::AudioSong::kTracks,
                           std::max(se->song().usedTracks() + 1, crosspad::AudioSong::kTracksV5));
    }
    lv_obj_t* t = lv_label_create(s_bounceBox);
    lv_obj_set_style_text_color(t, kText, 0);
    lv_obj_set_width(t, LV_PCT(100));
    lv_label_set_long_mode(t, LV_LABEL_LONG_WRAP);
    set_text_fmt(t, "Bounce Seq %u (%u bars) into \"%s\" -- on which track?",
                          (unsigned)(sq->getActiveScene() + 1),
                          (unsigned)sq->getSceneBars(sq->getActiveScene()),
                          project.empty() ? "no card" : project.c_str());
    /* Two tracks a row: one a row was four to eight rows of 26 px under a
     * two-line question, and Cancel fell off the 200 px screen. */
    lv_obj_t* grid = lv_obj_create(s_bounceBox);
    lv_obj_remove_style_all(grid);
    lv_obj_set_size(grid, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(grid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_style_pad_row(grid, 4, 0);
    lv_obj_set_style_pad_column(grid, 4, 0);
    lv_obj_remove_flag(grid, LV_OBJ_FLAG_SCROLLABLE);
    for (int i = 0; i < offered; ++i) {
        char label[16];
        lv_snprintf(label, sizeof(label), "Track %d", i + 1);
        lv_obj_set_size(bounce_button(grid, label, bounce_track_cb, (void*)(intptr_t)i), LV_PCT(49), 24);
    }
    lv_obj_set_height(bounce_button(s_bounceBox, "Cancel", bounce_close_cb, nullptr), 24);
}

// ── building ───────────────────────────────────────────────────────────────

lv_obj_t* make_button(lv_obj_t* parent, int32_t w, int32_t h, const char* text,
                      lv_event_cb_t cb, void* ud = nullptr)
{
    lv_obj_t* b = lv_button_create(parent);
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_set_style_radius(b, 4, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, kPanel, 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(0x2C3239), LV_STATE_PRESSED);
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
/// leave. A button that shows the whole number, not a spinbox: the spinbox is
/// a text area that scrolls to its cursor, and in edit mode it showed only the
/// last digit. A tap puts it in edit mode too, for the encoder to turn.
lv_obj_t* make_bpm(lv_obj_t* parent)
{
    lv_obj_t* b = lv_CreateEditableButton(parent);
    lv_obj_set_size(b, 44, 22);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, kPanel, 0);
    lv_obj_set_style_bg_color(b, kAccent, LV_STATE_EDITED);
    /* On the button: only it is ever EDITED; the label takes the colour. */
    lv_obj_set_style_text_color(b, kText, 0);
    lv_obj_set_style_text_color(b, kInk, LV_STATE_EDITED);
    lv_obj_set_style_outline_color(b, kHead, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(b, 2, LV_STATE_FOCUS_KEY);
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
    lv_obj_add_flag(l, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_center(l);
    lv_obj_add_event_cb(b, bpm_cb, LV_EVENT_KEY, nullptr);
    lv_obj_add_event_cb(b, bpm_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(b, bpm_cb, LV_EVENT_FOCUSED, nullptr);
    lv_obj_add_event_cb(b, bpm_cb, LV_EVENT_DEFOCUSED, nullptr);
#ifdef SEQ_USE_ENCODER
    if (s_encGroup) lv_group_add_obj(s_encGroup, b);
#endif
    return b;
}

} // namespace

/* ── scene window ───────────────────────────────────────────────────────────
 *
 * What the design's two rows had no room for. The tempo is here rather than
 * on the screen because the transport bar shows it and nothing else can edit
 * it yet -- leaving it out of both would have meant a board whose tempo
 * cannot be changed at all. ADD and DEL make and unmake scenes; SRC and SYNC
 * are this sequencer's own and belong to no design row. */

void scene_menu_close()
{
    if (!s_sceneMenu) return;
#ifdef SEQ_USE_ENCODER
    if (indev_encoderTop && s_encGroup) lv_indev_set_group(indev_encoderTop, s_encGroup);
    lv_group_delete_deferred(s_sceneGroup);   // the click that closed it is still being read
    s_sceneGroup = nullptr;
#endif
    lv_obj_delete_async(s_sceneMenu);
    s_sceneMenu = nullptr;
    s_bpm = nullptr;
    s_srcBtn = s_srcLabel = nullptr;
    s_syncBtn = s_levelsBtn = s_preBtn = nullptr;
    sync_ui();
}

void scene_menu_close_cb(lv_event_t*) { scene_menu_close(); }

/// A row of the scene window, in its group rather than the screen's.
lv_obj_t* scene_menu_button(lv_obj_t* parent, int32_t w, const char* text, lv_event_cb_t cb)
{
    lv_obj_t* b = lv_button_create(parent);
    lv_obj_set_size(b, w, 28);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_set_style_radius(b, 4, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, kPanel, 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(0x2C3239), LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(b, kHead, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(b, 2, LV_STATE_FOCUS_KEY);
    if (cb) lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* l = lv_label_create(b);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(l, kText, 0);
    lv_label_set_text(l, text);
    lv_obj_center(l);
#ifdef SEQ_USE_ENCODER
    if (s_sceneGroup) lv_group_add_obj(s_sceneGroup, b);
#endif
    return b;
}

void scene_menu_cb(lv_event_t*)
{
    if (!seq() || s_sceneMenu) return;

    s_sceneMenu = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_sceneMenu);
    lv_obj_set_size(s_sceneMenu, LV_PCT(100), LV_PCT(100));
    lv_obj_add_flag(s_sceneMenu, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_style_bg_color(s_sceneMenu, kBg, 0);
    lv_obj_set_style_bg_opa(s_sceneMenu, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_sceneMenu, 8, 0);
    lv_obj_set_style_pad_row(s_sceneMenu, 5, 0);
    lv_obj_set_flex_flow(s_sceneMenu, LV_FLEX_FLOW_COLUMN);
    lv_obj_remove_flag(s_sceneMenu, LV_OBJ_FLAG_SCROLLABLE);
#ifdef SEQ_USE_ENCODER
    s_sceneGroup = lv_group_create();
    if (indev_encoderTop) lv_indev_set_group(indev_encoderTop, s_sceneGroup);
#endif

    lv_obj_t* title = lv_label_create(s_sceneMenu);
    lv_obj_set_style_text_color(title, kText, 0);
    lv_label_set_text(title, "SCENE");

    /* Tempo first: it is the one thing here that no other screen can set. */
    lv_obj_t* tempo = make_row(s_sceneMenu, 30);
    lv_obj_t* tag = lv_label_create(tempo);
    lv_obj_set_style_text_font(tag, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(tag, kDim, 0);
    lv_label_set_text(tag, "BPM");
    s_bpm = make_bpm(tempo);
#ifdef SEQ_USE_ENCODER
    if (s_sceneGroup && s_bpm) lv_group_add_obj(s_sceneGroup, s_bpm);
#endif

    lv_obj_t* scenes = make_row(s_sceneMenu, 28);
    scene_menu_button(scenes, 68, LV_SYMBOL_PLUS "  ADD", scene_add_cb);
    scene_menu_button(scenes, 68, LV_SYMBOL_MINUS "  DEL", scene_del_cb);
    /* The count-in lives here since the soft keys became the track view's. */
    s_preBtn = scene_menu_button(scenes, 68, "PRE", pre_cb);

    lv_obj_t* srcs = make_row(s_sceneMenu, 28);
    s_srcBtn = scene_menu_button(srcs, 68, "KIT", src_cb);
    s_srcLabel = lv_obj_get_child(s_srcBtn, 0);
    s_syncBtn = scene_menu_button(srcs, 68, "SYNC", sync_cb);
    s_levelsBtn = scene_menu_button(srcs, 104, "16 LEVELS", levels_cb);
    src_sync();

    scene_menu_button(s_sceneMenu, 120, LV_SYMBOL_OK "  Done", scene_menu_close_cb);

    sync_ui();
#ifdef SEQ_USE_ENCODER
    if (s_sceneGroup && s_bpm) lv_group_focus_obj(s_bpm);
#endif
}

lv_obj_t* Sequencer_create(lv_obj_t* parent, App* app)
{
    /* Before anything is built: the keys and the name depend on it. */
    adopt_song_track();

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
    lv_obj_set_style_pad_all(s_root, 0, 0);
    lv_obj_remove_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);

    /* Absolute positions, not the flex column this screen had: the design
     * fixes every row to a screen y and its two gaps are not equal -- 5 above
     * the grid, 6 below it -- which one pad_row cannot express. SEQ_Y turns a
     * screen y into a root y: the root fills the app container, which begins
     * under the transport bar -- 40 px since it grew from the design's 30, so
     * the rows above the grid sit 10 px lower than the design's and the grid
     * is 10 px shorter: placed at the design's y the top row was 5 px under
     * the bar. */
#define SEQ_Y(v) ((v) - TRANSPORT_BAR_HEIGHT)
    auto place = [](lv_obj_t* o, int32_t x, int32_t y) { if (o) lv_obj_set_pos(o, x, y); return o; };

    /* ── context row: which scene, how long, how snapped ──────────────────
     * No back button and no tempo field: back is the power button's click
     * everywhere, and the tempo is in the transport bar (edited in the scene
     * window behind the name). */
    place(make_button(s_root, 20, 22, LV_SYMBOL_LEFT, scene_prev_cb), 6, SEQ_Y(45));

    s_sceneBtn = make_button(s_root, 56, 22, "Seq 1/1", scene_menu_cb);
    place(s_sceneBtn, 26, SEQ_Y(45));
    s_sceneLabel = lv_obj_get_child(s_sceneBtn, 0);
    /* The design sets the name in montserrat 12, which this firmware does not
     * build; 14 is the next one up and 56 px holds "Seq 1/1" in it. Flat,
     * because the design draws the name on the background and not as a key --
     * it only has to look like a button once the encoder reaches it. */
    lv_obj_set_style_text_font(s_sceneLabel, &lv_font_montserrat_14, 0);
    lv_obj_set_style_bg_opa(s_sceneBtn, LV_OPA_TRANSP, 0);

    place(make_button(s_root, 20, 22, LV_SYMBOL_RIGHT, scene_next_cb), 82, SEQ_Y(45));
    place(make_button(s_root, 22, 22, LV_SYMBOL_MINUS, len_dn_cb), 108, SEQ_Y(45));
    place(make_button(s_root, 22, 22, LV_SYMBOL_PLUS,  len_up_cb), 134, SEQ_Y(45));

    lv_obj_t* barsBtn = make_button(s_root, 30, 22, "4b", bars_cb);
    place(barsBtn, 162, SEQ_Y(45));
    s_barsLabel = lv_obj_get_child(barsBtn, 0);

    s_quantBtn = make_button(s_root, 36, 22, "1/16", quant_cb);
    place(s_quantBtn, 196, SEQ_Y(45));
    s_quantLabel = lv_obj_get_child(s_quantBtn, 0);

    /* 78 px, so the amount is on the key rather than hidden behind it: a feel
     * you cannot see the value of is one you cannot put back. */
    s_swingBtn = make_button(s_root, 78, 22, "SWING", swing_cb);
    place(s_swingBtn, 236, SEQ_Y(45));

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

    /* ── the pattern ──────────────────────────────────────────────────────
     * Still drawn into the layer, not built from cells. The design lays out
     * 12 rows x 16 boxes plus 17 line objects; this keeps 16 lanes -- all the
     * pads, not twelve of them -- in the pad's own colour with the velocity
     * as its opacity, which the mockup has no way to show, and costs no
     * objects at all. On this board that is the difference between a screen
     * and an out-of-memory: 209 LVGL objects is tens of kB of the PSRAM the
     * UI lives in. */
    s_grid = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_grid);
    place(s_grid, 6, SEQ_Y(72));
    lv_obj_set_size(s_grid, 308, 128);
    lv_obj_set_style_bg_color(s_grid, kGridBg, 0);
    lv_obj_set_style_bg_opa(s_grid, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(s_grid, 4, 0);
    lv_obj_set_style_pad_all(s_grid, 3, 0);
    lv_obj_remove_flag(s_grid, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_grid, grid_draw_cb, LV_EVENT_DRAW_MAIN_END, nullptr);

    /* ── soft keys ──────────────────────────────────────────────────────
     * One row, the same whether this screen was opened on its own or as an
     * Arrange track's seq (the design's seq view): the way to the arrangement,
     * the way into the sound the notes play through, x2, /2, BOUNCE, ERASE and
     * CLEAR. On its own, ARR opens Arrange and BOUNCE puts the scene into the
     * open song; as a track, BOUNCE is asked over the arrangement it changes.
     * The click is the transport bar's key, the count-in is in the scene
     * window behind the name.
     *
     * Chosen here, not switched later, because an app is built on every start
     * -- so what Arrange handed over is known by the time this runs. */
    s_arrBtn = make_button(s_root, 44, 28, LV_SYMBOL_LEFT " ARR", arr_back_cb);
    place(s_arrBtn, 6, SEQ_Y(206));
    s_editBtn = make_button(s_root, 48, 28, "EDIT " LV_SYMBOL_RIGHT, edit_src_cb);
    place(s_editBtn, 52, SEQ_Y(206));
    /* Lime: of the keys this is the one that leads somewhere new -- the sound
     * the notes play through. */
    lv_obj_set_style_bg_color(s_editBtn, kAccent, 0);
    lv_obj_set_style_text_color(lv_obj_get_child(s_editBtn, 0), kInk, 0);
    place(make_button(s_root, 28, 28, "x2", double_cb), 102, SEQ_Y(206));
    place(make_button(s_root, 28, 28, "/2", halve_cb), 132, SEQ_Y(206));
    place(make_button(s_root, 54, 28, "BOUNCE", in_track() ? track_bounce_cb : bounce_cb), 162, SEQ_Y(206));
    s_eraseBtn = make_button(s_root, 48, 28, "ERASE", nullptr);
    place(s_eraseBtn, 218, SEQ_Y(206));
    s_clearBtn = make_button(s_root, 46, 28, "CLEAR", clear_cb);
    place(s_clearBtn, 268, SEQ_Y(206));
    lv_obj_add_event_cb(s_eraseBtn, erase_cb, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_event_cb(s_eraseBtn, erase_cb, LV_EVENT_RELEASED, nullptr);
    lv_obj_add_event_cb(s_eraseBtn, erase_cb, LV_EVENT_PRESS_LOST, nullptr);
#undef SEQ_Y


    /* A track that plays through Pitch Play plays its instrument, not the
     * kit. Pitch Play publishes which one (InstrumentPlayState), so there is
     * nothing to ask the player here: the track said PITCH on Arrange's
     * "plays with" card and this is what that means. With no instrument
     * chosen yet the source stays the kit and the lime EDIT key is where one
     * is picked -- which is what that key is for. */
    if (in_track() && crosspad::arrangeFocus().source == crosspad::SeqSource::Pitch &&
        s_src == Src::Kit) {
        const crosspad::InstrumentPlayState& ps = crosspad::instrumentPlayState();
        if (!ps.instrumentPath.empty()) src_to_inst(ps.instrumentPath, ps.instrumentName);
    }

    seq_rebuild_pads();
    sync_ui();
    s_drawn = pattern_shape();
    s_headX = -1;

#ifdef SEQ_USE_ENCODER
    /* The scene name: the first thing a player reaches for is which groove,
     * and PLAY is on the transport bar now. */
    if (s_encGroup && s_sceneBtn) lv_group_focus_obj(s_sceneBtn);
#endif

    s_timer = lv_timer_create(tick_cb, kTickMs, nullptr);

    if (app) {
        app->setOnShow([](lv_obj_t*) {
            /* PLAY, REC and DUB in the top bar mean this pattern while this
             * screen is up. Without it the claim stayed with whatever took it
             * last -- the Arranger's song, which cannot record -- and REC and
             * DUB were greyed out here, so the screen that records was the one
             * screen that could not. */
            /* Opened from an Arrange track it is one block of it, looped on
             * its own here; recording against the song is Arrange's REC. */
            open_track_block();
            crosspad::transportClaim(crosspad::sequencerTransport());
            if (s_timer) lv_timer_resume(s_timer);
#ifdef SEQ_USE_ENCODER
            if (indev_encoderTop) {
                /* Back to whatever window is on top, if one was left open:
                 * the screen's group would steer buttons hidden under it. */
                lv_group_t* g = s_encGroup;
                if (s_srcBox && s_srcGroup) g = s_srcGroup;
                else if (s_bounceBox && s_bounceGroup) g = s_bounceGroup;
                else if (s_swingBox && s_swingGroup) g = s_swingGroup;
                else if (s_sceneMenu && s_sceneGroup) g = s_sceneGroup;
                if (g) lv_indev_set_group(indev_encoderTop, g);
            }
#endif
        });
        app->setOnHide([](lv_obj_t*) {
            if (s_timer) lv_timer_pause(s_timer);
            /* Erase must never outlive the button holding it. */
            if (auto* sq = seq()) sq->setErasing(false);
            /* The block's last changes, whichever way the screen is left. */
            if (auto* se = songs()) se->closeSeqEdit();
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

    /* The pads go back to the kit's. The engine too -- unless a bounce is
     * still capturing them, which it does on its own after the screen is
     * gone: reloading the kit under it would record the wrong instrument.
     * It is then held until SRC is set back to KIT, or Pitch Play takes it. */
    src_drop_pads();
    if (s_src == Src::Inst && !s_bouncing) {
        if (auto* h = inst_host()) h->release();
        s_src = Src::Kit;
        s_instPath.clear();
        s_instName.clear();
    }
    s_instReady.store(false);
    if (s_srcPadLogic) s_srcPadLogic->setReady(false);
    s_srcNote.clear();
    s_srcListing = false;

#ifdef SEQ_USE_ENCODER
    if (s_srcGroup) { lv_group_delete(s_srcGroup); s_srcGroup = nullptr; }
    if (s_bounceGroup) { lv_group_delete(s_bounceGroup); s_bounceGroup = nullptr; }
    if (s_swingGroup) { lv_group_delete(s_swingGroup); s_swingGroup = nullptr; }
#endif
    s_bounceBox = s_bounceText = s_bounceBar = s_bounceBtn = nullptr;
    s_swingBox = s_swingText = s_swingVal = s_swingBtn = nullptr;
    s_sceneMenu = s_sceneBtn = nullptr;
#ifdef SEQ_USE_ENCODER
    /* Set only while the scene window is open; a group does not die with the
     * tree, so closing the app over the window would leak it. */
    if (s_sceneGroup) { lv_group_delete(s_sceneGroup); s_sceneGroup = nullptr; }
#endif
    s_srcBox = s_srcList = s_srcNoteLbl = s_srcOctLbl = s_srcBtn = s_srcLabel = nullptr;
    s_srcChordLbl = nullptr;
    s_srcClose = nullptr;
    for (auto& c : s_swingChip) c = nullptr;
    s_bouncing = false;
    s_root = s_grid = s_bpm = s_syncBtn = s_levelsBtn = s_clearBtn = s_preBtn = s_barsLabel = nullptr;
    s_clearArmed = false;
    s_quantBtn = s_quantLabel = s_sceneLabel = s_eraseBtn = nullptr;
    s_arrBtn = s_editBtn = nullptr;
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

    if (root) lv_obj_delete_async(root);
}

// Auto-registration: the app shows up in the launcher with no wiring anywhere
// else. No pad logic: the pads stay the Sampler's, so they play and record here
// exactly as they do everywhere else.
REGISTER_APP(Sequencer, nullptr, "sequencer.png",
             Sequencer_create, Sequencer_destroy,
             nullptr, nullptr, nullptr, 0)
