#pragma once
// What the touchscreen screens are built from: the colours, one frame's view of the touch
// panel, text helpers and the widgets. Plain C++ (no ESPHome, no LVGL), so the device, the PC
// simulator (tools/ui_sim) and the host tests run the same code.
//
// The UI is "immediate mode": every frame a screen's code runs top to bottom, and the call that
// draws a button also says whether it was just tapped:
//
//     if (button(f, 788, 536, 220, 52, "START", kGreen)) start_job();
//
// There are no widget objects. What must outlive a frame (the open screen, a pop-up, the
// selected plane) is a member of BalancerUi. The touch handling and the way a frame is drawn
// come from cellkeeper-hmi (components/hmi_ui), which runs on the same panel.

#include <cstdint>

#include "ui_canvas.h"
#include "ui_fonts.h"

namespace bal {

static const int SCREEN_W = 1024;
static const int SCREEN_H = 600;

// ---- colours ----
constexpr Rgb kBg = 0x0f1419, kPanel = 0x1b232c, kPanel2 = 0x26313d, kText = 0xe6edf3, kMuted = 0x8b98a5;
constexpr Rgb kAccent = 0x3b82f6, kGreen = 0x22c55e, kAmber = 0xf59e0b, kRed = 0xef4444;
constexpr Rgb kChA = 0x38bdf8, kChB = 0xfb923c;  // sensor A / plane 1, sensor B / plane 2
constexpr Rgb kPurple = 0x7c3aed;                 // demo-only controls
constexpr Rgb kGrid = 0x3a4654;                   // plot grid lines, disabled buttons

// ---- in: the touch panel, as it is now ----
struct Input {
  bool down = false;
  int x = 0, y = 0;
};

// ---- for the tests and the simulator: where each thing that can be tapped was drawn ----
struct Hit {
  char label[48];
  int x, y, w, h;
};
struct HitTrace {
  int count = 0;
  Hit hits[160];
  // The n-th thing drawn this frame with that label; nullptr if there is none
  const Hit *find(const char *label, int nth = 0) const;
};

// ---- one frame, as a screen and its widgets see it ----
struct Frame {
  Canvas &c;
  uint32_t now_ms;

  bool active = true;     // false: widgets draw but do not react (a pop-up is open over them)
  bool down = false;      // a finger is on the panel
  bool released = false;  // it was lifted this frame, and no widget has taken the tap yet
  bool dragging = false;  // it has moved too far from where it came down to be a tap
  int tx = 0, ty = 0;     // where it is (where it was, when lifted)
  int press_x = 0, press_y = 0;  // where it came down
  HitTrace *trace = nullptr;
  bool changed = false;   // a widget changed something: the frame is drawn again

  // Is the finger down on this rectangle, as a press and not a drag?
  bool held(int x, int y, int w, int h) const;
  // Is the finger down, did it come down here and is it still inside? (a press-and-hold
  // button: a little wobble does not let go)
  bool holding(int x, int y, int w, int h) const;
  // Was this rectangle just tapped? Takes the tap, so only one widget gets it.
  bool tapped(int x, int y, int w, int h, const char *label);
  // Did the press that is going on start inside this rectangle?
  bool pressed_in(int x, int y, int w, int h) const;
};

// ---- text ----

// printf into one of a few static buffers: good for the arguments of one drawing call
const char *fmt(const char *format, ...) __attribute__((format(printf, 1, 2)));
// Lines split at '\n', each broken at spaces to fit `width` (0: no breaking). Draws at most
// max_lines lines; returns the y below the last one.
int text_block(Canvas &c, const Font &font, int x, int y, int width, const char *s, Rgb color, int max_lines = 8,
               int line_gap = 0, Align align = LEFT);
// Text vertically centred on the line at y_mid
void text_mid(Canvas &c, const Font &font, int x, int y_mid, const char *s, Rgb color, Align align = LEFT);

// ---- shapes ----

void panel(Canvas &c, int x, int y, int w, int h, Rgb color = kPanel, int radius = 10);
// A horizontal bar filled to `frac` (0..1) from the left
void hbar(Canvas &c, int x, int y, int w, int h, float frac, Rgb fill, Rgb track = kPanel2, int radius = 0);
// A vertical bar filled to `frac` from the bottom
void vbar(Canvas &c, int x, int y, int w, int h, float frac, Rgb fill, Rgb track = kPanel2, int radius = 3);
// A line through n points with round joins
void polyline(Canvas &c, const float *xs, const float *ys, int n, float width, Rgb color);

// ---- widgets: they draw themselves and say what the touch did ----

// A button. True when it was tapped. A disabled one is drawn dim and never reacts. `label`
// names it for the tests too, unless `name` is given (buttons with the same label on one screen).
bool button(Frame &f, int x, int y, int w, int h, const char *label, Rgb fill = kPanel2, bool enabled = true,
            const Font &font = FONT_M20, const char *name = nullptr);
// A button that moves on to the next step: its label with a chevron after it
bool button_next(Frame &f, int x, int y, int w, int h, const char *label, Rgb fill = kGreen, bool enabled = true,
                 const Font &font = FONT_M20);
// A big tile: a title and a line under it
bool tile(Frame &f, int x, int y, int w, int h, const char *title, const char *sub, Rgb fill, bool enabled = true);
// One setting: its name on the left, its value on the right. True when tapped.
bool setting_row(Frame &f, int x, int y, int w, const char *name, const char *value);
static const int ROW_H = 40;
// An area that can be tapped but draws nothing itself. `label` names it for the tests.
bool hot(Frame &f, int x, int y, int w, int h, const char *label);

}  // namespace bal
