#include "ui_kit.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace bal {

// ---- hits ----

const Hit *HitTrace::find(const char *label, int nth) const {
  for (int i = 0; i < count; i++) {
    if (strcmp(hits[i].label, label) == 0 && nth-- == 0) return &hits[i];
  }
  return nullptr;
}

static bool in_rect(int px, int py, int x, int y, int w, int h) { return px >= x && px < x + w && py >= y && py < y + h; }

bool Frame::pressed_in(int x, int y, int w, int h) const { return active && in_rect(press_x, press_y, x, y, w, h); }

bool Frame::held(int x, int y, int w, int h) const { return down && !dragging && this->pressed_in(x, y, w, h); }

bool Frame::holding(int x, int y, int w, int h) const {
  return down && this->pressed_in(x, y, w, h) && in_rect(tx, ty, x, y, w, h);
}

bool Frame::tapped(int x, int y, int w, int h, const char *label) {
  if (!active) return false;
  if (trace != nullptr && label[0] && trace->count < (int) (sizeof(trace->hits) / sizeof(trace->hits[0]))) {
    Hit &hit = trace->hits[trace->count++];
    strncpy(hit.label, label, sizeof(hit.label) - 1);
    hit.label[sizeof(hit.label) - 1] = 0;
    hit.x = x;
    hit.y = y;
    hit.w = w;
    hit.h = h;
  }
  if (!released || dragging || !this->pressed_in(x, y, w, h) || !in_rect(tx, ty, x, y, w, h)) return false;
  released = false;
  changed = true;
  return true;
}

bool hot(Frame &f, int x, int y, int w, int h, const char *label) { return f.tapped(x, y, w, h, label); }

// ---- text ----

const char *fmt(const char *format, ...) {
  // On the heap at first use, not in static RAM: on the ESP32-P4 static data
  // shares a small block of internal RAM with the heap FreeRTOS starts from.
  static char (*buf)[320] = new char[12][320];
  static int next = 0;
  char *out = buf[next];
  next = (next + 1) % 12;
  va_list args;
  va_start(args, format);
  vsnprintf(out, sizeof(buf[0]), format, args);
  va_end(args);
  return out;
}

int text_block(Canvas &c, const Font &font, int x, int y, int width, const char *s, Rgb color, int max_lines,
               int line_gap, Align align) {
  char line[320];
  int lines = 0;
  const int step = font.line_height() + line_gap;
  auto put = [&](const char *t) {
    c.text(font, x, y, t, color, align);
    y += step;
    lines++;
  };
  const char *p = s;
  while (*p && lines < max_lines) {
    // One paragraph: up to the next '\n'
    const char *end = strchr(p, '\n');
    if (end == nullptr) end = p + strlen(p);
    size_t len = 0;
    const char *q = p;
    while (q < end && lines < max_lines) {
      // The next word, with the spaces before it
      const char *w = q;
      while (w < end && *w == ' ') w++;
      while (w < end && *w != ' ') w++;
      const size_t word = (size_t) (w - q);
      if (len + word >= sizeof(line)) break;
      memcpy(line + len, q, word);
      line[len + word] = 0;
      if (width > 0 && len > 0 && Canvas::text_width(font, line) > width) {
        line[len] = 0;
        put(line);
        len = 0;
        while (q < end && *q == ' ') q++;
        continue;
      }
      len += word;
      q = w;
    }
    if (lines < max_lines) {
      line[len] = 0;
      put(line);
    }
    p = *end ? end + 1 : end;
  }
  return y;
}

void text_mid(Canvas &c, const Font &font, int x, int y_mid, const char *s, Rgb color, Align align) {
  c.text(font, x, y_mid - font.line_height() / 2, s, color, align);
}

// ---- shapes ----

void panel(Canvas &c, int x, int y, int w, int h, Rgb color, int radius) { c.round_rect(x, y, w, h, radius, color); }

// A NaN or a negative fraction shows as empty: a bar never shows more than there is
static float clamp_frac(float frac) { return frac > 0.0f ? std::min(1.0f, frac) : 0.0f; }

// The fill is the track's own shape in the fill colour, cut off where the value ends, so a
// short fill keeps the track's rounded end and never sticks out past it
void hbar(Canvas &c, int x, int y, int w, int h, float frac, Rgb fill, Rgb track, int radius) {
  c.round_rect(x, y, w, h, radius, track);
  const int fw = (int) (w * clamp_frac(frac) + 0.5f);
  if (fw <= 0) return;
  c.set_clip(x, y, fw, h);
  c.round_rect(x, y, w, h, radius, fill);
  c.reset_clip();
}

void vbar(Canvas &c, int x, int y, int w, int h, float frac, Rgb fill, Rgb track, int radius) {
  c.round_rect(x, y, w, h, radius, track);
  const int fh = (int) (h * clamp_frac(frac) + 0.5f);
  if (fh <= 0) return;
  c.set_clip(x, y + h - fh, w, fh);
  c.round_rect(x, y, w, h, radius, fill);
  c.reset_clip();
}

void polyline(Canvas &c, const float *xs, const float *ys, int n, float width, Rgb color) {
  for (int i = 0; i + 1 < n; i++) c.line(xs[i], ys[i], xs[i + 1], ys[i + 1], width, color);
}

// ---- widgets ----

static Rgb face_of(const Frame &f, int x, int y, int w, int h, Rgb fill, bool enabled) {
  if (!enabled) return kGrid;
  return f.held(x, y, w, h) ? mix(fill, 0xFFFFFF, 0.16f) : fill;
}

bool button(Frame &f, int x, int y, int w, int h, const char *label, Rgb fill, bool enabled, const Font &font,
            const char *name) {
  Canvas &c = f.c;
  c.round_rect(x, y, w, h, 10, face_of(f, x, y, w, h, fill, enabled));
  text_mid(c, font, x + w / 2, y + h / 2, label, enabled ? kText : kMuted, CENTER);
  return enabled && f.tapped(x, y, w, h, name != nullptr ? name : label);
}

bool button_next(Frame &f, int x, int y, int w, int h, const char *label, Rgb fill, bool enabled, const Font &font) {
  Canvas &c = f.c;
  c.round_rect(x, y, w, h, 10, face_of(f, x, y, w, h, fill, enabled));
  const Rgb ink = enabled ? kText : kMuted;
  const int tw = Canvas::text_width(font, label), arrow = font.size / 2, gap = font.size / 2;
  const int x0 = x + (w - tw - gap - arrow) / 2;
  text_mid(c, font, x0, y + h / 2, label, ink);
  const float ax = static_cast<float>(x0 + tw + gap), ay = y + h / 2.0f, s = arrow * 0.55f;
  const float lw = font.size >= 28 ? 3.5f : 2.5f;
  c.line(ax, ay - s, ax + s, ay, lw, ink);
  c.line(ax + s, ay, ax, ay + s, lw, ink);
  return enabled && f.tapped(x, y, w, h, label);
}

bool tile(Frame &f, int x, int y, int w, int h, const char *title, const char *sub, Rgb fill, bool enabled) {
  Canvas &c = f.c;
  c.round_rect(x, y, w, h, 10, face_of(f, x, y, w, h, fill, enabled));
  text_mid(c, FONT_M40, x + w / 2, y + h / 2 - 18, title, enabled ? kText : kMuted, CENTER);
  text_mid(c, FONT_R20, x + w / 2, y + h / 2 + 32, sub, enabled ? 0xc7d2dc : kMuted, CENTER);
  return enabled && f.tapped(x, y, w, h, title);
}

bool setting_row(Frame &f, int x, int y, int w, const char *name, const char *value) {
  Canvas &c = f.c;
  c.round_rect(x, y, w, ROW_H, 8, f.held(x, y, w, ROW_H) ? mix(kPanel2, 0xFFFFFF, 0.12f) : kPanel2);
  text_mid(c, FONT_R20, x + 14, y + ROW_H / 2, name, kText);
  text_mid(c, FONT_R20, x + w - 14, y + ROW_H / 2, value, kChA, RIGHT);
  return f.tapped(x, y, w, ROW_H, name);
}

}  // namespace bal
