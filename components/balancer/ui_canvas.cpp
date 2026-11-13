// The device build is optimised for size; this file is where the time goes when a frame is
// drawn, so it alone is built for speed.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize("O2")
#endif

#include "ui_canvas.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace bal {

static inline float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

void Canvas::begin_dry() {
  dry_ = true;
  for (auto &sum : band_sums_) sum = 2166136261u;
}

void Canvas::add_to_bands_(int y0, int y1, uint32_t call) {
  // What a call draws depends on the clip it is drawn under
  call = sum_up_(sum_up_(sum_up_(sum_up_(call, (uint32_t) cx0_), (uint32_t) cy0_), (uint32_t) cx1_), (uint32_t) cy1_);
  y0 = std::max(y0, cy0_);
  y1 = std::min(y1, cy1_);
  if (y0 >= y1) return;
  for (int band = y0 / BAND_ROWS; band <= (y1 - 1) / BAND_ROWS && band < MAX_BANDS; band++) {
    band_sums_[band] = sum_up_(band_sums_[band], call);
  }
}

void Canvas::set_window(int y0, int y1) {
  win_y0_ = std::max(0, y0);
  win_y1_ = std::min(h_, y1);
  this->reset_clip();
}

void Canvas::set_clip(int x, int y, int w, int h) {
  const int top = win_y1_ > win_y0_ ? win_y0_ : 0, bottom = win_y1_ > win_y0_ ? win_y1_ : h_;
  cx0_ = std::max(0, x);
  cy0_ = std::max(top, y);
  cx1_ = std::min(w_, x + w);
  cy1_ = std::min(bottom, y + h);
}

void Canvas::reset_clip() {
  cx0_ = 0;
  cy0_ = win_y1_ > win_y0_ ? win_y0_ : 0;
  cx1_ = w_;
  cy1_ = win_y1_ > win_y0_ ? win_y1_ : h_;
}

void Canvas::clear(Rgb c) {
  // Within the window and the clip, as everything else is
  this->fill(0, 0, w_, h_, c);
}

void Canvas::fill(int x, int y, int w, int h, Rgb c) {
  if (this->dry_call_(y, y + h, 4, x, y, w, h, c)) return;
  const int x0 = std::max(cx0_, x), y0 = std::max(cy0_, y);
  const int x1 = std::min(cx1_, x + w), y1 = std::min(cy1_, y + h);
  if (x0 >= x1 || y0 >= y1) return;
  const uint16_t p = to565(c);
  for (int yy = y0; yy < y1; yy++) std::fill(px_ + (size_t) yy * w_ + x0, px_ + (size_t) yy * w_ + x1, p);
}

// Mixes a colour into one pixel. The channels are worked on at 8 bits and stored back as 565.
inline void Canvas::blend_(int x, int y, uint32_t r, uint32_t g, uint32_t b, uint32_t alpha) {
  if (x < cx0_ || x >= cx1_ || y < cy0_ || y >= cy1_ || alpha == 0) return;
  uint16_t &p = px_[(size_t) y * w_ + x];
  if (alpha >= 255) {
    p = (uint16_t) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
    return;
  }
  const uint32_t dr = (p >> 8) & 0xF8, dg = (p >> 3) & 0xFC, db = (p << 3) & 0xF8;
  const uint32_t inv = 255 - alpha;
  const uint32_t orr = (r * alpha + dr * inv + 127) / 255;
  const uint32_t og = (g * alpha + dg * inv + 127) / 255;
  const uint32_t ob = (b * alpha + db * inv + 127) / 255;
  p = (uint16_t) (((orr & 0xF8) << 8) | ((og & 0xFC) << 3) | (ob >> 3));
}

void Canvas::fill_blend(int x, int y, int w, int h, Rgb c, uint8_t alpha) {
  if (this->dry_call_(y, y + h, 5, x, y, w, h, c, (int) alpha)) return;
  const uint32_t r = (c >> 16) & 0xFF, g = (c >> 8) & 0xFF, b = c & 0xFF;
  for (int yy = std::max(cy0_, y); yy < std::min(cy1_, y + h); yy++) {
    for (int xx = std::max(cx0_, x); xx < std::min(cx1_, x + w); xx++) this->blend_(xx, yy, r, g, b, alpha);
  }
}

// How much of a pixel lies inside a rounded rectangle: 1 inside, 0 outside, in between on the
// curve of a corner. (px, py) is the pixel's centre relative to the rectangle's top left.
static inline float round_cover(float px, float py, float w, float h, float radius) {
  const float dx = std::max(std::max(radius - px, px - (w - radius)), 0.0f);
  const float dy = std::max(std::max(radius - py, py - (h - radius)), 0.0f);
  if (dx == 0.0f || dy == 0.0f) return 1.0f;
  return clamp01(radius - std::sqrt(dx * dx + dy * dy) + 0.5f);
}

void Canvas::round_rect(int x, int y, int w, int h, int radius, Rgb c) {
  if (this->dry_call_(y, y + h, 6, x, y, w, h, radius, c)) return;
  radius = std::max(0, std::min(radius, std::min(w, h) / 2));
  if (radius == 0) {
    this->fill(x, y, w, h, c);
    return;
  }
  // The middle and the straight sides are solid; only the four corner squares need blending
  this->fill(x, y + radius, w, h - 2 * radius, c);
  this->fill(x + radius, y, w - 2 * radius, radius, c);
  this->fill(x + radius, y + h - radius, w - 2 * radius, radius, c);
  const uint32_t r = (c >> 16) & 0xFF, g = (c >> 8) & 0xFF, b = c & 0xFF;
  for (int j = 0; j < radius; j++) {
    for (int i = 0; i < radius; i++) {
      const uint32_t a = (uint32_t) (round_cover(i + 0.5f, j + 0.5f, (float) w, (float) h, (float) radius) * 255.0f);
      if (a == 0) continue;
      this->blend_(x + i, y + j, r, g, b, a);
      this->blend_(x + w - 1 - i, y + j, r, g, b, a);
      this->blend_(x + i, y + h - 1 - j, r, g, b, a);
      this->blend_(x + w - 1 - i, y + h - 1 - j, r, g, b, a);
    }
  }
}

void Canvas::round_frame(int x, int y, int w, int h, int radius, int thickness, Rgb c) {
  if (this->dry_call_(y, y + h, 7, x, y, w, h, radius, thickness, c)) return;
  radius = std::max(0, std::min(radius, std::min(w, h) / 2));
  const int t = thickness;
  const int corner = std::max(radius, t);
  // Straight parts
  this->fill(x + corner, y, w - 2 * corner, t, c);
  this->fill(x + corner, y + h - t, w - 2 * corner, t, c);
  this->fill(x, y + corner, t, h - 2 * corner, c);
  this->fill(x + w - t, y + corner, t, h - 2 * corner, c);
  // Corners: inside the outer shape and outside the inner one
  const uint32_t r = (c >> 16) & 0xFF, g = (c >> 8) & 0xFF, b = c & 0xFF;
  const float inner_radius = std::max(0.0f, (float) radius - t);
  for (int j = 0; j < corner; j++) {
    for (int i = 0; i < corner; i++) {
      const float outer = round_cover(i + 0.5f, j + 0.5f, (float) w, (float) h, (float) radius);
      const float inner = (i >= t && j >= t)
                              ? round_cover(i - t + 0.5f, j - t + 0.5f, (float) (w - 2 * t), (float) (h - 2 * t), inner_radius)
                              : 0.0f;
      const uint32_t a = (uint32_t) (clamp01(outer - inner) * 255.0f);
      if (a == 0) continue;
      this->blend_(x + i, y + j, r, g, b, a);
      this->blend_(x + w - 1 - i, y + j, r, g, b, a);
      this->blend_(x + i, y + h - 1 - j, r, g, b, a);
      this->blend_(x + w - 1 - i, y + h - 1 - j, r, g, b, a);
    }
  }
}

static inline bool all_finite(float a, float b, float c) { return std::isfinite(a) && std::isfinite(b) && std::isfinite(c); }

void Canvas::disc(float cx, float cy, float radius, Rgb c, uint8_t alpha) {
  if (!all_finite(cx, cy, radius)) return;
  if (this->dry_call_((int) std::floor(cy - radius) - 1, (int) std::ceil(cy + radius) + 2, 8, cx, cy, radius, c, (int) alpha)) return;
  const uint32_t r = (c >> 16) & 0xFF, g = (c >> 8) & 0xFF, b = c & 0xFF;
  int ymin = (int) std::floor(cy - radius - 1), ymax = (int) std::ceil(cy + radius + 1);
  int xmin = (int) std::floor(cx - radius - 1), xmax = (int) std::ceil(cx + radius + 1);
  this->clip_box_(xmin, ymin, xmax, ymax);
  for (int y = ymin; y <= ymax; y++) {
    for (int x = xmin; x <= xmax; x++) {
      const float dx = x + 0.5f - cx, dy = y + 0.5f - cy;
      const float cover = clamp01(radius - std::sqrt(dx * dx + dy * dy) + 0.5f);
      if (cover > 0.0f) this->blend_(x, y, r, g, b, (uint32_t) (cover * alpha));
    }
  }
}

void Canvas::ring(float cx, float cy, float r_in, float r_out, float a0, float a1, Rgb c, bool round_caps) {
  if (!all_finite(cx, cy, r_in) || !all_finite(r_out, a0, a1)) return;
  if (this->dry_call_((int) std::floor(cy - r_out) - 1, (int) std::ceil(cy + r_out) + 2, 9, cx, cy, r_in, r_out, a0, a1, c, round_caps)) {
    return;
  }
  const float span = a1 - a0;
  if (span <= 0.0f || r_out <= r_in) return;
  const bool full = span >= 360.0f;
  const uint32_t r = (c >> 16) & 0xFF, g = (c >> 8) & 0xFF, b = c & 0xFF;
  const float deg2rad = 0.017453292f;
  int ymin = (int) std::floor(cy - r_out - 1), ymax = (int) std::ceil(cy + r_out + 1);
  int xmin = (int) std::floor(cx - r_out - 1), xmax = (int) std::ceil(cx + r_out + 1);
  this->clip_box_(xmin, ymin, xmax, ymax);
  for (int y = ymin; y <= ymax; y++) {
    for (int x = xmin; x <= xmax; x++) {
      const float dx = x + 0.5f - cx, dy = y + 0.5f - cy;
      const float d = std::sqrt(dx * dx + dy * dy);
      float cover = clamp01(std::min(d - r_in, r_out - d) + 0.5f);
      if (cover <= 0.0f) continue;
      if (!full) {
        // Angle of this pixel from the start of the arc, clockwise, 0..360
        float t = std::atan2(dy, dx) / deg2rad - a0;
        t -= 360.0f * std::floor(t / 360.0f);
        // Distance in pixels to the nearer end, positive inside the arc
        const float inside = t <= span ? std::min(t, span - t) : -std::min(t - span, 360.0f - t);
        cover *= clamp01(inside * deg2rad * d + 0.5f);
        if (cover <= 0.0f) continue;
      }
      this->blend_(x, y, r, g, b, (uint32_t) (cover * 255.0f));
    }
  }
  if (round_caps && !full) {
    const float mid = (r_in + r_out) / 2.0f, cap = (r_out - r_in) / 2.0f;
    this->disc(cx + mid * std::cos(a0 * deg2rad), cy + mid * std::sin(a0 * deg2rad), cap, c);
    this->disc(cx + mid * std::cos(a1 * deg2rad), cy + mid * std::sin(a1 * deg2rad), cap, c);
  }
}

void Canvas::line(float x0, float y0, float x1, float y1, float width, Rgb c) {
  if (!all_finite(x0, y0, width) || !std::isfinite(x1) || !std::isfinite(y1)) return;
  if (this->dry_call_((int) std::floor(std::min(y0, y1) - width / 2.0f) - 1, (int) std::ceil(std::max(y0, y1) + width / 2.0f) + 2, 10, x0,
                      y0, x1, y1, width, c)) {
    return;
  }
  const uint32_t r = (c >> 16) & 0xFF, g = (c >> 8) & 0xFF, b = c & 0xFF;
  const float half = width / 2.0f;
  const float vx = x1 - x0, vy = y1 - y0;
  const float len2 = vx * vx + vy * vy;
  int ymin = (int) std::floor(std::min(y0, y1) - half - 1), ymax = (int) std::ceil(std::max(y0, y1) + half + 1);
  int xmin = (int) std::floor(std::min(x0, x1) - half - 1), xmax = (int) std::ceil(std::max(x0, x1) + half + 1);
  this->clip_box_(xmin, ymin, xmax, ymax);
  for (int y = ymin; y <= ymax; y++) {
    for (int x = xmin; x <= xmax; x++) {
      const float px = x + 0.5f - x0, py = y + 0.5f - y0;
      const float t = len2 > 0.0f ? clamp01((px * vx + py * vy) / len2) : 0.0f;
      const float dx = px - t * vx, dy = py - t * vy;
      const float cover = clamp01(half - std::sqrt(dx * dx + dy * dy) + 0.5f);
      if (cover > 0.0f) this->blend_(x, y, r, g, b, (uint32_t) (cover * 255.0f));
    }
  }
}

void Canvas::triangle(float x0, float y0, float x1, float y1, float x2, float y2, Rgb c) {
  if (!all_finite(x0, y0, x1) || !all_finite(y1, x2, y2)) return;
  if (this->dry_call_((int) std::floor(std::min({y0, y1, y2})) - 1, (int) std::ceil(std::max({y0, y1, y2})) + 2, 11, x0, y0, x1, y1, x2,
                      y2, c)) {
    return;
  }
  const uint32_t r = (c >> 16) & 0xFF, g = (c >> 8) & 0xFF, b = c & 0xFF;
  const float xs[3] = {x0, x1, x2}, ys[3] = {y0, y1, y2};
  // Which way round the corners go, so "inside" has the same sign for every edge
  const float turn = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0) < 0.0f ? -1.0f : 1.0f;
  int xmin = (int) std::floor(std::min({x0, x1, x2}) - 1), xmax = (int) std::ceil(std::max({x0, x1, x2}) + 1);
  int ymin = (int) std::floor(std::min({y0, y1, y2}) - 1), ymax = (int) std::ceil(std::max({y0, y1, y2}) + 1);
  this->clip_box_(xmin, ymin, xmax, ymax);
  for (int y = ymin; y <= ymax; y++) {
    for (int x = xmin; x <= xmax; x++) {
      float inside = 1e9f;
      for (int i = 0; i < 3; i++) {
        const int j = (i + 1) % 3;
        const float ex = xs[j] - xs[i], ey = ys[j] - ys[i];
        const float len = std::sqrt(ex * ex + ey * ey);
        if (len <= 0.0f) continue;
        const float d = turn * (ex * (y + 0.5f - ys[i]) - ey * (x + 0.5f - xs[i])) / len;
        inside = std::min(inside, d);
      }
      const float cover = clamp01(inside + 0.5f);
      if (cover > 0.0f) this->blend_(x, y, r, g, b, (uint32_t) (cover * 255.0f));
    }
  }
}

// The next code point of a UTF-8 string (0 at the end; bad bytes are skipped)
static uint32_t next_codepoint(const char *&s) {
  while (*s) {
    const uint8_t c = (uint8_t) *s++;
    if (c < 0x80) return c;
    int extra = c >= 0xF0 ? 3 : (c >= 0xE0 ? 2 : (c >= 0xC0 ? 1 : -1));
    if (extra < 0) continue;
    uint32_t cp = c & (0x3F >> extra);
    while (extra-- > 0 && (*s & 0xC0) == 0x80) cp = (cp << 6) | ((uint8_t) *s++ & 0x3F);
    return cp;
  }
  return 0;
}

static const Glyph *find_glyph(const Font &font, uint32_t cp) {
  const uint16_t *end = font.codepoints + font.count;
  const uint16_t *it = std::lower_bound(font.codepoints, end, cp);
  return (it != end && *it == cp) ? &font.glyphs[it - font.codepoints] : nullptr;
}

int Canvas::text_width(const Font &font, const char *utf8) {
  int width = 0;
  while (uint32_t cp = next_codepoint(utf8)) {
    if (const Glyph *gl = find_glyph(font, cp)) width += gl->advance;
  }
  return width;
}

int Canvas::text(const Font &font, int x, int y, const char *utf8, Rgb c, Align align) {
  const int width = text_width(font, utf8);
  if (dry_) {
    // The text itself is part of what is drawn
    uint32_t letters = 2166136261u;
    for (const char *p = utf8; *p; p++) letters = sum_up_(letters, (uint8_t) *p);
    // Which font, not only its size: two fonts of one size look different
    this->dry_call_(y - 2, y + font.line_height() + 2, 12, (uint32_t) (uintptr_t) font.alpha, (int) font.size, x, y, c,
                    (int) align, letters);
    return width;
  }
  int pen = align == LEFT ? x : (align == CENTER ? x - width / 2 : x - width);
  const int baseline = y + font.ascent;
  const uint32_t r = (c >> 16) & 0xFF, g = (c >> 8) & 0xFF, b = c & 0xFF;
  while (uint32_t cp = next_codepoint(utf8)) {
    const Glyph *gl = find_glyph(font, cp);
    if (gl == nullptr) continue;
    const int gx = pen + gl->left, gy = baseline - gl->top;
    if (gx < cx1_ && gx + gl->w > cx0_ && gy < cy1_ && gy + gl->h > cy0_) {
      const int stride = (gl->w + 1) / 2;
      for (int j = 0; j < gl->h; j++) {
        const uint8_t *row = font.alpha + gl->offset + (size_t) j * stride;
        for (int i = 0; i < gl->w; i++) {
          const uint32_t a4 = (i & 1) ? (row[i / 2] & 0x0F) : (row[i / 2] >> 4);
          if (a4) this->blend_(gx + i, gy + j, r, g, b, a4 * 17);
        }
      }
    }
    pen += gl->advance;
  }
  return width;
}

}  // namespace bal
