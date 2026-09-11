// wake_field.h
// Interactive wake/ripple heightfield for the CFD jet-ski. Header-only, pure
// ISO C++20, no platform headers. Verified on Linux with boat_sim_test.cpp.
//
// A bounded N x N field solved by the (damped) linear wave equation. The hull
// writes displacement + velocity into it where it enters the water; the field
// propagates and decays, producing a near-field bow wave + V-wake. The vertex
// shader ADDS this field to the Gerstner base ocean (finalY = gerstner + wake).
// The same field is sampled back to let the boat ride its own wake (two-way).
//
// World-anchored: the field covers [x0,x0+S] x [z0,z0+S] in world XZ. For a
// boat that roams, either make S large or re-center the field on the boat each
// frame (see INTEGRATION.md).
#pragma once
#include <cmath>
#include <cfloat>
#include <vector>
#include <algorithm>

namespace boat {

class WakeField {
public:
    // S = domain size (m), N = grid cells per side, c = wave speed (m/s),
    // mu = damping (1/s), sponge = boundary absorption 0..1.
    WakeField(float S = 26.0f, int N = 128, float c = 2.6f, float mu = 1.4f, float sponge = 0.9f)
        : S_(S), N_(N), c_(c), mu_(mu), sponge_(sponge),
          x0_(-S * 0.5f), z0_(-S * 0.5f), cell_(S / N),
          h_(N * N, 0.0f), v_(N * N, 0.0f), lap_(N * N, 0.0f) {}

    int   width()  const { return N_; }
    int   height() const { return N_; }
    float cellSize() const { return cell_; }
    float x0() const { return x0_; }
    float z0() const { return z0_; }
    float size() const { return S_; }
    const float* data() const { return h_.data(); }   // for texture upload
    void clear() { std::fill(h_.begin(), h_.end(), 0.0f); std::fill(v_.begin(), v_.end(), 0.0f); }

    // Bilinear splat: add dH (height) and dV (velocity) centered at world (x,z).
    void inject(float x, float z, float dH, float dV) {
        float fx = (x - x0_) / cell_ - 0.5f;   // cell-space (centered on cell)
        float fz = (z - z0_) / cell_ - 0.5f;
        int i0 = (int)std::floor(fx), j0 = (int)std::floor(fz);
        float tx = fx - i0, tz = fz - j0;
        for (int dj = 0; dj <= 1; ++dj)
            for (int di = 0; di <= 1; ++di) {
                int i = i0 + di, j = j0 + dj;
                if (i < 1 || i >= N_ - 1 || j < 1 || j >= N_ - 1) continue;
                float w = (di ? tx : 1 - tx) * (dj ? tz : 1 - tz);
                int idx = j * N_ + i;
                h_[idx] += dH * w;
                v_[idx] += dV * w;
            }
    }

    // Bilinear sample of the wake height at world (x,z).
    float sample(float x, float z) const {
        float fx = (x - x0_) / cell_ - 0.5f;
        float fz = (z - z0_) / cell_ - 0.5f;
        int i0 = (int)std::floor(fx), j0 = (int)std::floor(fz);
        float tx = fx - i0, tz = fz - j0;
        if (i0 < 0 || i0 >= N_ - 1 || j0 < 0 || j0 >= N_ - 1) return 0.0f;
        float h00 = h_[j0 * N_ + i0],         h10 = h_[j0 * N_ + i0 + 1];
        float h01 = h_[(j0 + 1) * N_ + i0],   h11 = h_[(j0 + 1) * N_ + i0 + 1];
        float a = h00 * (1 - tx) + h10 * tx;
        float b = h01 * (1 - tx) + h11 * tx;
        return a * (1 - tz) + b * tz;
    }
    // Surface normal (for lighting) from the field gradient.
    void normal(float x, float z, float* outNx, float* outNy, float* outNz) const {
        float e = cell_ * 0.5f;
        float hx0 = sample(x - e, z), hx1 = sample(x + e, z);
        float hz0 = sample(x, z - e), hz1 = sample(x, z + e);
        float dx = (hx1 - hx0) / (2 * e);
        float dz = (hz1 - hz0) / (2 * e);
        float inv = 1.0f / std::sqrt(1 + dx * dx + dz * dz);
        *outNx = -dx * inv; *outNy = inv; *outNz = -dz * inv;
    }

    // Advance the field by dt (CFL-safe: caller must keep dt <= cell/(c*sqrt2)).
    void step(float dt) {
        const float c2dt2 = (c_ * dt / cell_) * (c_ * dt / cell_);
        // Laplacian (5-point), Neumann (zero-gradient) at the clamp edges.
        for (int j = 0; j < N_; ++j)
            for (int i = 0; i < N_; ++i) {
                int idx = j * N_ + i;
                float c  = h_[idx];
                float L  = h_[j * N_ + (i > 0 ? i - 1 : i)];
                float R  = h_[j * N_ + (i < N_ - 1 ? i + 1 : i)];
                float D  = h_[(j > 0 ? j - 1 : j) * N_ + i];
                float U  = h_[(j < N_ - 1 ? j + 1 : j) * N_ + i];
                lap_[idx] = (L + R + U + D - 4.0f * c);
            }
        // Sponge mask (absorbing boundary) precomputed on the fly.
        for (int j = 0; j < N_; ++j)
            for (int i = 0; i < N_; ++i) {
                int idx = j * N_ + i;
                float damp = 1.0f;
                int edge = std::min(std::min(i, N_ - 1 - i), std::min(j, N_ - 1 - j));
                int band = std::max(1, N_ / 16);
                if (edge < band) damp = sponge_ * (edge / (float)band);   // 0..sponge at rim
                float a = c2dt2 * lap_[idx] - mu_ * v_[idx];
                v_[idx] = (v_[idx] + a * dt) * damp;
                h_[idx] += v_[idx] * dt;
            }
    }

   // Boat-locked scroll: the field is centered on the boat; when the boat moves
    // by (dx,dz) in world XZ, shift the field content so the wake trails it.
    // newField(q) = oldField(q + (dx,dz)). Bilinear resample.
    void scroll(float dx, float dz) {
        float ox = dx / cell_, oz = dz / cell_;
        if (ox == 0.0f && oz == 0.0f) return;
       auto samp = [&](const std::vector<float>& g, float fi, float fj) -> float {
            int i0 = (int)std::floor(fi), j0 = (int)std::floor(fj);
            // Guard on the FLOORED corner (matches sample()): keeps i0+1, j0+1 in [0, N-1].
            if (i0 < 0 || i0 >= N_ - 1 || j0 < 0 || j0 >= N_ - 1) return 0.0f;
            float tx = fi - i0, tz = fj - j0;
            float a = g[j0 * N_ + i0] * (1 - tx) + g[j0 * N_ + i0 + 1] * tx;
            float b = g[(j0 + 1) * N_ + i0] * (1 - tz) + g[(j0 + 1) * N_ + i0 + 1] * tx;
            return a * (1 - tz) + b * tz;
        };
        std::vector<float> nh(h_.size()), nv(v_.size());
        for (int j = 0; j < N_; ++j)
            for (int i = 0; i < N_; ++i) {
                nh[j * N_ + i] = samp(h_, (float)i + ox, (float)j + oz);
                nv[j * N_ + i] = samp(v_, (float)i + ox, (float)j + oz);
            }
        h_.swap(nh); v_.swap(nv);
    }

private:
    float S_;
    int N_;
    float c_, mu_, sponge_, x0_, z0_, cell_;
    std::vector<float> h_, v_, lap_;
};

}  // namespace boat
