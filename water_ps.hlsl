// water_ps_wake.hlsl
// Drop-in replacement for water_ps.hlsl in CFD_but_actually_fun.
// Adds the interactive-wake FOAM (white) on top of the existing water shading.
//
// ARCHITECTURE (matches the repo's CPU-displaced water):
//   The water SURFACE displacement (Gerstner + interactive wake field) is done on
//   the CPU in UpdateWater() -> the grid arrives pre-displaced, so the VS is a flat
//   pass-through (unchanged). The wake field is what makes the surface heave in the
//   real V + let the boat ride its own wake. This file adds only the VISUAL foam:
//   a procedural Kelvin V + bow spray, oriented by the boat's forward vector that
//   main.cpp puts in the cbuffer. No texture/SRV/sampler needed -> the ONLY D3D12
//   change is the push-constant cbuffer growing 56 -> 64 floats.
//
// Best-effort: NOT compile-verified on the RHEL dev host (no fxc/D3D). Expect a
// trivial typo; the C++ physics/wake side is Linux-verified.
//
// cbuffer: the original 56 floats are UNCHANGED at the front, so water_vs.hlsl
// (which declares the shorter cbuffer) still works. boatState + wakeParams are
// appended (floats 56..63) -> 64 total, exactly the D3D12 256-byte push-constant limit.

cbuffer RootConstants : register(b0)
{
    row_major float4x4 mvpWater;    // 16
    row_major float4x4 mvpModel;    // 16
    row_major float4x4 model;       // 16
    float4 lightDir;                // 4   (.w unused)
    float4 camPosTime;              // 4   (.w = time)
    float4 boatState;               // 4   .x .z = boat world XZ, .y = fwdX, .w = fwdZ (unit hull forward)
    float4 wakeParams;              // 4   .x = speed (m/s), .y = throttle, .z = foamGain, .w = time
};

struct VSOut
{
    float4 clip   : SV_Position;
    float3 world  : TEXCOORD0;
    float3 normal : TEXCOORD1;
};

// Procedural Kelvin V + bow spray foam, in the boat's frame of reference.
// The V opens BEHIND the boat (opposite its forward) and decays with distance;
// the bow spray is a small bright patch in front of the hull while planing.
float3 WakeFoam(float3 world)
{
    float2 d     = world.xz - boatState.xz;
    float2 fwd   = normalize(float2(boatState.y, boatState.w) + 1e-4);
    float  dist  = length(d);
    float2 dir   = d * (1.0 / max(dist, 1e-4));

    // Kelvin V: half-angle ~ asin(1/pi) ~= 19.5 deg. 1 inside the cone, 0 outside.
    float  cosAng  = dot(dir, -fwd);
    float  inV     = smoothstep(0.88, 0.95, cosAng);        // cos(19.5deg) ~= 0.943
    float  vFade   = exp(-dist * 0.12);                      // wake decays with distance

    // Bow spray: bright, close, in front of the hull while planing.
    float  bowFwd   = dot(dir, fwd);
    float  bowSpray = smoothstep(0.80, 0.98, bowFwd) * (1.0 - smoothstep(0.5, 1.3, dist));

    float  moving   = smoothstep(1.5, 4.5, wakeParams.x);    // only when actually moving
    float  amt      = saturate((inV * vFade * 0.9 + bowSpray * 0.8) * moving * wakeParams.z);
    return float3(0.90, 0.95, 0.99) * amt;
}

float4 main(VSOut input) : SV_Target
{
    float3 N = normalize(input.normal);
    float3 L = normalize(lightDir.xyz);
    float3 V = normalize(camPosTime.xyz - input.world);
    float3 H = normalize(L + V);

    float ndl = saturate(dot(N, L));

    float3 deep = float3(0.01, 0.10, 0.20);
    float3 shallow = float3(0.04, 0.36, 0.44);
    float3 base = lerp(deep, shallow, ndl);

    float spec = pow(saturate(dot(N, H)), 120.0) * 1.6;
    float foam = smoothstep(0.30, 0.48, input.world.y);

    float3 col = base * (0.35 + 0.65 * ndl) + spec;
    col = lerp(col, float3(0.85, 0.92, 0.97), foam * 0.65);

    // ---- interactive wake foam (new) -------------------------------------
    float3 wf = WakeFoam(input.world);
    col = col + wf;                       // additive white in the V + at the bow
    col = lerp(col, col * 0.80 + wf, wf.r * 0.4);   // soften the crest slightly
    return float4(col, 1.0);
}

// ---- crate + jetski shaders: unchanged from water_ps.hlsl ----------------
float4 psCrate(VSOut input) : SV_Target
{
    float3 N = normalize(input.normal);
    float3 L = normalize(lightDir.xyz);
    float ndl = saturate(dot(N, L));

    float3 albedo = float3(0.45, 0.30, 0.16);
    float3 col = albedo * (0.25 + 0.75 * ndl);

    float3 V = normalize(camPosTime.xyz - input.world);
    float3 H = normalize(L + V);
    col += pow(saturate(dot(N, H)), 40.0) * 0.15;

    float wet = smoothstep(0.15, -0.05, input.world.y);
    col = lerp(col, col * 0.35, wet);
    return float4(col, 1.0);
}

struct VSOutJetski
{
    float4 clip   : SV_Position;
    float3 world  : TEXCOORD0;
    float3 normal : TEXCOORD1;
    float3 color  : TEXCOORD2;
};

float4 psJetski(VSOutJetski input) : SV_Target
{
    float3 N = normalize(input.normal);
    float3 L = normalize(lightDir.xyz);
    float ndl = saturate(dot(N, L));

    float3 albedo = input.color;
    float3 col = albedo * (0.25 + 0.75 * ndl);

    float3 V = normalize(camPosTime.xyz - input.world);
    float3 H = normalize(L + V);
    col += pow(saturate(dot(N, H)), 40.0) * 0.15;

    float wet = smoothstep(0.15, -0.05, input.world.y);
    col = lerp(col, col * 0.35, wet);
    return float4(col, 1.0);
}
