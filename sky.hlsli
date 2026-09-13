// sky.hlsli — procedural sky model for CFD_but_actually_fun.
//
// A physically-motivated, ASSET-FREE sky (no textures): a fullscreen triangle
// reconstructs the world-space view ray per pixel, then colors it from a compact
// Rayleigh/Mie single-scattering approximation. The key phenomena, grounded in
// real atmospheric optics:
//   * Rayleigh (molecular) scattering ~ 1/lambda^4 -> the BLUE sky (blue > green > red).
//   * Mie (aerosol) scattering, Henyey-Greenstein forward lobe -> the bright GLOW
//     around the sun and the whitened HORIZON haze.
//   * Beer-Lambert sun attenuation over a slant path -> the SUNSET color shift
//     (long low-sun path removes blue, leaving red/orange).
//   * Exponential altitude falloff -> the horizon is hazier than the zenith.
//   * Time-of-day -> the sun's elevation/azimuth drive everything (day blue,
//     sunset orange, night dark).
//   * Procedural FBM clouds, sun-lit, catching fire at sunset.
//
// The "vista" is defined by the SkyStyle parameters (packed into the sky root
// constant, register b1) so alternate vistas are just different parameter sets
// (see the C++ presets in main.cpp).
//
// Pipeline note: the target build uses a plain R8G8B8A8_UNORM RTV (no exposure
// curve / sRGB encode), so all colors are in LDR display space and values >1
// clamp to white (the sun + its glow read as white-hot — as they should).
//
// Included by water_vs.hlsl (SkyVSOut + skyVS) and water_ps.hlsl (SkyColor + skyPS).

// Fullscreen-triangle VS output (also the sky PS input).
struct SkyVSOut
{
    float4 pos : SV_Position;
    float2 ndc : TEXCOORD0;
};

// ---------------------------------------------------------------------------
// Sky ray reconstruction + style, in a dedicated root constant (register b1).
// The C++ packs exactly this 48-float layout into g_skyRc every frame:
//   [0..3]  rightTan (cameraRight * tan(fovX/2))
//   [4..7]  upTan    (cameraUp    * tan(fovY/2))
//   [8..11] fwd      (cameraForward)
//   [12..15]sun      (.xyz = sunDir, .w = sunIntensity)
//   [16..19]skyTime  (.x = time in seconds, for cloud drift)
//   [20..47]SkyStyle (see below)
// ---------------------------------------------------------------------------
cbuffer SkyRoot : register(b1)
{
    float4 rightTan;      // [0]
    float4 upTan;         // [4]
    float4 fwd;           // [8]
    float4 sun;           // [12]  xyz=sunDir  w=sunIntensity
    float4 skyTime;       // [16]  x=time
    float4 dayZenith;     // [20]  Caribbean blue at noon
    float4 dayHorizon;    // [24]  pale cyan-white at noon
    float4 sunsetZenith;  // [28]  indigo at sunset
    float4 sunsetHorizon; // [32]  orange-pink at sunset
    float4 styleA;        // [36]  x=gradientFalloff y=haze z=mieStrength w=rayleighScale
    float4 styleB;        // [40]  x=sunDisc y=cloudCoverage z=cloudSpeed w=cloudScale
    float4 palette;       // [44]  xyz = overall tint
};

// Reconstruct the world-space view ray from NDC + the camera axes.
float3 SkyViewDir(float2 ndc)
{
    return normalize(fwd.xyz + rightTan.xyz * ndc.x + upTan.xyz * ndc.y);
}

// ---- value noise + FBM (cheap, deterministic) ------------------------------
float SkyHash21(float2 p)
{
    return frac(sin(dot(p, float2(127.1, 311.7))) * 43758.5453);
}
float SkyVNoise(float2 p)
{
    float2 i = floor(p);
    float2 f = frac(p);
    float2 u = f * f * (3.0 - 2.0 * f);
    float a = SkyHash21(i);
    float b = SkyHash21(i + float2(1.0, 0.0));
    float c = SkyHash21(i + float2(0.0, 1.0));
    float d = SkyHash21(i + float2(1.0, 1.0));
    return a + (b - a) * u.x + (c - a) * u.y + (a - b - c + d) * u.x * u.y;
}
float SkyFBM4(float2 p)
{
    float val = 0.0;
    float amp = 0.5;
    for (int i = 0; i < 4; ++i)
    {
        val += amp * SkyVNoise(p);
        p *= 2.03;
        amp *= 0.5;
    }
    return val;
}

// Procedural clouds: direction-based FBM "painted" on the dome, sun-lit, with a
// sunset catch-fire on the cloud tops facing the low sun. Returns the blended
// sky+cloud color (no-op below the horizon).
float3 SkyClouds(float3 skyCol, float3 dir)
{
    if (dir.y <= 0.02) return skyCol;

    float3 sunDir = normalize(sun.xyz);
    float  sunI   = sun.w;
    float  dayF   = smoothstep(-0.05, 0.15, sunDir.y);
    float  sunsetT= (1.0 - saturate(sunDir.y / 0.30)) * dayF;

    // Gnomonic dome projection + time drift.
    float2 cuv = dir.xz / max(dir.y, 0.05) * styleB.w;   // cloudScale
    cuv += float2(skyTime.x * styleB.z, skyTime.x * styleB.z * 0.5);

    float   d     = SkyFBM4(cuv);
    float   cover = smoothstep(1.0 - styleB.y * 1.1, 1.0 - styleB.y * 0.4, d);
    if (cover <= 0.0) return skyCol;

    float  sunUp  = saturate(sunDir.y * 0.5 + 0.5);
    float3 lit    = lerp(float3(1.0, 0.98, 0.94), sunsetHorizon.xyz * 1.05, sunsetT);
    float3 shad   = lerp(float3(0.50, 0.55, 0.62), sunsetZenith.xyz * 1.6, sunsetT);
    float  shade  = 0.45 + 0.55 * saturate(d * 1.4);
    float3 cloud  = lerp(shad, lit, shade * sunUp);

    // Sunset: cloud tops facing the low sun catch fire (horizontal alignment).
    float2 hd = normalize(dir.xz + 1e-4);
    float2 sd = normalize(sunDir.xz + 1e-4);
    float  edge = pow(saturate(dot(hd, sd)), 3.0) * sunsetT;
    cloud += sunsetHorizon.xyz * (edge * 0.6);

    // Thin near the horizon and near the zenith.
    float fade = smoothstep(0.02, 0.14, dir.y) * (1.0 - 0.5 * smoothstep(0.5, 1.0, dir.y));
    return lerp(skyCol, cloud, cover * fade * 0.92);
}

// The full sky color for a world-space view direction. Reads sun + style from the
// SkyRoot cbuffer. LDR display space (values may exceed 1 near the sun and clamp).
float3 SkyColor(float3 dir)
{
    dir = normalize(dir);
    float3 sunDir  = normalize(sun.xyz);
    float  sunI    = sun.w;

    const float dayF   = smoothstep(-0.05, 0.15, sunDir.y);
    const float mu     = dot(dir, sunDir);
    const float h      = saturate(dir.y);
    const float sunsetT= (1.0 - saturate(sunDir.y / 0.30)) * dayF;

    // --- base vertical gradient (palette; day<->night, then day<->sunset) ---
    float3 nightZenith  = dayZenith.xyz  * 0.04 + 0.004;
    float3 nightHorizon = dayHorizon.xyz * 0.05 + 0.004;
    float3 zenith  = lerp(nightZenith,  dayZenith.xyz,  dayF);
    float3 horizon = lerp(nightHorizon, dayHorizon.xyz, dayF);
    zenith  = lerp(zenith,  sunsetZenith.xyz,  sunsetT);
    horizon = lerp(horizon, sunsetHorizon.xyz, sunsetT);
    float3 col = lerp(horizon, zenith, pow(h, styleA.x));

    // --- horizon haze (Mie): more atmosphere in the line of sight near horizon ---
    const float viewAir = 1.0 / (h + 0.10);
    const float hazeAmt = styleA.y * saturate(viewAir * 0.12) * (0.3 + 0.7 * dayF);
    float3 hazeCol = lerp(dayHorizon.xyz, float3(1.0, 1.0, 1.0), 0.35);
    hazeCol = lerp(hazeCol, sunsetHorizon.xyz, sunsetT * 0.7);
    col = lerp(col, hazeCol, hazeAmt);

    // --- sun glow (Mie forward scatter, Henyey-Greenstein lobe) ---
    const float g = 0.72;
    const float gg = g * g;
    const float pMie = (3.0 / (8.0 * 3.14159265)) * ((1.0 - gg) * (mu * mu + 1.0))
                       / (pow(1.0 + gg - 2.0 * mu * g, 1.5) * (2.0 + gg));
    float3 glowCol = lerp(float3(1.0, 0.93, 0.78), sunsetHorizon.xyz, sunsetT * 0.8);
    col += glowCol * (pMie * styleA.z * sunI * 0.6);

    // --- Rayleigh blue boost (molecular): vivid blue away from the sun ---
    const float pRlh = (3.0 / (16.0 * 3.14159265)) * (1.0 + mu * mu);
    col += dayZenith.xyz * (pRlh * styleA.w * dayF * (0.2 + 0.8 * h) * (1.0 - 0.5 * pMie));

    // --- sun disc ---
    const float disc = pow(saturate(mu), 900.0);
    float3 core = lerp(float3(1.0, 1.0, 0.97), float3(1.0, 0.8, 0.5), sunsetT);
    col += core * (disc * styleB.x * sunI);

    // --- clouds ---
    col = SkyClouds(col, dir);

    // --- below horizon: fade to a dark haze (the water overdraws the surface) ---
    const float below = smoothstep(0.0, -0.06, dir.y);
    col = lerp(col, col * 0.5 + hazeCol * 0.15, below);

    col *= palette.xyz;
    return max(col, 0.0);
}
