cbuffer RootConstants : register(b0)
{
    row_major float4x4 mvpWater;
    row_major float4x4 mvpModel;
    row_major float4x4 model;
    float4 lightDir;
    float4 camPosTime;
};

struct VSIn
{
    float3 position : POSITION;
    float3 normal : NORMAL;
};

struct VSOut
{
    float4 clip : SV_Position;
    float3 world : TEXCOORD0;
    float3 normal : TEXCOORD1;
};

// Water vertices are already in world space.
VSOut main(VSIn input)
{
    VSOut o;
    o.world = input.position;
    o.normal = input.normal;
    o.clip = mul(float4(input.position, 1.0), mvpWater);
    return o;
}

// Crate vertices are in body space; transform them.
VSOut mainModel(VSIn input)
{
    VSOut o;
    o.world = mul(float4(input.position, 1.0), model).xyz;
    o.normal = normalize(mul(input.normal, (float3x3)model));
    o.clip = mul(float4(input.position, 1.0), mvpModel);
    return o;
}
