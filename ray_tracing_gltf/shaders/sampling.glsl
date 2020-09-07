#define M_PI 3.1415926535

// Makes a random unsigned integer from 2 unsigned int values, using 16 pairs
// of rounds of the Tiny Encryption Algorithm.
// Zafar, Olano, Curtis, "GPU Random Numbers via the Tiny Encryption Algorithm"
uint TinyEncryption(uint val0, uint val1)
{
    uint v0 = val0;
    uint v1 = val1;
    uint s0 = 0;

    for (uint n = 0; n < 16; ++n) {
        s0 += 0x9e3779b9;
        v0 += ((v1 << 4) + 0xa341316c) ^ (v1 + s0) ^ ((v1 >> 5) + 0xc8013ea4);
        v1 += ((v0 << 4) + 0xad90777d) ^ (v0 + s0) ^ ((v0 >> 5) + 0x7e95761e);
    }

    return v0;
}

// Generates a random unsigned int in [0, 2^24) given the previous RNG state
// using the numerical recipes - linear congruential generator
uint LCG(inout uint prev)
{
    uint kLcgA = 1664525u;
    uint kLcgC = 1013904223u;

    prev = (kLcgA * prev + kLcgC);
    return prev & 0x00FFFFFF;
}

float RandF32(inout uint prev)
{
    return (float(LCG(prev)) / float(0x01000000));
}

// Making samples from given random numbers

// Randomly sampling around +Z hemisphere.
vec3 SampleLocalHemi(inout uint seed)
{
    float r1 = RandF32(seed) * 2*M_PI;
    float r2 = RandF32(seed);
    float sq = sqrt(1.0 - r2);

    vec3 direction = vec3(cos(r1) * sq, sin(r1) * sq, sqrt(r2));
    
    return direction;
}

vec3 samplingHemisphere(inout uint seed, in vec3 x, in vec3 y, in vec3 z) {
    vec3 local = SampleLocalHemi(seed);
    return local.x * x + local.y * y + local.z * z;
}


void MakeCoordinateSystem(in vec3 base_x, out vec3 y, out vec3 z) {
    if (abs(base_x.x) > abs(base_x.y)) {
        y = vec3(base_x.z, 0, -base_x.x) / sqrt(base_x.x*base_x.x + base_x.z*base_x.z);
    } else {
        y = vec3(0, -base_x.z, base_x.y) / sqrt(base_x.y*base_x.y + base_x.z*base_x.z);
    }
    z = cross(base_x, y);
}