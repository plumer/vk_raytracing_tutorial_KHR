
// Information on the intersection point.
struct hitPayload {
    int  hits_emissive;  // Non-zero: hit an emissive surface. Zero: hit non-emissive surface.
    vec3 hitValue;       // If hit an emissive surface, records the radiance.

    vec3 position;
    vec3 normal;
    vec3 wo_world;
    vec3 wi_world;
};

struct Sphere {
    vec3  center;
    float radius;
};

struct Aabb {
    vec3 minimum;
    vec3 maximum;
};

vec3 BarycentricInterpolate(vec3 bc_coords, vec3 v0, vec3 v1, vec3 v2) {
    return bc_coords.x * v0 + bc_coords.y * v1 + bc_coords.z * v2;
}

vec2 BarycentricInterpolate(vec3 bc_coords, vec2 v0, vec2 v1, vec2 v2) {
return bc_coords.x * v0 + bc_coords.y * v1 + bc_coords.z * v2;
}

#define KIND_SPHERE 0
#define KIND_CUBE 1
