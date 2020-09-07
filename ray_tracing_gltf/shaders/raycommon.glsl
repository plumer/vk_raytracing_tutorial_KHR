
// Information on the intersection point.
struct hitPayload {

    // False if hits a surface.
    bool hits_background;
    // Non-zero: hit an emissive surface. Zero: hit non-emissive surface.
    bool hits_emissive;  
    
    // If hit an emissive surface, records the radiance.
    // Otherwise, records the evaluated BSDF value.
    vec3 hitValue;       
    
    vec3 position;
    vec3 normal;
    vec3 wo_world;
    vec3 wi_world;
    
    uint seed;
    uint depth;
    vec3 ray_origin;
    vec3 ray_direction;
    vec3 bsdf_weight;
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
