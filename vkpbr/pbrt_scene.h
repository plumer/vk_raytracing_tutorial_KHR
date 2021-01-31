#ifndef NVCOPY_PBRT_SCENE_H_
#define NVCOPY_PBRT_SCENE_H_

#include <filesystem>
#include <glm/glm.hpp>
#include <map>

#include "../common/types.h"
#include "parser/tree.h"

namespace vkpbr {

// Stores the geometric data for a triangular mesh.
struct PlyMesh {
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec2> texture_uvs;
    std::vector<i32>       indices;
    int                 material_index = -1;
};

/** 
 * @brief Stores the geometric data for a Vertex.
 * A triangular mesh can be represented with an array of objects of this structure, paired with
 * an array of triangle indices. 
 * \see PlyMesh for a different usage.
 */
struct VertexData {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 texture_uv;
};

struct MeshInstance {
    int    mesh_index     = -1;
    int    material_index = -1;
    glm::mat4 obj_to_world   = glm::mat4(1.0f);
};

// Records the recipe for an object - a collection of mesh instances.
struct Object {
    std::vector<MeshInstance> mesh_instances;
};

// PBRT-style mapping.
struct Mapping {
    enum class Type { eUV, eSphere, eCylinder, ePlanar } type;
    float          uscale = 0.0, vscale = 0.0, udelta = 0.0, vdelta = 0.0;
    static Mapping UV(float uscale, float vscale, float udelta, float vdelta)
    {
        Mapping m{Type::eUV, uscale, vscale, udelta, vdelta};
        return m;
    }
};

// Stores pixel data and format for an image.
struct ImageData {
    std::vector<u8> data;
    u32             width = 0, height = 0;
    int             num_channels;
    enum Depth { eU8, eF32 } depth;
    std::string TypeName() const {
        static const char * kDepthNames[] = {"U8", "F32"};
        std::string depth_name = kDepthNames[cast_u32(depth)];
        return depth_name +  'C' + std::to_string(num_channels);
    }
};

// PBRT-style texture.
struct Texture {
    enum class Type { eNone, eConstant, eCheckered, eBilinear, eSampledImage } type = Type::eNone;
    const char * TypeName() const {
        static const char * names[] = {"None", "Constant", "Checkered", "Bilinear", "SampledImage"};
        return names[cast_u32(type)];
    }
    glm::vec3 constant_color = glm::vec3{0.0f};
    glm::vec3 bilinear_colors[4] = {glm::vec3{0.0f}};
    int    checker_indices[2] = {-1, -1};
    int    sampler            = -1;

    static Texture Constant(glm::vec3 color)
    {
        Texture tex;
        tex.type           = Type::eConstant;
        tex.constant_color = color;
        return tex;
    }
    static Texture Bilinear(glm::vec3 color0, glm::vec3 color1, glm::vec3 color2, glm::vec3 color3)
    {
        Texture tex;
        tex.type               = Type::eBilinear;
        tex.bilinear_colors[0] = color0;
        tex.bilinear_colors[1] = color1;
        tex.bilinear_colors[2] = color2;
        tex.bilinear_colors[3] = color3;

        return tex;
    }
};

// PBRT-style material.
struct Material {
    enum class Type : int { eMatte, ePlastic, eMetal, eMirror, eGlass, eSubstrate, eUber } type;
    const char *TypeName() const {
        static const char * names[] = {"Matte", "Plastic", "Metal", "Mirror", "Glass", "Uber"};
        return names[cast_u32(type)];
    }
    glm::vec3 diffuse             = glm::vec3{0.0f};
    int       diffuse_tex_id      = -1;  // eMatte, ePlastic, eSubstrate
    glm::vec3 specular            = glm::vec3{0.0f};
    int       specular_tex_id     = -1;  // eMirror, ePlastic, eSubstrate
    glm::vec3 reflective          = glm::vec3{0.0f};
    int       reflective_tex_id   = -1;  // eUber
    glm::vec3 transmissive        = glm::vec3{0.0f};
    int       transmissive_tex_id = -1;  // eUber
    glm::vec3 emission            = glm::vec3{0.0f};

    glm::vec3 eta                 = glm::vec3{0.0f}; // Metal & Dielectric
    glm::vec3 eta_imaginary       = glm::vec3{0.0f}; // Metal
    float roughness_sigma     = 0.0;  // eMatte
    float u_roughness         = 0.0;  // ePlastic
    float v_roughness         = 0.0;  // ePlastic
    explicit Material(Type t)
        : type(t)
    {}
    i32 __dummy = 0;

    static_assert(sizeof(type) == 4, "type doesn't take 4 bytes");
};

// PBRT-style light.
struct Light {
    enum class Type { eError, ePoint, eDistant, eInfinite, eSpot, eArea } type;
    const char *TypeName() const {
        static const char * names[] = {"Error", "Point", "Distant", "Infinite", "Spot", "Area"};
        return names[cast_u32(type)];
    }
    glm::vec3 illuminance = {0, 0, 0};
    glm::mat4 transform;
    glm::vec3 direction;
    // For InfiniteAreaLight. If -1, look for illuminance.
    int envmap_image_index = -1;
    
    // For spot light.
    float cone_angle_degree = 0.0f;
    float cone_delta_angle_degree = 0.0f;
    explicit Light(Type t)
        : type(t)
    {}
    static Light Point(glm::vec3 illuminance_, glm::mat4 transform_)
    {
        Light p(Type::ePoint);
        p.illuminance = illuminance_;
        p.transform   = transform_;
        return p;
    }
};

class Scene
{
  public:
    // Loads the scene from a ".pbrt" file. Instantiates a `SceneLoader` object
    // to do the work. Writes to all members.
    bool LoadFromFile(const char* pbrt_file);

    // Loads the scene from a parsed ".pbrt" file in the form of an AST.
    // TODO(zixun): it's possible to only take the WORLD part of the AST.
    bool LoadFromAst(const scene_t* root, const std::filesystem::path& root_path);

    // Stores all primitives, with their underlying shapes and materials etc.
    // For all triangle meshes, a shared pointer is stored in `loaded_meshes`
    // organized as bounding-volume hierarchies (working like low-level accel
    // structures in Vulkan RT extension).
    // std::vector<std::shared_ptr<GeometricPrimitive>> primitives;

    // Caches the meshes loaded from file, organized as TriangleBVH.
    // The meshes are stored with associated material in GeometricPrimitive.
    // std::map<std::string, std::shared_ptr<TriangleBVH>> loaded_meshes;

    // bbox3 world_bound() const;
    // bool intersectP(const Ray &r) const;

    /** Performs ray-primitive intersection to all primitives and returns the
     * intersection information.
     * @param isect where intersection info is written if ray hits an object.
     * @return if the ray hits any object in the scene.
     * TODO(zixun): organizes all primitives as a "top-level accel structure".
     */
    // bool intersect(const Ray &r, SurfaceInteraction * isect) const;

    // Light objects.
    // std::vector<std::shared_ptr<Light>> lights;

    // From 2020-09-27

    // Caches the named objects by directives `ObjectBegin`/`ObjectEnd`
    std::vector<Object>        object_descriptors_;
    std::map<std::string, int> named_objects_;

    std::vector<Texture>       texture_descriptors_;
    std::map<std::string, int> named_textures_;

    std::vector<PlyMesh>       meshes_;
    std::map<std::string, int> named_meshes_descriptors_;

    std::vector<Material>      material_descriptors_;
    std::map<std::string, int> named_materials_;

    std::vector<Light> lights_;

    // Stores all image data read from files.
    std::vector<ImageData> image_contents_;

    std::vector<MeshInstance> mesh_instances_;

  private:

    // std::map<std::string, std::shared_ptr<tex::Texture<float>>>    named_float_textures_;
    // std::map<std::string, std::shared_ptr<tex::Texture<Spectrum>>> named_color_textures_;
    // std::map<std::string, std::shared_ptr<Material>>               named_materials_;

    // helper functions
    bool traverse_tree();
    bool traverse_worlditem(const worlditem_node_t*);
    // all Primitives will be put into `object_builder`
    bool create_object(const worlditem_node_t*);
};

class SceneLoader
{
  public:
    void SetRootScene(const scene_t* s) { scene_root_ = s; }
    bool traverse_tree();
    bool TraverseWorldItem(const worlditem_node_t& wi);
    bool CreateObject(const std::vector<worlditem_node_t>& children, const std::string& name);

    Light ParseLight(const std::vector<param_node_t>& params, const std::string& implementation);

    // Creates a texture based on implementation and parameters.
    // If additional textures need to be created, they are added implicitly as well.
    // Returns the index to the texture_descriptors_.
    int ParseTexture(const std::vector<param_node_t>& parameters,
                     const std::string&               implementation);

    /**
     * Creates a Material based on implementation and parameters, and returns the index to
     * material_descriptors_.
     * If additional textures are needed, they are added to texture_descriptor_ implicitly.
     * \return
     */
    int ParseMaterial(const std::vector<param_node_t>& params, const std::string& impl);

    // Makes a shape from the parameters, adds it to meshes_, and returns its index in meshes_.
    // Currently supports triangle meshes, either from pbrt file or .obj/.ply file.
    int ParseShape(const std::vector<param_node_t>& parameters, const std::string& implementation);

    /**
     * Returns the index to the named textures. Either numbers or strings is empty.
     *
     * \param numbers If non empty, returns a constant texture.
     * \param strings If non
     * empty, looks for the named texture.
     * \param default_value
     * \return
     */
    int ConstantOrImageTexture(const std::vector<float>&       numbers,
                               const std::vector<std::string>& strings, glm::vec3 default_value);

  private:
    const scene_t* scene_root_ = nullptr;

    // Stacks of loader's current states.
    std::vector<bool>      reverse_orientation_stack;
    std::vector<glm::mat4> ctm_stack_;
    int                 current_material_          = kNotFound;
    int                 current_emitting_material_ = kNotFound;

    // Records the path of the root PBRT file. It is used to compute paths to
    // images/plymesh files when needed.
    std::filesystem::path pbrt_file_root_path_;

    // Caches the named objects by directives `ObjectBegin`/`ObjectEnd`
    std::vector<Object>        object_descriptors_;
    std::map<std::string, int> named_objects_;

    std::vector<Texture>       texture_descriptors_;
    std::map<std::string, int> named_textures_;

    std::vector<PlyMesh>       meshes_;
    std::map<std::string, int> named_meshes_descriptors_;  // Maps from file name to mesh.

    std::vector<Material>      material_descriptors_;
    std::map<std::string, int> named_materials_;

    std::vector<Light> lights_;

    // Stores all image data read from files.
    std::vector<ImageData> image_contents_;

    std::vector<MeshInstance> mesh_instances_;

    // temporary collection of primitives as a named object.
    // Only non-empty when parsing `object[begin/end]` pairs in an input file.
    // std::vector<std::shared_ptr<Primitive>> object_builder_;

    // The loading process would be handled by a different class.
    // This loader class would take care of the parsing AST, parsing context
    // (CTM stack and material stack) and some other stuff.
    friend class Scene;
    static constexpr int kNotFound = -1;
};


}  // namespace vkpbr


#endif  // NVCOPY_PBRT_SCENE_H_
