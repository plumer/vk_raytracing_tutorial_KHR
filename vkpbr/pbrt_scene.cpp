#include "pbrt_scene.h"

#include <cctype>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
#include <set>

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

#include "../parser/tokenizer.h"
#include "hello_vulkan.h"
#include "logging.h"
#include "meshloader.h"
#include "spectrum.hpp"
#include "stb_image.h"
#include "vk_memory.h"

using Transform = glm::mat4;

glm::vec3 FromRgb(const std::vector<float>& values)
{
    if (values.size() == 1) {
        return glm::vec3(values[0], values[0], values[0]);
    }
    CHECK_EQ(values.size(), 3);
    return glm::vec3(values[0], values[1], values[2]);
}
glm::vec3 FromXyz(const std::vector<float>& values)
{
    CHECK_EQ(values.size(), 3);
    RGBSpectrum rgb = RGBSpectrum::FromXyz(values.data());
    glm::vec3   result;
    rgb.ToRgb(&result.x);
    return result;
}


// +------------------------------------------------------------------+
// | Parsing utility function
// +------------------------------------------------------------------+

namespace {

bool contains(const std::string& haystack, const char* needle)
{
    return haystack.find(needle) != std::string::npos;
}

/**
 * Interprets a sequence of numbers as a color.
 *
 * \param numbers
 * \param key Encoding of numbers. Can be: "rgb", "xyz", "blackbody" or "spectrum".
 * \return The color in RGB space (not necessarily clamped to 1.0).
 */
glm::vec3 ParseSpectrum(const std::vector<float>& numbers, const std::string& key)
{
    if (contains(key, "rgb")) {
        return FromRgb(numbers);
    } else if (contains(key, "xyz")) {
        return FromXyz(numbers);
    } else if (contains(key, "blackbody")) {
        float           temperature = numbers[0], coeff = numbers[1];
        SampledSpectrum full_spectrum;
        float           lambdas[kNSpectralSamples];
        for (int i = 0; i < kNSpectralSamples; ++i) {
            lambdas[i] = cast_f32(kSampledLambdaStart + i * kSampleWidth);
        }
        BlackbodyNormalized(lambdas, kNSpectralSamples, temperature, &full_spectrum[0]);
        float xyz[3];
        full_spectrum.ToXyz(xyz);
        auto      rgb_spectrum = RGBSpectrum::FromXyz(xyz);
        glm::vec3 result;
        rgb_spectrum.ToRgb(&result.x);
        return result;
    } else if (contains(key, "spectrum")) {
        std::vector<float> lambdas, values;
        for (size_t i = 0; i < numbers.size() / 2; ++i) {
            lambdas.push_back(numbers[2 * i + 0]);
            values.push_back(numbers[2 * i + 1]);
        }
        auto sspec = SampledSpectrum::FromSampled(lambdas.data(), values.data(), lambdas.size());
        LOG(ERROR) << "unimplemented: RGBSpectrum::from_sampled";
        return glm::vec3(0.f);
    } else {
        LOG(ERROR) << "unimplemented key: " << key;
        return glm::vec3(0.f);
    }
}

template <typename T>
T parse_color(const std::vector<float>& numbers, const std::string& key)
{
    static_assert(sizeof(T) == 0, "Use overloaded versions");
    return T(0.f);
}
template <>
float parse_color(const std::vector<float>& numbers, const std::string& key)
{
    CHECK_EQ(numbers.size(), 1);
    return numbers.front();
}
template <>
glm::vec3 parse_color(const std::vector<float>& numbers, const std::string& key)
{
    return ParseSpectrum(numbers, key);
}

// Makes a 4x4 transform matrix from trans parameters.
Transform ParseTransform(const transform_node_t* trans)
{
    switch (trans->t) {
        case transform_node_t::type::Identity:
            return glm::mat4(1.0f);
        case transform_node_t::type::Translate: {
            float x = trans->numbers[0], y = trans->numbers[1], z = trans->numbers[2];
            return glm::translate(glm::vec3{x, y, z});
        }
        case transform_node_t::type::Scale: {
            float x = trans->numbers[0], y = trans->numbers[1], z = trans->numbers[2];
            return glm::scale(glm::vec3{x, y, z});
        }
        case transform_node_t::type::Rotate: {
            float degree = trans->numbers[0], x = trans->numbers[1], y = trans->numbers[2],
                  z = trans->numbers[3];
            return glm::rotate(glm::radians(degree), glm::vec3{x, y, z});
        }
        case transform_node_t::type::LookAt: {
            const auto n = trans->numbers;
            glm::vec3  from{n[0], n[1], n[2]};
            glm::vec3  to{n[3], n[4], n[5]};
            glm::vec3  up{n[6], n[7], n[8]};
            return glm::lookAt(from, to, up);
        }
        case transform_node_t::type::CoordSys:
        case transform_node_t::type::CoordSysTrans: {
            return Transform(1.0f);
        }
        case transform_node_t::type::Mat4:
        case transform_node_t::type::ConcatMat4: {
            glm::mat4 m;
            memcpy(&m[0][0], reinterpret_cast<const float*>(trans->numbers.data()), sizeof(m));
            m = glm::transpose(m);
            // glm::mat4 m{ reinterpret_cast<const float*>(trans->numbers.data()), true };
            return m;
        }
        default:
            LOG(ERROR) << "unhandled type";
            return Transform(1.0f);
    }
}

bool MakeCoordSystem(const glm::vec3& x, glm::vec3* y, glm::vec3* z)
{

    int min_magnitude_dim = 0;
    if (std::abs(x.y) < std::abs(x[min_magnitude_dim]))
        min_magnitude_dim = 1;
    if (std::abs(x.z) < std::abs(x[min_magnitude_dim]))
        min_magnitude_dim = 2;

    int dim_1 = (min_magnitude_dim + 1) % 3;
    int dim_2 = (min_magnitude_dim + 2) % 3;
    // x[0] x[1] x[2]
    // 0   -x[2] x[1]
    glm::vec3 u;
    u[min_magnitude_dim] = 0.f;
    u[dim_1]             = -x[dim_2];
    u[dim_2]             = x[dim_1];
    if (std::abs(glm::dot(u, x)) > 1e-6f)
        return false;
    (*y) = glm::normalize(u);
    (*z) = glm::cross(glm::normalize(x), *y);

    return true;
}

}  // anonymous namespace

namespace fs = std::filesystem;

namespace vkpbr {


// +------------------------------------------------------------------+
// | SceneLoader methods
// +------------------------------------------------------------------+

// Based on either numbers or strings, creates a constant texture or retrieves an image texture from
// `named_textures_`, and returns the index of the found/created texture descriptor.
size_t SceneLoader::ConstantOrImageTexture(const std::vector<float>&       numbers,
                                           const std::vector<std::string>& strings,
                                           glm::vec3                       default_value)
{
    if (!numbers.empty()) {
        glm::vec3 Kr_value = FromRgb(numbers);
        texture_descriptors_.emplace_back(Texture::Constant(Kr_value));
        return texture_descriptors_.size() - 1;
    } else if (!strings.empty()) {
        std::string tex_name = strings.front();
        auto        iter     = named_textures_.find(tex_name);
        CHECK(iter != named_textures_.end()) << "can't find texture named " << tex_name;
        return iter->second;
    } else {
        texture_descriptors_.emplace_back(Texture::Constant(default_value));
        return texture_descriptors_.size() - 1;
    }
    return static_cast<size_t>(-1);
}


// Creates a Light object with the parameters and implementation.
// Per documentation, `current transform matrix` is applied to light objects.
Light SceneLoader::ParseLight(const std::vector<param_node_t>& params,
                              const std::string&               implementation)
{
    const auto& ctm = ctm_stack_.back();
    if (implementation == "point") {
        glm::vec3 illuminance = glm::vec3(1.0f, 1.0f, 1.0f);
        glm::vec3 location    = glm::vec3(0.0, 0.0, 0.0);

        // reads from param node.
        for (const auto& param : params) {
            CHECK_EQ(param.numbers.size(), 3) << "point light params doesn't have numbers";
            const auto& numbers = param.numbers;
            if (contains(param.key, "from")) {
                location = glm::vec3(numbers[0], numbers[1], numbers[2]);
            } else if (contains(param.key, "I")) {
                illuminance = ParseSpectrum(param.numbers, param.key);
            } else {
                LOG(ERROR) << "point light: unrecognized param: " << param.key;
            }
        }

        Transform translation = glm::translate(location);
        return Light::Point(illuminance, translation);
    } else if (implementation == "distant") {
        /**
         *  spectrum L	rgb (1 1 1)	The radiance emitted from the light source.
            point	from	(0,0,0)	"from" and "to" define the direction vector.
            point	to	(0,0,1)
         */
        glm::vec3 radiance;
        glm::vec3 from = glm::vec3(0, 0, 0);
        glm::vec3 to   = glm::vec3(0, 0, 1);

        for (const auto& param : params) {
            CHECK(!param.numbers.empty()) << "distant light params doesn't have numbers";
            if (contains(param.key, "L")) {
                radiance = ParseSpectrum(param.numbers, param.key);
            } else if (contains(param.key, "from")) {
                auto numbers = param.numbers;
                CHECK_EQ(numbers.size(), 3);
                from = glm::vec3(numbers[0], numbers[1], numbers[2]);
            } else if (contains(param.key, "to")) {
                auto numbers = param.numbers;
                CHECK_EQ(numbers.size(), 3);
                to = glm::vec3(numbers[0], numbers[1], numbers[2]);
            } else {
                LOG(FATAL) << "distant light: unrecognized param: " << param.key;
            }
        }

        glm::vec3 dir = to - from;

        // TODO(zixun): Is the distant light direction the opposite?
        // DistantLight's direction parameter is wi, so incident direction, opposite to the light's
        // direction.
        // auto  distant_l = std::shared_ptr<DistantLight>(new DistantLight(ctm, radiance, -dir));
        Light distant_light(Light::Type::eDistant);
        distant_light.illuminance = radiance;
        distant_light.transform   = ctm;
        distant_light.direction   = -dir;
        return distant_light;
    } else if (implementation == "infinite") {
        /**
         * spectrum	L rgb (1 1 1)	A radiance scale factor for the light; final emitted radiance
         *                          values for a particular direction are computed as the product of
         *                          this value and the radiance value found from the environment
         * map. integer	samples	1	Suggested number of shadow samples to take when computing
         * illumination from the light. Depending on the number of pixel samples being taken, this
         * value may need to be increased to reduce noise in the illumination computation for the
         * light. string	mapname	none	The environment map to use for the infinite area light.
         *                          If this is not provided, the light will be a constant color.
         */
        glm::vec3   radiance;
        int         num_samples = 1;
        std::string map_name;

        for (const auto& param : params) {
            if (contains(param.key, "L")) {
                radiance = ParseSpectrum(param.numbers, param.key);
            } else if (contains(param.key, "samples")) {
                const auto& nn = param.numbers;
                CHECK_GE(nn.size(), 1)
                    << "infinite light: key " << param.key << " doesn't have numbers";
                num_samples = static_cast<int>(nn.front());
            } else if (contains(param.key, "mapname")) {
                map_name = param.strings.front();
                map_name = strip_quotes(map_name);
                // 1.1. The file name is also relative to the pbrt file
                fs::path relative_path = fs::path(map_name).make_preferred();
                map_name               = (pbrt_file_root_path_ / relative_path).string();
            } else {
                LOG(ERROR) << "infinite area light, unrecognized param key: " << param.key;
            }
        }
        LOG(INFO) << "Reading environmental map " << map_name;

        Light inf_light(Light::Type::eInfinite);
        if (!map_name.empty()) {
            // auto inf_light = std::shared_ptr<InfiniteAreaLight>(
            //     new InfiniteAreaLight(ctm, radiance, num_samples, mapname));
            // return inf_light;
            auto        full_path = (pbrt_file_root_path_ / map_name).string();
            std::string ext       = full_path.substr(full_path.size() - 3);
            if (ext != "png") {
                LOG_ERROR << "unsupported for now";
            } else {

                int      image_width, image_height, image_channels;
                stbi_uc* pixels = stbi_load(full_path.c_str(), &image_width, &image_height,
                                            &image_channels, STBI_rgb_alpha);

                ImageData image_data;
                image_data.width        = image_width;
                image_data.height       = image_height;
                image_data.num_channels = image_channels;
                image_data.depth        = ImageData::Depth::eU8;
                u64 num_pixels          = image_width * image_height;
                image_data.data.resize(num_pixels * image_channels * sizeof(u8));
                memcpy(image_data.data.data(), pixels, num_pixels * image_channels * sizeof(u8));
                image_contents_.push_back(std::move(image_data));
            }

            inf_light.transform          = ctm;
            inf_light.envmap_image_index = image_contents_.size() - 1;
        } else {
            inf_light.illuminance = radiance;
        }
        return inf_light;
    } else if (implementation == "goniometric") {

    } else if (implementation == "projection") {

    } else if (implementation == "spot") {
        // spectrum	I	rgb (1 1 1)	Maximum radiant intensity of the light, i.e. the emitted radiant
        // intensity in the center of the illumination cone. It falls off to zero outside of the
        // cone. point	from, to	see the defaults are (0,0,0) and (0,0,1).
        //                      This gives a light that is pointing down the z axis.
        // float	coneangle	30	The angle that the spotlight's cone makes with its primary axis.
        // float	conedeltaangle	5 The angle where the intensity begins to fall off at the edges.
        glm::vec3 intensity{1.0f, 1.0f, 1.0f};
        glm::vec3 from           = glm::vec3(0, 0, 0), to(0.0f, 0.0f, 1.0f);
        float     cone_angle_deg = 30, cone_delta_angle_deg = 5;

        for (const auto& param : params) {
            if (contains(param.key, "I")) {
                intensity = ParseSpectrum(param.numbers, param.key);
            } else if (contains(param.key, "from")) {
                auto numbers = param.numbers;
                from         = glm::vec3(numbers[0], numbers[1], numbers[2]);
            } else if (contains(param.key, "to")) {
                auto numbers = param.numbers;
                to           = glm::vec3(numbers[0], numbers[1], numbers[2]);
            } else if (contains(param.key, "conedeltaangle")) {
                cone_delta_angle_deg = param.numbers.front();
            } else if (contains(param.key, "coneangle")) {
                cone_angle_deg = param.numbers.front();
            } else {
                LOG(ERROR) << "spot light, unrecognized parameter key: " << param.key;
            }
        }

        glm::vec3 dir = glm::normalize(to - from), dirx, diry;
        MakeCoordSystem(dir, &dirx, &diry);
        glm::mat4 rotation(1.0f);
        rotation[0]           = glm::vec4(dirx, 0.0);
        rotation[1]           = glm::vec4(diry, 0.0);
        rotation[2]           = glm::vec4(dir, 0.0);
        Transform local_trans = glm::translate(from) * rotation;

        Light spot_light(Light::Type::eSpot);
        spot_light.cone_angle_degree       = cone_angle_deg;
        spot_light.cone_delta_angle_degree = cone_delta_angle_deg;
        spot_light.illuminance             = intensity;
        return spot_light;
    } else {
        LOG(ERROR) << "unrecognized light implementation";
    }
    return Light(Light::Type::eError);
}

Light ParseAreaLight(const std::vector<param_node_t>& params, const std::string& implementation,
                     const Transform& ctm)
{
    LOG(ERROR) << "unimplemented";
    return Light{Light::Type::eArea};
}

size_t SceneLoader::ParseMaterial(const std::vector<param_node_t>& params, const std::string& impl)
{
    /*
    Name	Implementation Class
    disney	DisneyMaterial
    fourier	FourierMaterial
    glass	GlassMaterial
    hair	HairMaterial
    kdsubsurface	KdSubsurfaceMaterial
    matte	MatteMaterial
    metal	MetalMaterial
    mirror	MirrorMaterial
    mix	MixMaterial
    none	A special material that signifies that the surface it is associated with should be
    ignored for ray intersections. (This is useful for specifying regions of space associated with
                                    participating media.)
    plastic	PlasticMaterial
    substrate	SubstrateMaterial
    subsurface	SubsurfaceMaterial
    translucent	TranslucentMaterial
    uber	UberMaterial*/

    std::vector<param_node_t> non_bumpmap_parameters;

    // The common part of all material: bump map.
    size_t bump_map;
    for (const auto& param : params) {
        if (contains(param.key, "string type")) {
            ;  // pass; this parameter has been consumed by the caller.
        } else if (contains(param.key, "bumpmap")) {
            std::string tex_name = param.strings.front();
            bump_map = ConstantOrImageTexture(param.numbers, param.strings, glm::vec3(0.0f));
        } else {
            non_bumpmap_parameters.push_back(param);
        }
    }

    Material res(Material::Type::eGlass);
    if (impl == "mirror") {
        size_t Kr_tex;
        for (auto& parameter : params) {
            if (contains(parameter.key, "Kr")) {
                Kr_tex =
                    ConstantOrImageTexture(parameter.numbers, parameter.strings, glm::vec3(0.9));
            } else {
                LOG(ERROR) << "unrecognized parameter: " << parameter.key;
            }
        }
        if (Kr_tex == kNotFound) {
            Kr_tex = ConstantOrImageTexture({}, {}, glm::vec3(0.9));
        }
        CHECK_NE(Kr_tex, kNotFound) << "no parameters present for Kr";
        res.type            = Material::Type::eMirror;
        res.specular_tex_id = Kr_tex;
    } else if (impl == "matte") {
        size_t diffuse_tex = kNotFound;
        float  sigma       = 0.f;
        for (auto p : non_bumpmap_parameters) {
            if (contains(p.key, "Kd")) {
                diffuse_tex = ConstantOrImageTexture(p.numbers, p.strings, glm::vec3(0.5));
            } else if (contains(p.key, "sigma")) {
                CHECK_EQ(p.numbers.size(), 1);
                sigma = p.numbers[0];
            }
        }
        res.type            = Material::Type::eMatte;
        res.diffuse_tex_id  = diffuse_tex;
        res.roughness_sigma = sigma;
    } else if (impl == "plastic") {
        res.type = Material::Type::ePlastic;
        for (auto p : params) {
            if (contains(p.key, "Kd")) {
                res.diffuse_tex_id = ConstantOrImageTexture(p.numbers, p.strings, glm::vec3(0.25));
            } else if (contains(p.key, "Ks")) {
                res.specular_tex_id = ConstantOrImageTexture(p.numbers, p.strings, glm::vec3(0.25));
            } else if (contains(p.key, "uroughness")) {
                res.u_roughness = p.numbers.size() > 1 ? p.numbers.front() : 0.1f;
                if (!p.strings.empty())
                    LOG_ERROR << "unimplemented textured roughness";
            } else if (contains(p.key, "vroughness")) {
                res.v_roughness = p.numbers.size() > 1 ? p.numbers.front() : 0.1f;
                if (!p.strings.empty())
                    LOG_ERROR << "unimplemented textured roughness";
            } else if (p.key == "remaproughness") {
                LOG_ERROR << "unimplemented: remap roughness at parsing time";
            } else {
                LOG_ERROR << "unhandled parameter: " << p.key;
            }
        }
    } else if (impl == "metal") {
        LOG(ERROR) << "unimplemented : metal material";
        print_param(params);
    } else if (impl == "uber") {
        float opacity = 1.0f;

        res.type = Material::Type::eUber;
        for (const auto& p : non_bumpmap_parameters) {
            if (contains(p.key, "Kd")) {
                res.diffuse_tex_id = ConstantOrImageTexture(p.numbers, p.strings, glm::vec3(0.25));
            } else if (contains(p.key, "Ks")) {
                res.specular_tex_id = ConstantOrImageTexture(p.numbers, p.strings, glm::vec3(0.25));
            } else if (contains(p.key, "Kr")) {
                res.reflective_tex_id =
                    ConstantOrImageTexture(p.numbers, p.strings, glm::vec3(0.0));
            } else if (contains(p.key, "Kt")) {
                res.transmissive_tex_id =
                    ConstantOrImageTexture(p.numbers, p.strings, glm::vec3(0.0));
            } else if (contains(p.key, "uroughness")) {
                res.u_roughness = p.numbers.empty() ? 0.0f : p.numbers.front();
            } else if (contains(p.key, "vroughness")) {
                res.v_roughness = p.numbers.empty() ? 0.0f : p.numbers.front();
            } else if (contains(p.key, "roughness")) {
                res.roughness_sigma = p.numbers.empty() ? 0.1f : p.numbers.front();
            } else if (contains(p.key, "eta") || contains(p.key, "index")) {
                res.eta = p.numbers.empty() ? 1.5f : p.numbers.front();
            } else if (contains(p.key, "opacity")) {
                opacity = p.numbers.empty() ? 1.0f : p.numbers.front();
            } else if (p.key == "remaproughness") {
                LOG_ERROR << "unimplemented: remap roughness at parsing time";
            } else {
                LOG(ERROR) << "unhandled parameter: " << p.key;
            }
        }
    } else if (impl == "mix") {
        LOG(ERROR) << "unimplemented : " << impl;
    } else if (impl == "glass") {
        res.type = Material::Type::eGlass;
        for (const auto& p : non_bumpmap_parameters) {
            if (contains(p.key, "Kr")) {
                res.reflective_tex_id =
                    ConstantOrImageTexture(p.numbers, p.strings, glm::vec3(1.0f));
            } else if (contains(p.key, "Kt")) {
                res.transmissive_tex_id =
                    ConstantOrImageTexture(p.numbers, p.strings, glm::vec3(1.0f));
            } else if (contains(p.key, "eta")) {
                res.eta = p.numbers.empty() ? 1.2f : p.numbers.front();
            } else {
                LOG(ERROR) << "Unsupported parameters: " << p.key;
            }
        }
    } else if (impl == "translucent") {
        LOG(ERROR) << "unimplemented : " << impl;
    } else {
        LOG(ERROR) << "unimplemented : " << impl;
        print_param(params);
    }
    material_descriptors_.push_back(res);
    return material_descriptors_.size() - 1;
}

size_t SceneLoader::ParseShape(const std::vector<param_node_t>& parameters,
                               const std::string&               implementation)
{
    PlyMesh mesh_model;
    size_t  mesh_index = kNotFound;
    if (implementation == "sphere") {
        float radius = 1.0f;
        float zmin = -radius, zmax = radius, phimax_degree = 360.0f;
        for (auto p : parameters) {
            if (contains(p.key, "radius")) {
                CHECK_EQ(p.numbers.size(), 1);
                radius = p.numbers.front();
            }
        }
        LOG_ERROR << "unimplemented";
        // shape.reset(new Sphere(ctm, nullptr, false, radius, phimax_degree, zmin, zmax));
    } else if (implementation == "trianglemesh") {
        std::vector<int>       indices;
        std::vector<glm::vec3> positions;
        std::vector<glm::vec3> tangents;
        std::vector<glm::vec3> normals;
        std::vector<glm::vec2> texture_coords;
        for (const auto& p : parameters) {
            auto numbers = p.numbers;
            if (contains(p.key, "indices")) {
                for (float n : numbers)
                    indices.push_back(static_cast<int>(n));
            } else if (contains(p.key, "P")) {
                CHECK_EQ(numbers.size() % 3, 0) << "positions not a multiple of 3";
                for (size_t i = 0; i < numbers.size(); i += 3) {
                    positions.push_back(glm::vec3(numbers[i + 0], numbers[i + 1], numbers[i + 2]));
                }
            } else if (contains(p.key, "N")) {
                CHECK_EQ(numbers.size() % 3, 0) << "normals not a multiple of 3";
                for (size_t i = 0; i < numbers.size(); i += 3) {
                    normals.push_back(glm::vec3(numbers[i + 0], numbers[i + 1], numbers[i + 2]));
                }
            } else if (contains(p.key, "S")) {
                CHECK_EQ(numbers.size() % 3, 0) << "tangents not a multiple of 3";
                for (size_t i = 0; i < numbers.size(); i += 3) {
                    tangents.push_back(glm::vec3(numbers[i + 0], numbers[i + 1], numbers[i + 2]));
                }
            } else if (contains(p.key, "uv") || contains(p.key, "st")) {
                CHECK_EQ(numbers.size() % 2, 0) << "uv coords not a multiple of 2";
                for (size_t i = 0; i < numbers.size(); i += 2) {
                    texture_coords.push_back(glm::vec2(numbers[i + 0], numbers[i + 1]));
                }
            }
        }
        int max_index = *std::max_element(indices.begin(), indices.end());
        CHECK_LT(max_index, positions.size());
        if (!normals.empty()) {
            CHECK_EQ(normals.size(), positions.size());
        } else {
            // Fills in normal vectors.
            normals = ComputeNormals(positions, indices);
            CHECK_EQ(normals.size(), positions.size());
        }
        if (!tangents.empty())
            CHECK_EQ(tangents.size(), positions.size());
        if (!texture_coords.empty())
            CHECK_EQ(texture_coords.size(), positions.size());

        mesh_model.positions   = std::move(positions);
        mesh_model.normals     = std::move(normals);
        mesh_model.texture_uvs = std::move(texture_coords);
        mesh_model.indices     = std::move(indices);
        meshes_.push_back(std::move(mesh_model));
        mesh_index = meshes_.size() - 1;
    } else if (implementation == "plymesh" || implementation == "objmesh") {
        // 1. Obtain the ply file name from the scene AST.
        // 2. Check if the desired mesh has been cached in `this->meshes_`
        // 3. If not cached, load the mesh **without** ctm, and add to `meshes_`

        // 1. the file name is in the first pair of parameters
        std::string ply_file_path;
        Texture     alpha_texture;

        mesh_index = kNotFound;

        for (const auto& param : parameters) {
            if (contains(param.key, "string filename")) {
                ply_file_path = param.strings.front();
            } else if (contains(param.key, "shadowalpha")) {
                LOG(INFO) << "shape has shadow alpha texture " << param.strings.front();
            } else if (contains(param.key, "alpha")) {
                LOG(INFO) << "shape has alpha texture " << param.strings.front();
            }
        }
        CHECK(!ply_file_path.empty()) << "PLY file is not specified for the shape";
        ply_file_path = strip_quotes(ply_file_path);
        // 1.1. The file name is also relative to the pbrt file
        std::string mesh_file_name =
            (pbrt_file_root_path_ / fs::path(ply_file_path).make_preferred()).string();

        // 2. Check the availability by file name.
        if (named_meshes_descriptors_.find(mesh_file_name) != named_meshes_descriptors_.end()) {
            mesh_index = named_meshes_descriptors_[mesh_file_name];
            CHECK_NE(mesh_index, kNotFound);
        } else {
            // 3. load the mesh from the file
            PlyMesh tri_bvh = LoadTriangleMesh(mesh_file_name);
            //, ctm_stack_.back(), reverse_orientation_stack.back());

            // tri_bvh->set_alpha_mask(alpha_texture);
            meshes_.push_back(std::move(tri_bvh));

            // cache the loaded mesh
            mesh_index                                = meshes_.size() - 1;
            named_meshes_descriptors_[mesh_file_name] = mesh_index;
        }
        // TODO: figure out how to incorporate the transform.
    }
    return mesh_index;
}


size_t SceneLoader::ParseTexture(const std::vector<param_node_t>& parameters,
                                 const std::string&               implementation)
{
    std::vector<param_node_t> non_mapping_parameters;
    // Consumes the parameters for common parameters. Makes a 2D in case 2D texture is parsed.
    Mapping map2D;
    {
        glm::vec3   v1(1, 0, 0), v2(0, 1, 0);
        std::string mapping_type = "uv";
        float       uscale = 1.0f, vscale = 1.0f, udelta = 0.0f, vdelta = 0.0f;
        for (const auto& param : parameters) {
            if (contains(param.key, "mapping")) {
                mapping_type = param.strings.front();
            } else if (contains(param.key, "uscale")) {
                uscale = param.numbers.front();
            } else if (contains(param.key, "vscale")) {
                vscale = param.numbers.front();
            } else if (contains(param.key, "udelta")) {
                udelta = param.numbers.front();
            } else if (contains(param.key, "vdelta")) {
                vdelta = param.numbers.front();
            } else if (contains(param.key, "v1")) {
                auto numbers = param.numbers;
                CHECK_EQ(numbers.size(), 3);
                v1 = glm::vec3(numbers[0], numbers[1], numbers[2]);
            } else if (contains(param.key, "v2")) {
                auto numbers = param.numbers;
                CHECK_EQ(numbers.size(), 3);
                v2 = glm::vec3(numbers[0], numbers[1], numbers[2]);
            } else {
                non_mapping_parameters.push_back(param);
            }
        }
        mapping_type = strip_quotes(mapping_type);
        if (mapping_type == "uv") {
            map2D = Mapping::UV(uscale, vscale, udelta, vdelta);
        } else if (mapping_type == "spherical") {
            map2D.type = Mapping::Type::eSphere;
        } else if (mapping_type == "cylindrical") {
            LOG(FATAL) << "cylindrical mapping unimplemented";
            map2D.type = Mapping::Type::eCylinder;
        } else if (mapping_type == "planar") {
            LOG(FATAL) << "planar mapping unimplemented";
            map2D.type = Mapping::Type::ePlanar;
        } else {
            LOG(FATAL) << "unrecognized 2D mapping: " << mapping_type;
        }
    }

    Texture result_texture;
    if (implementation == "Constant") {
        for (const auto& p : parameters) {
            if (contains(p.key, "value")) {
                result_texture.type           = Texture::Type::eConstant;
                result_texture.constant_color = parse_color<glm::vec3>(p.numbers, p.key);
            } else {
                LOG(ERROR) << "unhandled parameter for constant texture: " << p.key;
            }
        }
    } else if (implementation == "scale") {
        // Identify the types from the key.
        // TODO: scale
        result_texture.type = Texture::Type::eCheckered;
        for (const auto& p : parameters) {
            if (contains(p.key, "tex1")) {
                result_texture.checker_indices[0] =
                    ConstantOrImageTexture(p.numbers, p.strings, glm::vec3(1.0f));
            } else if (contains(p.key, "tex2")) {
                result_texture.checker_indices[1] =
                    ConstantOrImageTexture(p.numbers, p.strings, glm::vec3(1.0f));
            }
        }
        CHECK_NE(kNotFound, result_texture.checker_indices[0]);
        CHECK_NE(kNotFound, result_texture.checker_indices[1]);
    } else if (implementation == "imagemap") {
        enum class ImageWrap
        {
            eRepeat,
            eClamp,
            eBlack
        };
        std::string filename;  // required, supports TGA, PFM and EXR.
        ImageWrap   wrap_mode      = ImageWrap::eRepeat;
        float       max_anisotropy = 8.f;
        bool        do_trilinear   = false;  // If not doing trilinear, use EWA filtering.
        float       scale          = 1.0f;
        bool        gamma = true;  // TRUE for 8-bit format (TGA, PNG) and FALSE for floating point
                                   // formats.
        bool gamma_mentioned      = false;
        bool gamma_from_file_type = true;

        for (auto p : parameters) {
            if (contains(p.key, "filename")) {
                filename        = strip_quotes(p.strings.front());
                std::string ext = filename.substr(filename.size() - 3);
                for (auto& c : ext)
                    c = std::tolower(c);
                if (ext == "png" || ext == "tga")
                    gamma_from_file_type = true;
                else if (ext == "exr" || ext == "pfm")
                    gamma_from_file_type = false;
            } else if (contains(p.key, "wrap")) {
                const auto wrap_mode_str = strip_quotes(p.strings.front());

                if (wrap_mode_str == "clamp")
                    wrap_mode = ImageWrap::eClamp;
                else if (wrap_mode_str == "black")
                    wrap_mode = ImageWrap::eBlack;
                else if (wrap_mode_str == "repeat")
                    ;
                else
                    LOG(FATAL) << "unrecognized wrapping mode: " << wrap_mode_str;
            } else if (contains(p.key, "maxanisotropy")) {
                max_anisotropy = p.numbers.front();
            } else if (contains(p.key, "trilinear")) {
                do_trilinear = p.strings.front() == std::string("true");
            } else if (contains(p.key, "scale")) {
                scale = p.numbers.front();
            } else if (contains(p.key, "gamma")) {
                gamma           = p.strings.front() == std::string("true");
                gamma_mentioned = true;
            }
        }
        if (!gamma_mentioned)
            gamma = gamma_from_file_type;

        auto        fullpath = (pbrt_file_root_path_ / filename).string();
        std::string ext      = fullpath.substr(fullpath.size() - 3);
        if (ext != "png") {
            LOG_ERROR << "unsupported for now";
        } else {

            int      image_width, image_height, image_channels;
            stbi_uc* pixels = stbi_load(fullpath.c_str(), &image_width, &image_height,
                                        &image_channels, STBI_rgb_alpha);

            ImageData image_data;
            image_data.width        = image_width;
            image_data.height       = image_height;
            image_data.num_channels = image_channels;
            image_data.depth        = ImageData::Depth::eU8;
            u64 num_pixels          = image_width * image_height;
            image_data.data.resize(num_pixels * image_channels * sizeof(u8));
            memcpy(image_data.data.data(), pixels, num_pixels * image_channels * sizeof(u8));
            image_contents_.push_back(std::move(image_data));
        }
        result_texture.type    = Texture::Type::eSampledImage;
        result_texture.sampler = image_contents_.size() - 1;

    } else if (implementation == "mix") {
        LOG(ERROR) << "unimplemented: mix. parameters: ";
        print_param(parameters);
    } else if (implementation == "bilerp") {
        result_texture.type = Texture::Type::eBilinear;
        glm::vec3 v00(0.f), v01(1.f), v10(0.f), v11(1.f);
        for (const auto& params : non_mapping_parameters) {
            if (contains(params.key, "v00")) {
                v00 = parse_color<glm::vec3>(params.numbers, params.key);
            } else if (contains(params.key, "v01")) {
                v01 = parse_color<glm::vec3>(params.numbers, params.key);
            } else if (contains(params.key, "v10")) {
                v10 = parse_color<glm::vec3>(params.numbers, params.key);
            } else if (contains(params.key, "v11")) {
                v11 = parse_color<glm::vec3>(params.numbers, params.key);
            } else {
                LOG(ERROR) << "unhandled params: " << params.key;
            }
        }
        result_texture = Texture::Bilinear(v00, v01, v10, v11);
    } else if (implementation == "checkerboard") {
        result_texture.type = Texture::Type::eCheckered;
        for (const auto& p : parameters) {
            if (contains(p.key, "tex1")) {
                result_texture.checker_indices[0] =
                    ConstantOrImageTexture(p.numbers, p.strings, glm::vec3(1.0f));
            } else if (contains(p.key, "tex2")) {
                result_texture.checker_indices[1] =
                    ConstantOrImageTexture(p.numbers, p.strings, glm::vec3(1.0f));
            }
        }
    } else {
        LOG(ERROR) << "unimplemented material of implementation " << implementation;
        print_param(parameters);
        result_texture = Texture::Constant({0.5f, 0.5f, 0.5f});
    }

    if (result_texture.type == Texture::Type::eNone) {
        return kNotFound;
    } else {
        texture_descriptors_.push_back(result_texture);
        return static_cast<size_t>(texture_descriptors_.size() - 1);
    }
}

bool SceneLoader::traverse_tree()
{
    // Cleans up what we had.

    // Initializes the status stack with a set of default values.
    ctm_stack_.clear();
    ctm_stack_.emplace_back(Transform(1.0f));
    reverse_orientation_stack.clear();
    reverse_orientation_stack.push_back(false);

    bool res = true;

    for (int i = 0; i < scene_root_->world_items.size(); ++i) {

        const auto& world_item = scene_root_->world_items[i];
        res                    = TraverseWorldItem(world_item) && res;
        LOG(INFO) << "i = " << i << ", #meshes = " << meshes_.size()
                  << ", #mesh instances = " << mesh_instances_.size();
    }
    return res;
}

bool SceneLoader::TraverseWorldItem(const worlditem_node_t& wi)
{
    using wi_t = worlditem_node_t::directive_t;
    if (wi.directive == wi_t::Transform) {
        // Parses the transform from parameters, and applies that onto the current transform.
        // TODO: refer to Section B.2.2 on page 1111; transform applies on what side?
        this->ctm_stack_.back() = this->ctm_stack_.back() * ParseTransform(wi.trans.get());
    } else if (wi.directive == wi_t::Shape) {

        std::vector<param_node_t> non_alpha_params;
        size_t                    alpha_texture;
        for (const auto& param : wi.params) {
            if (contains(param.key, "shadowalpha")) {
                LOG(ERROR) << "shape has (unimplemented) shadow alpha texture "
                           << param.strings.front();
            } else if (contains(param.key, "alpha")) {
                LOG(INFO) << "shape has alpha texture " << param.strings.front();
                alpha_texture =
                    ConstantOrImageTexture(param.numbers, param.strings, glm::vec3(1.0f));
            } else {
                non_alpha_params.push_back(param);
            }
        }

        size_t old_num_meshes = meshes_.size();
        size_t mesh_index     = ParseShape(non_alpha_params, strip_quotes(wi.implementation));
        if (mesh_index == kNotFound) {
            LOG(ERROR) << "Error parsing shape";
            return false;
        } else {
            CHECK_GT(meshes_.size(), old_num_meshes);
        }
        MeshInstance mesh_instance;
        mesh_instance.mesh_index     = mesh_index;
        mesh_instance.material_index = current_material_;
        mesh_instance.obj_to_world   = ctm_stack_.back();
        mesh_instances_.push_back(mesh_instance);
    } else if (wi.directive == wi_t::AttrPair) {
        // Pushes transformation stack and reverse-orientation stack.
        Transform last_trans = ctm_stack_.back();
        ctm_stack_.push_back(last_trans);
        bool last_ro = reverse_orientation_stack.back();
        reverse_orientation_stack.push_back(last_ro);

        const auto& children = wi.child;

        for (const auto& child : children) {
            TraverseWorldItem(child);
        }

        ctm_stack_.pop_back();
        reverse_orientation_stack.pop_back();
    } else if (wi.directive == wi_t::TransPair) {
        // Pushes ctm only.
        Transform last_trans = ctm_stack_.back();
        ctm_stack_.push_back(last_trans);

        const auto& children = wi.child;

        for (const auto& child : children) {
            TraverseWorldItem(child);
        }

        ctm_stack_.pop_back();
    } else if (wi.directive == wi_t::ObjPair) {
        // Records how to make an object.
        // TODO: make a map from objpair name to shape
        // 1. push ctm
        Transform last_trans = ctm_stack_.back();
        ctm_stack_.push_back(last_trans);
        LOG(ERROR) << "unimplemented: ObjPair";

        bool create_success = CreateObject(wi.child, wi.object_name);
        if (create_success) {
            named_objects_[wi.object_name] = object_descriptors_.size() - 1;
        }
        ctm_stack_.pop_back();
    } else if (wi.directive == wi_t::ObjInst) {
        // TODO(zixun): use a different mechanism to store object instances.
        LOG(INFO) << "instantiating object named " << wi.inst_name;

        // For all mesh instances in the object formula, applies the transform and adds the instance
        // to mesh_instances_.
        auto it = named_objects_.find(wi.inst_name);
        if (it == named_objects_.end()) {
            LOG(ERROR) << "can't fine object named \'" << wi.inst_name << "\'";
        } else {
            CHECK_NE(it->second, kNotFound);
            const Object& formula = object_descriptors_[it->second];
            for (auto mi : formula.mesh_instances) {
                mi.obj_to_world = ctm_stack_.back() * mi.obj_to_world;
                mesh_instances_.push_back(mi);
            }
        }
    } else if (wi.directive == wi_t::Light) {
        // Parses a light and add to lights_.
        auto impl = strip_quotes(wi.implementation);
        lights_.emplace_back(ParseLight(wi.params, impl));
    } else if (wi.directive == wi_t::AreaLight) {
        // Parses an arealight.
        auto impl = strip_quotes(wi.implementation);
        lights_.emplace_back(ParseAreaLight(wi.params, impl, ctm_stack_.back()));
    } else if (wi.directive == wi_t::Texture) {
        // Parses a texture and adds it to the pool.
        const auto& tex_name       = wi.tex_name;
        const auto& tex_type       = wi.tex_type;  // "float" or "spectrum"
        const auto& implementation = wi.implementation;
        LOG(INFO) << "Texture name: " << tex_name << ", type = " << tex_type
                  << ", implementation = " << implementation;
        if (tex_type == "float" || tex_type == "color" || tex_type == "spectrum") {
            auto new_tex_index = ParseTexture(wi.params, implementation);
            if (new_tex_index != kNotFound)
                named_textures_[tex_name] = new_tex_index;
        } else {
            LOG(FATAL) << "unrecognized color type of texture: " << tex_type;
        }
    } else if (wi.directive == wi_t::Material) {
        // Parses a material and sets it to the current material.
        /**
         * Materials specify the light scattering properties of surfaces in the scene.
         * The Material directive specifies the current material, which then applies for all
         * subsequent shape definitions (until the end of the current attribute scope or
                                         until a new material is defined).
         */
        auto material = ParseMaterial(wi.params, wi.implementation);
        // This discards the last parsed material, and if no shapes take the material, it will be
        // destroyed.
        current_material_ = material;
    } else if (wi.directive == wi_t::MakeMaterial) {
        // Parses a material and name it for future use.
        std::string mtl_name = wi.maked_name;

        // Looks for the parameter "string type" and get is value as the implementation.
        std::string implementation;
        for (const auto& p : wi.params) {
            if (p.key == "string type")
                implementation = p.strings.front();
        }
        if (implementation.empty()) {
            LOG(FATAL) << "cannot find the implementation of material named " << mtl_name;
        }
        auto material              = ParseMaterial(wi.params, implementation);
        named_materials_[mtl_name] = material;
    }
    return true;
}

bool SceneLoader::CreateObject(const std::vector<worlditem_node_t>& children,
                               const std::string&                   obj_name)
{
    SceneLoader object_loader;
    object_loader.ctm_stack_.push_back(ctm_stack_.back());
    object_loader.pbrt_file_root_path_ = pbrt_file_root_path_;
    object_loader.reverse_orientation_stack.push_back(false);

    LOG(INFO) << "Creating object_loader named " << obj_name;
    for (const auto& child : children) {
        LOG(INFO) << "Visiting children in object_loader";
        object_loader.TraverseWorldItem(child);
    }

    LOG(INFO) << "Named materials in the object_loader:";
    for (const auto& [name, material] : object_loader.named_materials_) {
        LOG(INFO) << "\t" << name << ' ' << typeid(material).name();
    }
    LOG(INFO) << "Named meshes in the object_loader:";
    for (const auto& [name, mesh] : object_loader.named_meshes_descriptors_) {
        LOG(INFO) << "\t" << name;
    }

    const size_t kNumExistingImages    = image_contents_.size();
    const size_t kNumExistingTextures  = texture_descriptors_.size();
    const size_t kNumExistingMaterials = material_descriptors_.size();
    const size_t kNumExistingMeshes    = meshes_.size();

    // Transfers the loaded images to the main scene loader.
    // ---------------------------------------------------------------------------------------------
    for (auto& image_data : image_contents_) {
        image_contents_.push_back(std::move(image_data));
    }

    // Transfers the loaded textures to the main scene loader.
    // ---------------------------------------------------------------------------------------------
    for (auto tex : object_loader.texture_descriptors_) {
        if (tex.sampler != kNotFound)
            tex.sampler += kNumExistingImages;
        if (tex.checker_indices[0] != kNotFound)
            tex.checker_indices[0] += kNumExistingTextures;
        if (tex.checker_indices[1] != kNotFound)
            tex.checker_indices[1] += kNumExistingTextures;
        texture_descriptors_.push_back(tex);
    }
    for (auto& [name, tex_index] : object_loader.named_textures_) {
        if (named_textures_.find(name) != named_textures_.end()) {
            LOG(ERROR) << "duplicated names in float textures";
        } else {
            named_textures_[name] = tex_index + kNumExistingTextures;
        }
    }
    {
        std::set<size_t> indices;
        for (const auto &[name, tex_index]: named_textures_) {
            if (indices.find(tex_index) != indices.end()) {
                LOG(FATAL) << "duplicate indices: " << name << ',' << tex_index;
            }
            indices.insert(tex_index);
        }
    }

    // Transfers the loaded materials to the main scene loader.
    // ---------------------------------------------------------------------------------------------
    for (auto mtl : object_loader.material_descriptors_) {
        if (mtl.diffuse_tex_id != kNotFound)
            mtl.diffuse_tex_id += kNumExistingTextures;
        if (mtl.specular_tex_id != kNotFound)
            mtl.specular_tex_id += kNumExistingTextures;
        if (mtl.reflective_tex_id != kNotFound)
            mtl.reflective_tex_id += kNumExistingTextures;
        if (mtl.transmissive_tex_id != kNotFound)
            mtl.reflective_tex_id += kNumExistingTextures;
        material_descriptors_.push_back(mtl);
    }
    for (auto& [name, mtl_index] : object_loader.named_materials_) {
        if (named_materials_.find(name) != named_materials_.end()) {
            LOG(ERROR) << "duplicated names in materials";
        } else {
            named_materials_[name] = mtl_index + kNumExistingMaterials;
        }
    }

    // Transfers the loaded meshes to the main scene loader.
    // ---------------------------------------------------------------------------------------------
    for (auto& mesh : object_loader.meshes_) {
        if (mesh.material_index != kNotFound)
            mesh.material_index += kNumExistingMaterials;
        meshes_.push_back(std::move(mesh));
    }
    // Constructs the object.
    Object object_formula;
    object_formula.mesh_instances = std::move(object_loader.mesh_instances_);
    for (auto& mi : object_formula.mesh_instances) {
        if (mi.mesh_index != kNotFound)
            mi.mesh_index += kNumExistingMeshes;
        if (mi.material_index != kNotFound)
            mi.material_index += kNumExistingMaterials;
    }
    // Saves the object formula.
    object_descriptors_.push_back(std::move(object_formula));
    LOG_IF(ERROR, !object_loader.object_descriptors_.empty()) << " nested object not supported";

    return true;
}

// +------------------------------------------------------------------+
// | Scene methods
// +------------------------------------------------------------------+

bool Scene::LoadFromFile(const char* file_name)
{
    std::ifstream file_in(file_name);
    if (!file_in) {
        LOG(ERROR) << "Cannot open file " << file_name;
    }

    pbr::Tokenizer tokenizer(&file_in);
    scene_t        scene_root;
    pbr::Parser    parser(tokenizer, &scene_root);
    int            parse_error_code = parser.parse();

    if (parse_error_code == 0) {
        bool success = LoadFromAst(&scene_root, tokenizer.file_root_path());
        return success;
    } else {
        LOG(ERROR) << "Cannot parse the input file.";
        return false;
    }
}

bool Scene::LoadFromAst(const scene_t* root, const std::filesystem::path& root_path)
{
    SceneLoader loader;
    loader.pbrt_file_root_path_ = root_path;
    loader.SetRootScene(root);
    loader.traverse_tree();

    this->image_contents_ = std::move(loader.image_contents_);

    this->lights_ = std::move(loader.lights_);

    this->object_descriptors_       = std::move(loader.object_descriptors_);
    this->named_objects_            = std::move(loader.named_objects_);
    this->texture_descriptors_      = std::move(loader.texture_descriptors_);
    this->named_textures_           = std::move(loader.named_textures_);
    this->meshes_                   = std::move(loader.meshes_);
    this->named_meshes_descriptors_ = std::move(loader.named_meshes_descriptors_);
    this->material_descriptors_     = std::move(loader.material_descriptors_);
    this->named_materials_          = std::move(loader.named_materials_);
    this->mesh_instances_           = std::move(loader.mesh_instances_);

    return true;
}

}  // namespace vkpbr
