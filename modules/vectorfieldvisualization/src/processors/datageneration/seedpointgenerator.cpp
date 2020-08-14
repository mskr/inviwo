/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2015-2020 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#include <modules/vectorfieldvisualization/processors/datageneration/seedpointgenerator.h>
#include <modules/base/algorithm/randomutils.h>
#include <inviwo/core/interaction/events/mouseevent.h>

#include <glm/gtx/vector_angle.hpp>
#include <math.h>

#define RND 1
#define PLANE 2
#define LINE 3
#define SPHERE 4
#define INTERACTIVE 5
#define SURFACEMESH 6

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo SeedPointGenerator::processorInfo_{
    "org.inviwo.SeedPointGenerator3D",  // Class identifier
    "Seed Point Generator 3D",          // Display name
    "Data Creation",                    // Category
    CodeState::Stable,                  // Code state
    "CPU, Seed Points, Generator",      // Tags
};
const ProcessorInfo SeedPointGenerator::getProcessorInfo() const { return processorInfo_; }

SeedPointGenerator::SeedPointGenerator()
    : Processor()
    , seedSurface_("seedSurface")
    , seedPoints_("seedPoints")
    , lineGroup_("line", "Line")
    , planeGroup_("plane", "Plane")
    , sphereGroup_("sphere", "Sphere")
    , numberOfPoints_("numberOfPoints", "Number of Points", 10, 1, 1000)
    , planeResolution_("planeResolution", "Resolution", vec2(4, 4), vec2(2, 2), vec2(100, 100))
    , planeOrigin_("planeOrigin_", "Origin", vec3(0.0f, 0.0f, 0.5f), vec3(-1, -1, -1),
                   vec3(1, 1, 1))
    , planeE1_("planeP1_", "Offset 1", vec3(1.0f, 0.0f, 0.5f), vec3(-1, -1, -1), vec3(1, 1, 1))
    , planeE2_("planeP2_", "Offset 2", vec3(0.0f, 1.0f, 0.5f), vec3(-1, -1, -1), vec3(1, 1, 1))
    , sphereCenter_("sphereCenter", "Center", vec3(0.5f, 0.5f, 0.5f), vec3(0, 0, 0), vec3(1, 1, 1))
    , sphereRadius_("sphereRadius", "Radius")
    , lineStart_("lineStart", "Start", vec3(0.5f, 0.0f, 0.5f), vec3(-1, -1, -1), vec3(1, 1, 1))
    , lineEnd_("lineEnd_", "End", vec3(0.5f, 1.0f, 0.5f), vec3(-1, -1, -1), vec3(1, 1, 1))
    , generator_("generator", "Generator")
    , randomness_("randomness", "Randomness")
    , useSameSeed_("useSameSeed", "Use same seed", true)
    , seed_("seed", "Seed", 1, 0, 1000)
    , rd_()
    , mt_(rd_())
    , interactive_("interactive", "Interactive")
    , hoverEvents_(
          "hoverEvents", "Hover Events", [this](Event* e) { processPickEvent(e); },
          MouseButton::None, MouseState::Move)
    , clickEvents_(
          "clickEvents", "Click Events", [this](Event* e) { processPickEvent(e); },
          MouseButton::Left, MouseState::Press)
    , seedMin_("seedMin", "Seed Min", vec3(0), vec3(-1000), vec3(1000))
    , seedMax_("seedMax", "Seed Max", vec3(1), vec3(-1000), vec3(1000))
    , pickedSeed_("pickedSeed", "Seed", vec3(0), seedMin_.get(), seedMax_.get()) {
    addPort(seedSurface_);
    seedSurface_.setOptional(true);
    addPort(seedPoints_);

    generator_.addOption("random", "Random", RND);
    generator_.addOption("line", "Line", LINE);
    generator_.addOption("plane", "Plane", PLANE);
    generator_.addOption("sphere", "Sphere", SPHERE);
    generator_.addOption("interactive", "Interactive", INTERACTIVE);
    generator_.addOption("surfacemesh", "Surface Mesh", SURFACEMESH);
    generator_.setCurrentStateAsDefault();
    generator_.onChange([this]() { onGeneratorChange(); });
    addProperty(generator_);

    lineGroup_.addProperty(lineStart_);
    lineGroup_.addProperty(lineEnd_);
    addProperty(lineGroup_);

    planeGroup_.addProperty(planeResolution_);
    planeGroup_.addProperty(planeOrigin_);
    planeGroup_.addProperty(planeE1_);
    planeGroup_.addProperty(planeE2_);
    addProperty(planeGroup_);

    sphereGroup_.addProperty(sphereCenter_);
    sphereGroup_.addProperty(sphereRadius_);
    sphereRadius_.set(vec2(0.45, 0.55));
    sphereRadius_.setCurrentStateAsDefault();
    addProperty(sphereGroup_);

    addProperty(numberOfPoints_);

    addProperty(randomness_);
    randomness_.addProperty(useSameSeed_);
    randomness_.addProperty(seed_);
    useSameSeed_.onChange([&]() { seed_.setVisible(useSameSeed_.get()); });

    addProperties(interactive_);
    interactive_.addProperties(hoverEvents_, clickEvents_, seedMin_, seedMax_, pickedSeed_);

    onGeneratorChange();
}

void SeedPointGenerator::processPickEvent(Event* e) {

    const auto mouseEvent = static_cast<MouseEvent*>(e);
    auto mousePos = vec2(mouseEvent->posNormalized());

    auto coords = seedMax_.get() - seedMin_.get();
    auto z = seedMin_.get().z + coords.z / 2.f;

    auto val = seedMin_.get() + vec3(mousePos, z) * coords;

    pickedSeed_.set(val);

    process();
}

void SeedPointGenerator::process() {
    if (useSameSeed_.get()) {
        mt_.seed(seed_.get());
    }

    switch (generator_.get()) {
        case RND:
            randomPoints();
            break;
        case PLANE:
            planePoints();
            break;
        case LINE:
            linePoints();
            break;
        case SPHERE:
            spherePoints();
            break;
        case INTERACTIVE:
            seedPoints_.setData(std::make_shared<std::vector<vec3>>(1, pickedSeed_.get()));
            break;
        case SURFACEMESH:
            seedOnInputSurface();
            break;
        default:
            LogWarn("No points generated since given type is not yet implemented");
            break;
    }
}

void SeedPointGenerator::onGeneratorChange() {
    bool rnd = generator_.get() == RND;
    bool plane = generator_.get() == PLANE;
    bool line = generator_.get() == LINE;
    bool sphere = generator_.get() == SPHERE;
    bool surfacemesh = generator_.get() == SURFACEMESH;

    numberOfPoints_.setVisible(rnd || line || sphere || surfacemesh);

    planeResolution_.setVisible(plane);
    planeOrigin_.setVisible(plane);
    planeE1_.setVisible(plane);
    planeE2_.setVisible(plane);

    sphereCenter_.setVisible(sphere);
    sphereRadius_.setVisible(sphere);

    lineStart_.setVisible(line);
    lineEnd_.setVisible(line);
}

void SeedPointGenerator::spherePoints() {
    auto T = [](auto&& r) { return util::randomNumber<float>(r, 0, glm::two_pi<float>()); };
    auto cos_phi = [](auto&& r) { return util::randomNumber<float>(r, -1, 1); };
    auto R = [](auto&& r) { return util::randomNumber<float>(r, 0, 1); };

    auto points = std::make_shared<std::vector<vec3>>();

    for (int i = 0; i < numberOfPoints_.get(); i++) {
        float theta = T(mt_);
        float phi = std::acos(cos_phi(mt_));

        vec2 range = sphereRadius_.get();
        float r = std::pow(R(mt_), 1.0f / 3.0f);
        r = range.x + r * (range.y - range.x);

        float ct = std::cos(theta);
        float st = std::sin(theta);
        float sp = std::sin(phi);
        float cp = std::cos(phi);

        vec3 g = vec3(ct * sp, st * sp, cp);
        vec3 p = g * r + sphereCenter_.get();

        points->push_back(p);
    }

    seedPoints_.setData(points);
}

void SeedPointGenerator::linePoints() {
    auto points = std::make_shared<std::vector<vec3>>();
    float dt = 1.0f / (numberOfPoints_.get() - 1);
    for (int i = 0; i < numberOfPoints_.get(); i++) {
        auto p = lineStart_.get() + (lineEnd_.get() - lineStart_.get()) * (i * dt);
        points->push_back(p);
    }
    seedPoints_.setData(points);
}

void SeedPointGenerator::planePoints() {
    auto points = std::make_shared<std::vector<vec3>>();
    float dx = 1.0f / (planeResolution_.get().x - 1);
    float dy = 1.0f / (planeResolution_.get().y - 1);

    vec3 ox = planeE1_.get();
    vec3 oy = planeE2_.get();

    for (int i = 0; i < planeResolution_.get().x; i++) {
        for (int j = 0; j < planeResolution_.get().y; j++) {
            vec3 p = planeOrigin_.get();
            p += ox * (i * dx);
            p += oy * (j * dy);
            points->push_back(p);
        }
    }
    seedPoints_.setData(points);
}

void SeedPointGenerator::randomPoints() {
    auto points = std::make_shared<std::vector<vec3>>();
    for (int i = 0; i < numberOfPoints_.get(); i++) {
        const float x = util::randomNumber<float>(mt_);
        const float y = util::randomNumber<float>(mt_);
        const float z = util::randomNumber<float>(mt_);
        points->emplace_back(x, y, z);
    }
    seedPoints_.setData(points);
}

struct BoundedPlane {
    dvec3 origin, up, right;
};

using Real = double;
using Real3 = glm::dvec3;

Real intersectPlaneBidirectional(Real3 pos, Real3 dir, Real3 middle, Real3 normal) {
    const Real a = dot(dir, normal);
    const Real b = dot(middle - pos, normal);
    if (a >= 0.0 && b <= 0.0 || a <= 0.0 && b >= 0.0) return std::numeric_limits<Real>::infinity();
    return b / a;
}

void SeedPointGenerator::seedOnInputSurface() {

    auto pointMesh = [](std::shared_ptr<std::vector<vec3>> points, std::vector<vec4> color = {}) {
        auto mesh = std::make_shared<Mesh>(DrawType::Points, ConnectivityType::None);

        auto vertexRAM = std::make_shared<BufferRAMPrecision<vec3>>(points->size());
        auto colorRAM = std::make_shared<BufferRAMPrecision<vec4>>(points->size());
        auto radiiRAM = std::make_shared<BufferRAMPrecision<float>>(points->size());

        mesh->addBuffer(Mesh::BufferInfo(BufferType::PositionAttrib),
                        std::make_shared<Buffer<vec3>>(vertexRAM));
        mesh->addBuffer(Mesh::BufferInfo(BufferType::ColorAttrib),
                        std::make_shared<Buffer<vec4>>(colorRAM));
        mesh->addBuffer(Mesh::BufferInfo(BufferType::RadiiAttrib),
                        std::make_shared<Buffer<float>>(radiiRAM));

        auto& vertices = vertexRAM->getDataContainer();
        auto& colors = colorRAM->getDataContainer();
        auto& radii = radiiRAM->getDataContainer();

        auto& pointsRef = *points;
        for (size_t i = 0; i < pointsRef.size(); i++) {
            auto p = pointsRef[i];
            vertices.push_back(p);
            colors.push_back(color.size() > i ? color[i] : vec4(1));
            radii.push_back(.1f);
        }

        return mesh;
    };

    auto meshGet = [](Mesh& mesh, BufferType buf) {
        for (const auto b : mesh.getBuffers()) {
            if (b.first.type == buf) return b.second->getRepresentation<BufferRAM>();
        }
    };

    auto planeFromMesh = [&, meshGet, pointMesh](Mesh& mesh) {
        BoundedPlane result;
        auto verts = meshGet(mesh, BufferType::PositionAttrib);
        dvec3 lower(std::numeric_limits<double>::infinity());
        dvec3 upper(-std::numeric_limits<double>::infinity());
        dvec3 xstart(std::numeric_limits<double>::infinity());
        dvec3 xend(-std::numeric_limits<double>::infinity());
        for (size_t i = 0; i < verts->getSize(); i++) {
            auto v = verts->getAsDVec3(i);
            if (v.y < lower.y) lower = v;
            if (v.y > upper.y) upper = v;
            if (v.x < xstart.x) xstart = v;
            if (v.x > xend.x) xend = v;
        }
        if (glm::epsilonEqual(lower.y, upper.y, .0001)) {
            lower = dvec3(std::numeric_limits<double>::infinity());
            upper = dvec3(-std::numeric_limits<double>::infinity());
            for (size_t i = 0; i < verts->getSize(); i++) {
                auto v = verts->getAsDVec3(i);
                if (v.z < lower.z) lower = v;
                if (v.z > upper.z) upper = v;
            }
        }
        if (glm::epsilonEqual(xstart.x, xend.x, .0001)) {
            xstart = dvec3(std::numeric_limits<double>::infinity());
            xend = dvec3(-std::numeric_limits<double>::infinity());
            for (size_t i = 0; i < verts->getSize(); i++) {
                auto v = verts->getAsDVec3(i);
                if (v.z < xstart.z) xstart = v;
                if (v.z > xend.z) xend = v;
            }
        }
        result.up = upper - lower;
        auto n = glm::normalize(glm::cross(glm::normalize(result.up), glm::normalize(xend - xstart)));
        auto rightAxis = glm::normalize(glm::cross(glm::normalize(result.up), n));
        dvec3 leftest = rightAxis * 100000.;
        dvec3 rightest = rightAxis * (-100000.);
        for (size_t i = 0; i < verts->getSize(); i++) {
            auto v = verts->getAsDVec3(i);
            auto projV = glm::dot(v, rightAxis);
            if (projV < glm::dot(leftest, rightAxis)) leftest = v;
            if (projV > glm::dot(rightest, rightAxis)) rightest = v;
        }
        result.right = rightest - leftest;
        result.origin = lower;
        return result;

        /*auto pnts = std::make_shared<std::vector<vec3>>();
        pnts->push_back(lower);
        pnts->push_back(upper);
        pnts->push_back(leftest);
        pnts->push_back(rightest);
        pnts->push_back(origin);
        insideTest_.setData(
            pointMesh(pnts, {vec4(1, 0, 0, 1), vec4(1), vec4(0, 1, 1, 1), vec4(0, 0, 1, 1),
        vec4(1,1,0,1)}));*/
    };

    auto insideSeedSurface = [&](vec3 pos, Mesh& surface, BoundedPlane pl) {
        bool result = false;

        const auto buffers = surface.getBuffers();
        const BufferRAM* vertices = 0;
        const BufferRAM* normals = 0;
        for (const auto buf : buffers) {
            if (buf.first.type == BufferType::PositionAttrib) {
                vertices = buf.second->getRepresentation<BufferRAM>();
            }
            if (buf.first.type == BufferType::NormalAttrib) {
                normals = buf.second->getRepresentation<BufferRAM>();
            }
        }

        if (vertices != 0 && normals != 0) {

            const auto r0 = pl.origin - pl.up * 0.5;
            const auto r1 = dvec3(pos);

            const auto ray = r0;
            const auto dir = glm::normalize(r1 - r0);

            int isectCount = 0;

            for (size_t i = 0; i < vertices->getSize(); i += 2) {
                const auto v0 = (vertices->getAsDVec3(i));
                const auto v1 = (vertices->getAsDVec3(i + 1));

                const auto normal = normals->getAsDVec3(i);
                const auto middle = v0;

                auto t = intersectPlaneBidirectional(ray, dir, middle, normal);
                if (t < 0) continue;

                const auto isect = ray + t * dir;

                if (isect.x >= std::min(v0.x, v1.x) && isect.x <= std::max(v0.x, v1.x) &&
                    isect.y >= std::min(v0.y, v1.y) && isect.y <= std::max(v0.y, v1.y) &&
                    isect.x >= std::min(r0.x, r1.x) && isect.x <= std::max(r0.x, r1.x) &&
                    isect.y >= std::min(r0.y, r1.y) && isect.y <= std::max(r0.y, r1.y)) {
                    isectCount++;
                }
            }

            result = isectCount % 2 != 0;
        }
        return result;
    };

    if (!seedSurface_.hasData()) {
        LogWarn("No Seed Surface in Inport.");
        return;
    }

    auto surface = *seedSurface_.getData();
    BoundedPlane pl = planeFromMesh(surface);

    auto seeds = std::make_shared<std::vector<vec3>>();
    std::vector<vec4> colors;

    size_t gridSize = 1;
    while (gridSize * gridSize < numberOfPoints_.get()) gridSize++;

    for (size_t i = 0; i < numberOfPoints_.get(); i++) {
        float x = (float)(i % gridSize) / (float)gridSize;
        float y = (float)(i / gridSize) / (float)gridSize;
        const auto p = vec3(pl.origin) + x * vec3(pl.up) + ((y * 1.2f - 0.5f) * vec3(pl.right));
        if (insideSeedSurface(p, surface, pl)) {
            seeds->push_back(p);
            colors.push_back(vec4(0, 1, 0, 1));
        }
    }

    size_t tries = 0;

    std::default_random_engine generator;
    while (seeds->size() < numberOfPoints_.get()) {
        const float x = util::randomNumber<float>(generator);
        const float y = util::randomNumber<float>(generator);
        const auto p = vec3(pl.origin) + x * vec3(pl.up) + ((y * 1.2f - 0.5f) * vec3(pl.right));
        if (insideSeedSurface(p, surface, pl)) {
            seeds->push_back(p);
            colors.push_back(vec4(0, 1, 0, 1));
        }
        if (tries++ > numberOfPoints_.get() * 2) break;
    }

    seedPoints_.setData(seeds);
};

}  // namespace inviwo
