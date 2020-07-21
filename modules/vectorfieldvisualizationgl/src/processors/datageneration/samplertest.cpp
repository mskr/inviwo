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

#include <modules/vectorfieldvisualizationgl/processors/datageneration/samplertest.h>

#include <inviwo/core/datastructures/image/imageram.h>
#include <inviwo/core/datastructures/image/layerramprecision.h>

#include <modules/eigenutils/eigenutils.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Eigenvalues/EigenSolver.h>

namespace inviwo {

const ProcessorInfo SamplerTest::processorInfo_{
    "org.inviwo.FlowFieldFeatureSeeding",  // Class identifier
    "Flow Field Feature Seeding",          // Display name
    "Data Creation",                       // Category
    CodeState::Stable,                     // Code state
    Tags::GL,                              // Tags
};

const ProcessorInfo SamplerTest::getProcessorInfo() const { return processorInfo_; }

SamplerTest::SamplerTest()
    : Processor()
    , a_("a")
    , b_("b")
    , image_("image")
    , seeds_("seeds")
    , markers_("markers")
    , thresholds_("thresholds", "Seed at", 0.1f, 0.9f, 0.f, 1.f)
    , step_("step", "Step", 0.02f)
    , maxSize_("maxSize", "Pixel Limit", 200, 100, 2000)
    , realStep_("realStep", "Real Step", step_.get())
    , min_("min", "Min")
    , max_("max", "Max")
    , minVelMag_("minVelMag", "Min ||V||")
    , maxVelMag_("maxVelMag", "Max ||V||") {
    addPort(a_);
    addPort(b_);
    addPort(image_);
    addPort(seeds_);
    addPort(markers_);
    addProperties(thresholds_, step_, maxSize_, realStep_, min_, max_, minVelMag_, maxVelMag_);
    realStep_.setReadOnly(true);
    maxVelMag_.setMaxValue(100.f);
}

SamplerTest::~SamplerTest() = default;

void SamplerTest::initializeResources() {}

void SamplerTest::process() {
    const auto W = max_.get().x - min_.get().x;
    const auto H = max_.get().y - min_.get().y;
    if (W / step_.get() > maxSize_.get()) {
        realStep_.set(W / maxSize_.get());
    }
    if (H / step_.get() > maxSize_.get()) {
        realStep_.set(H / maxSize_.get());
    }

    using FlowField = std::shared_ptr<const SpatialSampler<2, 2, double>>;

    const auto a = a_.getData();
    const auto b = b_.getData();

    auto seeds = std::make_shared<std::vector<vec2>>();

    const auto size = size2_t(W / realStep_.get(), H / realStep_.get());

    const auto h = step_.get();

    auto layer = std::make_shared<Layer>(
        size, DataFloat32::get(), LayerType::Color,
        SwizzleMask{ImageChannel::Red, ImageChannel::Green, ImageChannel::Blue, ImageChannel::One});
    auto ram = layer->getEditableRepresentation<LayerRAM>();

    auto mag = [&](FlowField field, vec2 p) { return glm::length(field->sample(p)); };

    auto grad = [&](FlowField field, vec2 p) {
        const auto dx = (mag(field, vec2(p.x + h, p.y)) - mag(field, vec2(p.x - h, p.y))) / (2 * h);
        const auto dy = (mag(field, vec2(p.x, p.y + h)) - mag(field, vec2(p.x, p.y - h))) / (2 * h);
        return vec2(dx, dy);
    };

    auto intersectPlane = [](vec3 pos, vec3 dir, vec3 middle, vec3 normal) {
        const float a = dot(dir, normal);
        if (a >= 0.0) return -1.0f;  // facing away
        const float b = dot(middle - pos, normal);
        if (b >= 0.0) return -1.0f;  // behind or on plane
        return b / a;
    };

    auto findZero = [&](FlowField field, float& x, float& y, float& z) {
        bool outOfCell = false;
        while (z > 0.0001f) {
            auto g = grad(field, vec2(x, y));
            float dz = mag(field, vec2(x, y) + glm::normalize(g) * h) - z;
            if (dz > 0.0f) dz = -dz;
            auto dir = glm::normalize(vec3(g, dz));
            auto t = intersectPlane(vec3(x, y, z), -dir, vec3(x, y, 0), vec3(0, 0, 1));
            auto vp = vec3(x, y, z) + t * dir;
            outOfCell = glm::distance(vp, vec3(x, y, z)) > realStep_.get();
            if (outOfCell) break;
            x = vp.x;
            y = vp.y;
            z = vp.z;
        }
        return outOfCell;
    };

    auto jac = [&](FlowField field, vec2 p) {
        mat2 j(0);
        for (int v = 0; v < 2; v++) {
            for (int x = 0; x < 2; x++) {
                vec2 p0 = p, p1 = p;
                p0[x] -= h;
                p1[x] += h;
                j[v][x] = (field->sample(p1)[v] - field->sample(p0)[v]) / (2 * h);
            }
        }
        return j;
    };

    auto eig = [](const mat2 M) {
        Eigen::EigenSolver<Eigen::Matrix<float, 2, 2>> solver(util::glm2eigen(M));
        auto lambda1 = solver.eigenvalues().col(0)[0].real();
        auto lambda2 = solver.eigenvalues().col(0)[1].real();
        return vec2(lambda1, lambda2);
    };

    auto minVelMag = std::numeric_limits<double>::infinity();
    auto maxVelMag = 0.0;

    std::vector<vec2> markers;

    for (size_t i = 0; i < size.x; i++) {
        for (size_t j = 0; j < size.y; j++) {
            float x = min_.get().x + (max_.get().x - min_.get().x) * ((float)i / size.x);
            float y = min_.get().y + (max_.get().y - min_.get().y) * ((float)j / size.y);
            float z = mag(a, vec2(x, y));
            vec2 g = grad(a, vec2(x, y));
            vec2 next = vec2(x, y) - g;
            vec2 nextG = grad(a, next);
            float nextZ = mag(a, next);
            if (a->withinBounds(vec2(x, y))) {
                // if (z < nextZ && glm::length(g) < glm::length(nextG))
                const auto J = jac(a, vec2(x, y));
                const auto S = (J + glm::transpose(J)) / 2;
                const auto O = (J - glm::transpose(J)) / 2;
                const auto e = eig(S * S + O * O);
                if (e[0] < 0.f && e[1] < 0.f) markers.push_back(vec2(x, y));
                if (z < minVelMag) minVelMag = z;
                if (z > maxVelMag) maxVelMag = z;
            }
            ram->setFromDouble({i, j}, z);
        }
    }

    minVelMag_.set(minVelMag);
    maxVelMag_.set(maxVelMag);

    image_.setData(std::make_shared<Image>(layer));
    seeds_.setData(seeds);

    auto mesh = std::make_shared<Mesh>(DrawType::Points, ConnectivityType::None);

    auto vertexRAM = std::make_shared<BufferRAMPrecision<vec3>>(markers.size());
    auto colorRAM = std::make_shared<BufferRAMPrecision<vec4>>(markers.size());
    auto radiiRAM = std::make_shared<BufferRAMPrecision<float>>(markers.size());

    mesh->addBuffer(Mesh::BufferInfo(BufferType::PositionAttrib),
                    std::make_shared<Buffer<vec3>>(vertexRAM));
    mesh->addBuffer(Mesh::BufferInfo(BufferType::ColorAttrib),
                    std::make_shared<Buffer<vec4>>(colorRAM));
    mesh->addBuffer(Mesh::BufferInfo(BufferType::RadiiAttrib),
                    std::make_shared<Buffer<float>>(radiiRAM));

    auto& vertices = vertexRAM->getDataContainer();
    auto& colors = colorRAM->getDataContainer();
    auto& radii = radiiRAM->getDataContainer();

    for (auto marker : markers) {
        vertices.push_back(vec3(marker, 0));
        colors.push_back(vec4(0, 1, 0, 1));
        radii.push_back(0.1);
    }

    markers_.setData(mesh);
}

}  // namespace inviwo