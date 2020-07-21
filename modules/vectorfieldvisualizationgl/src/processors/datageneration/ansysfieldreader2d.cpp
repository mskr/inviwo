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

#include <modules/vectorfieldvisualizationgl/processors/datageneration/ansysfieldreader2d.h>
#include <modules/opengl/texture/textureunit.h>
#include <modules/opengl/texture/textureutils.h>
#include <modules/opengl/shader/shaderutils.h>
#include <modules/opengl/volume/volumegl.h>
#include <modules/opengl/texture/textureutils.h>
#include <inviwo/core/util/colorbrewer.h>
#include <random>
#include <fstream>

namespace reading {

std::string line(std::istream& in) {
    std::string result;
    std::getline(in, result);
    return result;
}

std::vector<std::string> strings(std::string input = "split on    whitespace   ",
                                 char separator = ' ') {
    std::vector<std::string> result;
    std::string next = "";
    for (size_t i = 0; i < input.length(); i++) {
        if (input[i] != separator) {
            next += input[i];
        } else if (!next.empty()) {
            result.push_back(next);
            next = "";
        }
    }
    if (!next.empty()) result.push_back(next);
    return result;
}

using R = float;
using Index = size_t;

enum QuantityType { SCALAR, VEC2, VEC3, MAX_COMPONENTS };

#include <map>
#include <string>

struct SelectedQuantity {
    bool found;
    std::string name;
    QuantityType type;
    Index valueRefs[MAX_COMPONENTS];
    SelectedQuantity(std::map<std::string, Index> quantities, std::string quantity);
    size_t numComponents() const { return type == SCALAR ? 1 : type == VEC2 ? 2 : 3; }
};

SelectedQuantity::SelectedQuantity(std::map<std::string, Index> quantities, std::string quantity) {
    this->name = quantity;
    this->found = true;

    if (quantity[0] != '[') {
        this->type = SCALAR;
        try {
            this->valueRefs[0] = quantities.at(quantity);
        } catch (const std::out_of_range) {
            // throw std::runtime_error("Selected quantity not found.");
            this->found = false;
        }
    } else {
        const auto components = strings(quantity.substr(1, quantity.length() - 2), ',');

        if (components.size() == 2) {
            this->type = VEC2;
        } else if (components.size() == 3) {
            this->type = VEC3;
        } else {
            // throw std::runtime_error("Too many components.");
            this->found = false;
        }

        for (Index i = 0; i < components.size(); i++) {
            try {
                this->valueRefs[i] = quantities.at(components[i]);
            } catch (const std::out_of_range) {
                // throw std::runtime_error("One of the components of the selected quantity was not
                // found.");
                this->found = false;
            }
        }
    }
}

using namespace SimData2D;

void read(const std::string file, PointCloud& out) {

    std::ifstream in(file);

    const auto headers = strings(line(in));

    std::map<std::string, Index> quantities;

    for (Index i = 0; i < headers.size(); i++) {
        const auto name = headers[i];
        quantities[name] = i;
        printf("%s\n", name.c_str());
    }

    bool sim2D = false;

    try {
        const auto i = quantities.at("z-coordinate");
    } catch (const std::out_of_range) {
        sim2D = true;
    }

    out.subgroups.clear();
    out.points.clear();
    out.vel.clear();
    out.dist.clear();
    out.WSS.clear();
    out.press.clear();

    SelectedQuantity vel(quantities, "[x-velocity,y-velocity]");
    SelectedQuantity dist(quantities, "cell-wall-distance");
    SelectedQuantity wss(quantities, "wall-shear");
    SelectedQuantity press(quantities, "total-pressure");

    int groupID = 0;
    R lastCellnumber = 0;
    size_t count = 0;
    size_t countSinceGroupStart = 0;

    for (std::string data; std::getline(in, data);) {

        std::vector<R> numbers;

        for (auto ascii : strings(data)) {
            std::string::size_type sz;
            numbers.push_back(static_cast<R>(std::stod(ascii, &sz)));
            while (sz < ascii.length()) {
                ascii = ascii.substr(sz);
                numbers.push_back(static_cast<R>(std::stod(ascii, &sz)));
            }
        }

        R cellnumber = numbers[0];
        if (cellnumber < lastCellnumber) {
            out.subgroups.push_back({countSinceGroupStart, count - 1, groupID});
            groupID++;
            countSinceGroupStart = count;
        }
        lastCellnumber = cellnumber;

        out.points.push_back({numbers[1], numbers[2]});

        if (vel.found) out.vel.push_back({numbers[vel.valueRefs[0]], numbers[vel.valueRefs[1]]});

        if (dist.found) out.dist.push_back(numbers[dist.valueRefs[0]]);

        if (wss.found) out.WSS.push_back(numbers[wss.valueRefs[0]]);

        if (press.found) out.press.push_back(numbers[press.valueRefs[0]]);

        count++;
    }

    if (countSinceGroupStart > 0)
        out.subgroups.push_back({countSinceGroupStart, count - 1, groupID});
}
}  // namespace reading

namespace inviwo {

const ProcessorInfo AnsysFieldReader2D::processorInfo_{
    "org.inviwo.AnsysFieldReader2D",  // Class identifier
    "ANSYS Field ASCII Reader 2D",    // Display name
    "Data Creation",                  // Category
    CodeState::Stable,                // Code state
    Tags::GL,                         // Tags
};
const ProcessorInfo AnsysFieldReader2D::getProcessorInfo() const { return processorInfo_; }

AnsysFieldReader2D::AnsysFieldReader2D()
    : Processor()
    , boundaries_("boundaries")
    , pointCloud_("pointCloud")
    , sampler_("sampler")
    , velocityField_("velocityField", DataVec2Float32::get(), false)
    , wallDistanceField_("wallDistanceField", DataFloat32::get())
    , wssField_("wssField", DataFloat32::get())
    , pressField_("pressField", DataFloat32::get())
    , insideTest_("insideTest")
    , streamlineSeeds_("streamlineSeeds")
    , file_("filename", "File")
    , readButton_("readButton", "Read")
    , subgroupSelector_("subgroupSelector", "Subgroup")
    , pointcloudVis_("pointCloudVis", "Raw View")
    , pointSize_("pnts", "Point Size")
    , velocityScaling_("velocityScaling", "Velocity")
    , spaceStats_("spaceStats", "Space Stats")
    , pointCloudSize_("pointCloudSize", "Points", 0, 0, std::numeric_limits<size_t>::max())
    , pointCloudMinNearestDistance_("pointCloudMinDistance", "Min Grid Step")
    , pointCloudMaxNearestDistance_("pointCloudMaxDistance", "Max Grid Step")
    , pointCloudAvgNearestDistance_("pointCloudAvgDistance", "Avg Grid Step")
    , pointCloudMin_("pointCloudMin", "Point Cloud Min", vec2(0), vec2(-1000), vec2(1000))
    , pointCloudMax_("pointCloudMax", "Point Cloud Max", vec2(0), vec2(-1000), vec2(1000))
    , meshMin_("meshMin", "Mesh Min", vec2(0), vec2(-1000), vec2(1000))
    , meshMax_("meshMax", "Mesh Max", vec2(0), vec2(-1000), vec2(1000))
    , resampling_("resampling", "Resampling")
    , size_("size", "Resolution", size2_t(200), size2_t(1), size2_t(1024))
    , distanceTolerance_("distanceTolerance", "Dist Tolerance")
    , minDistance_("minDistance", "Min Dist")
    , maxDistance_("maxDistance", "Max Dist")
    , avgDistance_("avgDistance", "Avg Dist")
    , insideTestPoint_("insideTestPoint", "Inside Test Point", vec2(0), vec2(-1000), vec2(1000)) {

    addPort(boundaries_);

    addPort(pointCloud_);
    addPort(sampler_);
    addPort(velocityField_);
    addPort(wallDistanceField_);
    addPort(wssField_);
    addPort(pressField_);
    addPort(insideTest_);
    addPort(streamlineSeeds_);

    minDistance_.setReadOnly(true);
    maxDistance_.setReadOnly(true);
    avgDistance_.setReadOnly(true);
    distanceTolerance_.set(std::numeric_limits<float>::infinity());

    addProperties(file_, readButton_, subgroupSelector_, pointcloudVis_, spaceStats_, resampling_,
                  insideTestPoint_);

    pointcloudVis_.addProperties(pointSize_, velocityScaling_);

    spaceStats_.addProperties(pointCloudSize_, pointCloudMinNearestDistance_,
                              pointCloudMaxNearestDistance_, pointCloudAvgNearestDistance_,
                              pointCloudMin_, pointCloudMax_, meshMin_, meshMax_);
    for (auto prop : spaceStats_.getProperties()) prop->setReadOnly(true);

    resampling_.addProperties(size_, distanceTolerance_, minDistance_, maxDistance_, avgDistance_);

    readButton_.onChange([this]() {
        reading::read(file_.get(), pointcloud);

        pointCloudSize_.set(pointcloud.points.size());

        velocitySampler =
            std::make_shared<PointCloudVelocitySampler>(pointcloud, *boundaries_.getData());

        subgroupSelector_.clearOptions();
        subgroupSelector_.addOption("all", "All", -1);
        for (const auto g : pointcloud.subgroups) {
            subgroupSelector_.addOption(std::to_string(g.id), std::to_string(g.id), g.id);
        }

        float sum = 0;
        int count = 0;
        float minNearestDist = std::numeric_limits<float>::infinity();
        float maxNearestDist = 0;
        float avgNearestDist = 0;
        for (size_t i = 0; i < pointcloud.points.size(); i++) {
            float nearest = std::numeric_limits<float>::infinity();
            for (size_t j = 0; j < pointcloud.points.size(); j++) {
                float d = glm::distance(pointcloud.points[i], pointcloud.points[j]);
                if (i != j && d < nearest) nearest = d;
            }
            if (nearest < minNearestDist) minNearestDist = nearest;
            if (nearest > maxNearestDist) maxNearestDist = nearest;
            sum += nearest;
            count++;
        }
        if (count != 0) avgNearestDist = sum / count;

        pointCloudMinNearestDistance_.set(minNearestDist);
        pointCloudMaxNearestDistance_.set(maxNearestDist);
        pointCloudAvgNearestDistance_.set(avgNearestDist);

        pointCloudMin_.set(velocitySampler->pointCloudAABB.min);
        pointCloudMax_.set(velocitySampler->pointCloudAABB.max);
        meshMin_.set(velocitySampler->meshAABB.min);
        meshMax_.set(velocitySampler->meshAABB.max);

        pointSize_.set(avgNearestDist / 2.f);
        pointSize_.setMaxValue(maxNearestDist * 3.f);
        velocityScaling_.set(.8f);
        velocityScaling_.setMinValue(-2.f);
        velocityScaling_.setMaxValue(2.f);

        sampler_.setData(velocitySampler);

        if (resampling_.getBoolProperty()->get()) {
            if (pointcloud.vel.size() > 0)
                velocityField_.setData(pointCloudToImage2D<DataVec2Float32>(
                    pointcloud, [&](LayerRAM* ram, size_t i, size_t j, size_t nearestNeighbor) {
                        const vec2 value = pointcloud.vel[nearestNeighbor];
                        ram->setFromDVec2({i, j}, value);
                    }));
            if (pointcloud.dist.size() > 0)
                wallDistanceField_.setData(pointCloudToImage2D<DataFloat32>(
                    pointcloud, [&](LayerRAM* ram, size_t i, size_t j, size_t nearestNeighbor) {
                        const float value = pointcloud.dist[nearestNeighbor];
                        ram->setFromDouble({i, j}, value);
                    }));
            if (pointcloud.WSS.size() > 0)
                wssField_.setData(pointCloudToImage2D<DataFloat32>(
                    pointcloud, [&](LayerRAM* ram, size_t i, size_t j, size_t nearestNeighbor) {
                        const float value = pointcloud.WSS[nearestNeighbor];
                        ram->setFromDouble({i, j}, value);
                    }));
            if (pointcloud.press.size() > 0)
                pressField_.setData(pointCloudToImage2D<DataFloat32>(
                    pointcloud, [&](LayerRAM* ram, size_t i, size_t j, size_t nearestNeighbor) {
                        const float value = pointcloud.press[nearestNeighbor];
                        ram->setFromDouble({i, j}, value);
                    }));
        }
    });

    insideTestPoint_.onChange([this]() {
        if (velocitySampler) {
            auto mesh = std::make_shared<Mesh>(DrawType::Points, ConnectivityType::None);

            auto vertexRAM = std::make_shared<BufferRAMPrecision<vec3>>(1);
            auto colorRAM = std::make_shared<BufferRAMPrecision<vec4>>(1);
            auto radiiRAM = std::make_shared<BufferRAMPrecision<float>>(1);

            mesh->addBuffer(Mesh::BufferInfo(BufferType::PositionAttrib),
                            std::make_shared<Buffer<vec3>>(vertexRAM));
            mesh->addBuffer(Mesh::BufferInfo(BufferType::ColorAttrib),
                            std::make_shared<Buffer<vec4>>(colorRAM));
            mesh->addBuffer(Mesh::BufferInfo(BufferType::RadiiAttrib),
                            std::make_shared<Buffer<float>>(radiiRAM));

            auto& vertices = vertexRAM->getDataContainer();
            auto& colors = colorRAM->getDataContainer();
            auto& radii = radiiRAM->getDataContainer();

            vertices[0] = vec3(insideTestPoint_.get(), 0);

            if (velocitySampler->inside(insideTestPoint_.get()))
                colors[0] = vec4(0, 1, 0, 1);
            else
                colors[0] = vec4(1, 0, 0, 1);

            radii[0] = (double)pointSize_.get();

            insideTest_.setData(mesh);
        }
    });

    subgroupSelector_.onChange([&]() {
        const auto selected = subgroupSelector_.getSelectedValue();
        if (!pointcloud.subgroups.empty()) {
            const auto start = selected >= 0 ? pointcloud.subgroups[selected].start : 0;
            const auto end =
                selected >= 0 ? pointcloud.subgroups[selected].end : pointcloud.points.size();
            auto seeds = std::make_shared<std::vector<vec2>>(pointcloud.points.begin() + start,
                                                             pointcloud.points.begin() + end);
            if (end - start > 500) {
                auto reduced = std::make_shared<std::vector<vec2>>();
                std::default_random_engine generator;
                std::uniform_int_distribution<int> distribution(0, seeds->size());
                for (int i = 0; i < 500; i++)
                    reduced->push_back(seeds->at(distribution(generator)));
                seeds = reduced;
            }
            streamlineSeeds_.setData(seeds);
        }
    });
}

AnsysFieldReader2D::~AnsysFieldReader2D() = default;

void AnsysFieldReader2D::initializeResources() {}

void AnsysFieldReader2D::process() { pointCloud_.setData(pointCloudToMesh2D(pointcloud)); }

std::shared_ptr<Mesh> AnsysFieldReader2D::pointCloudToMesh2D(PointCloud& pointcloud) {

    const auto numSpheres = pointcloud.points.size();

    auto mesh = std::make_shared<Mesh>(DrawType::Points, ConnectivityType::None);

    auto vertexRAM = std::make_shared<BufferRAMPrecision<vec3>>(numSpheres);
    auto colorRAM = std::make_shared<BufferRAMPrecision<vec4>>(numSpheres);
    auto radiiRAM = std::make_shared<BufferRAMPrecision<float>>(numSpheres);

    mesh->addBuffer(Mesh::BufferInfo(BufferType::PositionAttrib),
                    std::make_shared<Buffer<vec3>>(vertexRAM));
    mesh->addBuffer(Mesh::BufferInfo(BufferType::ColorAttrib),
                    std::make_shared<Buffer<vec4>>(colorRAM));
    mesh->addBuffer(Mesh::BufferInfo(BufferType::RadiiAttrib),
                    std::make_shared<Buffer<float>>(radiiRAM));

    auto& vertices = vertexRAM->getDataContainer();
    auto& colors = colorRAM->getDataContainer();
    auto& radii = radiiRAM->getDataContainer();

    vec2 v_max(0);
    for (const auto v : pointcloud.vel) {
        if (v.x > v_max.x) v_max.x = v.x;
        if (v.y > v_max.y) v_max.y = v.y;
    }

    const auto colormap =
        colorbrewer::getColormap(colorbrewer::Family::Accent,
                                 pointcloud.subgroups.empty() ? 3 : pointcloud.subgroups.size());

    const auto selectedSubgroup = subgroupSelector_.getSelectedValue();

    int currentSubgroupId = 0;

    for (size_t i = 0; i < pointcloud.points.size(); i++) {

        if (!pointcloud.subgroups.empty()) {
            if (i > pointcloud.subgroups[currentSubgroupId].end) {
                currentSubgroupId++;
            }
        }

        if (selectedSubgroup == -1 || currentSubgroupId == selectedSubgroup) {
            vertices[i] = vec3(pointcloud.points[i], 0);
            colors[i] = colormap[currentSubgroupId];
            const auto v = pointcloud.vel[i];
            radii[i] = (double)pointSize_.get() * (1.f + length(v) * velocityScaling_.get());
        }
    }

    return mesh;
}

template <typename Format>
std::shared_ptr<Image> AnsysFieldReader2D::pointCloudToImage2D(
    PointCloud& pointcloud, std::function<void(LayerRAM*, size_t, size_t, size_t)> fillFn) {

    vec2 min{std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()};
    vec2 max{-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()};

    for (const auto node : pointcloud.points) {
        if (node.x < min.x) min.x = node.x;
        if (node.y < min.y) min.y = node.y;
        if (node.x > max.x) max.x = node.x;
        if (node.y > max.y) max.y = node.y;
    }

    auto layer = std::make_shared<Layer>(
        size_, Format::get(), LayerType::Color,
        SwizzleMask{ImageChannel::Red, ImageChannel::Green, ImageChannel::Zero, ImageChannel::One});
    auto ram = layer->getEditableRepresentation<LayerRAM>();

    const auto distance = [](vec2 a, vec2 b) {
        vec2 ab = {b.x - a.x, b.y - a.y};
        return std::sqrt(ab.x * ab.x + ab.y * ab.y);
    };

    float d_min = std::numeric_limits<float>::infinity(),
          d_max = -std::numeric_limits<float>::infinity(), d_sum = 0;

    for (size_t i = 0; i < size_.get().x; i++) {
        for (size_t j = 0; j < size_.get().y; j++) {

            float x = min.x + (max.x - min.x) * ((float)i / size_.get().x);
            float y = min.y + (max.y - min.y) * ((float)j / size_.get().y);

            size_t nearestNeighbor = 0;
            float smallestDistance = std::numeric_limits<float>::infinity();

            for (size_t k = 0; k < pointcloud.points.size(); k++) {

                if (distance(pointcloud.points[k], {x, y}) < smallestDistance) {
                    nearestNeighbor = k;
                    smallestDistance = distance(pointcloud.points[nearestNeighbor], {x, y});
                }
            }

            if (smallestDistance < d_min) {
                d_min = smallestDistance;
            }

            if (smallestDistance > d_max) {
                d_max = smallestDistance;
            }

            d_sum += smallestDistance;

            fillFn(ram, i, j, nearestNeighbor);
        }
    }

    minDistance_.set(d_min);
    maxDistance_.set(d_max);
    avgDistance_.set(d_sum / (float)(size_.get().x * size_.get().y));
    distanceTolerance_.setMinValue(minDistance_);
    distanceTolerance_.setMaxValue(maxDistance_);

    return std::make_shared<Image>(layer);
}

}  // namespace inviwo

Real intersectPlane(Real3 pos, Real3 dir, Real3 middle, Real3 normal) {
    const Real a = dot(dir, normal);
    if (a >= 0.0) return -1.0;  // facing away
    const Real b = dot(middle - pos, normal);
    if (b >= 0.0) return -1.0;  // behind or on plane
    return b / a;
}