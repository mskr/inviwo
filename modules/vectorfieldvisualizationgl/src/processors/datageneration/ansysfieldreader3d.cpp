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

#include <modules/vectorfieldvisualizationgl/processors/datageneration/ansysfieldreader3d.h>
#include <inviwo/core/util/colorbrewer.h>
#include <inviwo/core/interaction/events/mouseevent.h>
#include <inviwo/core/datastructures/geometry/plane.h>
#include <modules/base/algorithm/randomutils.h>
#include <random>
#include <fstream>

namespace SimFileReading3D {

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

using namespace SimData3D;

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
    } catch (const std::out_of_range e) {
        throw e;
    }

    out.subgroups.clear();
    out.points.clear();
    out.vel.clear();
    out.dist.clear();
    out.WSS.clear();
    out.press.clear();

    SelectedQuantity vel(quantities, "[x-velocity,y-velocity,z-velocity]");
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

        out.points.push_back({numbers[1], numbers[2], numbers[3]});

        if (vel.found)
            out.vel.push_back(
                {numbers[vel.valueRefs[0]], numbers[vel.valueRefs[1]], numbers[vel.valueRefs[2]]});

        if (dist.found) out.dist.push_back(numbers[dist.valueRefs[0]]);

        if (wss.found) out.WSS.push_back(numbers[wss.valueRefs[0]]);

        if (press.found) out.press.push_back(numbers[press.valueRefs[0]]);

        count++;
    }

    if (countSinceGroupStart > 0)
        out.subgroups.push_back({countSinceGroupStart, count - 1, groupID});
}
}  // namespace SimFileReading3D

namespace inviwo {

struct BoundedPlane {
    dvec3 origin, up, right;
};

const ProcessorInfo AnsysFieldReader3D::processorInfo_{
    "org.inviwo.AnsysFieldReader3D",  // Class identifier
    "ANSYS Field ASCII Reader 3D",    // Display name
    "Data Creation",                  // Category
    CodeState::Stable,                // Code state
    Tags::GL,                         // Tags
};
const ProcessorInfo AnsysFieldReader3D::getProcessorInfo() const { return processorInfo_; }

using namespace SimDataSampling3D;

AnsysFieldReader3D::AnsysFieldReader3D()
    : Processor()
    , boundaries_("boundaries")
    , seedSurface_("seedSurface")
    , pointCloud_("pointCloud")
    , sampler_("sampler")
    , insideTest_("insideTest")
    , streamlineSeeds_("streamlineSeeds")
    , file_("filename", "File")
    , readButton_("readButton", "Read")
    , seedButton_("seedButton", "Seed on Input Surface")
    , numSeeds_("numSeeds", "Seedpoints", 10, 1, 1000)
    , subgroupSelector_("subgroupSelector", "Subgroup")
    , pointcloudVis_("pointCloudVis", "Raw View")
    , pointSize_("pnts", "Point Size")
    , velocityScaling_("velocityScaling", "Velocity")
    , spaceStats_("spaceStats", "Grid Info")
    , pointCloudSize_("pointCloudSize", "Points", 0, 0, std::numeric_limits<size_t>::max())
    , computeStepButton_("computeStep", "Compute Step (Expensive!)")
    , pointCloudMinNearestDistance_("pointCloudMinDistance", "Min Grid Step")
    , pointCloudMaxNearestDistance_("pointCloudMaxDistance", "Max Grid Step")
    , pointCloudAvgNearestDistance_("pointCloudAvgDistance", "Avg Grid Step")
    , pointCloudMin_("pointCloudMin", "Pointcloud Min", vec3(0), vec3(-100000), vec3(100000))
    , pointCloudMax_("pointCloudMax", "Pointcloud Max", vec3(0), vec3(-100000), vec3(100000))
    , meshMin_("meshMin", "Mesh Min", vec3(0), vec3(-100000), vec3(100000))
    , meshMax_("meshMax", "Mesh Max", vec3(0), vec3(-100000), vec3(100000))
    , interactive_("interactive", "Interactive")
    , enableInteractive_("enableInteractive", "Enable")
    , hoverEvents_(
          "hoverEvents", "Hover Events", [this](Event* e) { processPickEvent(e); },
          MouseButton::None, MouseState::Move)
    , clickEvents_(
          "clickEvents", "Click Events", [this](Event* e) { processPickEvent(e); },
          MouseButton::Left, MouseState::Press)
    , insideTestPoint_("insideTestPoint", "Inside Test Point", vec3(0), vec3(-1000), vec3(1000)) {

    addPort(boundaries_);
    addPort(seedSurface_);
    boundaries_.setOptional(true);
    seedSurface_.setOptional(true);

    addPort(pointCloud_);
    addPort(sampler_);
    addPort(insideTest_);
    addPort(streamlineSeeds_);

    addProperties(file_, readButton_, seedButton_, numSeeds_, subgroupSelector_, pointcloudVis_,
                  spaceStats_, interactive_);

    pointcloudVis_.addProperties(pointSize_, velocityScaling_);

    spaceStats_.addProperties(pointCloudSize_, computeStepButton_, pointCloudMinNearestDistance_,
                              pointCloudMaxNearestDistance_, pointCloudAvgNearestDistance_,
                              pointCloudMin_, pointCloudMax_, meshMin_, meshMax_);
    pointCloudSize_.setSemantics(PropertySemantics::Text);
    for (auto prop : spaceStats_.getProperties()) prop->setReadOnly(true);
    computeStepButton_.setReadOnly(false);

    interactive_.addProperties(enableInteractive_, hoverEvents_, clickEvents_, insideTestPoint_);

    seedButton_.onChange([&]() { seedOnInputSurface(); });

    readButton_.onChange([&]() { process(); });

    computeStepButton_.onChange([&]() {
        using namespace std::chrono;
        using Clock = high_resolution_clock;
        using T = time_point<Clock>;
        auto sec = [](T t0, T t1) { return duration_cast<seconds>((t1) - (t0)).count(); };
        auto t0 = Clock::now();

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

        LogInfo("Analyzed grid step in " << sec(t0, Clock::now()) << " seconds.");

        pointCloudMinNearestDistance_.set(minNearestDist);
        pointCloudMaxNearestDistance_.set(maxNearestDist);
        pointCloudAvgNearestDistance_.set(avgNearestDist);

        pointSize_.set(avgNearestDist / 2.f);
        pointSize_.setMaxValue(maxNearestDist * 3.f);
        velocityScaling_.set(.8f);
        velocityScaling_.setMinValue(-2.f);
        velocityScaling_.setMaxValue(2.f);
    });

    subgroupSelector_.onChange([&]() {
        pointCloud_.setData(pointCloudToMesh3D(pointcloud));

        if (!seedSurface_.hasData()) {
            const auto selected = subgroupSelector_.getSelectedValue();
            const auto start = selected >= 0 ? pointcloud.subgroups[selected].start : 0;
            const auto end =
                selected >= 0 ? pointcloud.subgroups[selected].end : pointcloud.points.size();
            auto seeds = std::make_shared<std::vector<vec3>>(pointcloud.points.begin() + start,
                                                             pointcloud.points.begin() + end);
            if (end - start > numSeeds_.get()) {
                auto reduced = std::make_shared<std::vector<vec3>>();
                std::default_random_engine generator;
                std::uniform_int_distribution<int> distribution(0, seeds->size());
                for (int i = 0; i < 500; i++)
                    reduced->push_back(seeds->at(distribution(generator)));
                seeds = reduced;
            }
            streamlineSeeds_.setData(seeds);
        }
    });

    insideTestPoint_.onChange([this]() {
        if (velocitySampler) {
            auto mesh = std::make_shared<Mesh>(DrawType::Points, ConnectivityType::None);

            auto vertexRAM = std::make_shared<BufferRAMPrecision<vec3>>(2);
            auto colorRAM = std::make_shared<BufferRAMPrecision<vec4>>(2);
            auto radiiRAM = std::make_shared<BufferRAMPrecision<float>>(2);

            mesh->addBuffer(Mesh::BufferInfo(BufferType::PositionAttrib),
                            std::make_shared<Buffer<vec3>>(vertexRAM));
            mesh->addBuffer(Mesh::BufferInfo(BufferType::ColorAttrib),
                            std::make_shared<Buffer<vec4>>(colorRAM));
            mesh->addBuffer(Mesh::BufferInfo(BufferType::RadiiAttrib),
                            std::make_shared<Buffer<float>>(radiiRAM));

            auto& vertices = vertexRAM->getDataContainer();
            auto& colors = colorRAM->getDataContainer();
            auto& radii = radiiRAM->getDataContainer();

            auto rayStartOffset = dvec3(0.5);

            vertices[0] = vec3(insideTestPoint_.get());
            vertices[1] = velocitySampler->meshAABB.min - rayStartOffset;

            if (velocitySampler->inside(insideTestPoint_.get(), rayStartOffset)) {
                colors[0] = vec4(0, 1, 0, 1);
            } else {
                colors[0] = vec4(1, 0, 0, 1);
            }
            colors[1] = vec4(1);

            radii[0] = (double)pointSize_.get();
            radii[1] = (double)pointSize_.get();

            insideTest_.setData(mesh);
        }
    });
}  // namespace inviwo

AnsysFieldReader3D::~AnsysFieldReader3D() = default;

void AnsysFieldReader3D::initializeResources() {}

void AnsysFieldReader3D::process() {

    using namespace std::chrono;
    using Clock = high_resolution_clock;
    using T = time_point<Clock>;
    auto sec = [](T t0, T t1) { return duration_cast<seconds>((t1) - (t0)).count(); };
    auto millis = [](T t0, T t1) { return duration_cast<milliseconds>((t1) - (t0)).count(); };

    T t0 = Clock::now();

    SimFileReading3D::read(file_.get(), pointcloud);

    if (pointcloud.vel.empty()) {
        LogWarn("Velocity not found => abort.");
        return;
    }

    pointCloudSize_.set(pointcloud.points.size());

    if (!boundaries_.hasData()) {
        LogWarn("Read pointcloud without boundary mesh => no sampler, no rescale.");
        pointCloud_.setData(pointCloudToMesh3D(pointcloud));
        return;
    } else {
        createSampler(*boundaries_.getData());
    }

    const auto dur = sec(t0, Clock::now());
    LogInfo("Read field into sampler in " << dur << " seconds");

    auto testPoint = pointcloud.points[pointcloud.points.size() / 2] + vec3(0.01);
    t0 = Clock::now();
    if (velocitySampler->inside(testPoint)) {
        LogInfo("Inside test takes " << millis(t0, Clock::now()) << " millis.");
        t0 = Clock::now();
        auto v =
            pointcloud.vel[*velocitySampler->kdtree.findNearest(testPoint)->getDataAsPointer()];
        LogInfo("Lookup takes " << millis(t0, Clock::now()) << " millis.");
    }

    subgroupSelector_.clearOptions();
    subgroupSelector_.addOption("all", "All", -1);
    for (const auto g : pointcloud.subgroups) {
        subgroupSelector_.addOption(std::to_string(g.id), std::to_string(g.id), g.id);
    }

    if (seedSurface_.hasData()) seedOnInputSurface();
}

void AnsysFieldReader3D::createSampler(const Mesh& boundaries) {
    velocitySampler = std::make_shared<PointCloudVelocitySampler>(pointcloud, boundaries);

    pointCloudMin_.set(velocitySampler->pointCloudAABB.min);
    pointCloudMax_.set(velocitySampler->pointCloudAABB.max);
    meshMin_.set(velocitySampler->meshAABB.min);
    meshMax_.set(velocitySampler->meshAABB.max);
    insideTestPoint_.setMinValue(meshMin_.get() - vec3(.5f));
    insideTestPoint_.setMaxValue(meshMax_.get() + vec3(.5f));

    sampler_.setData(velocitySampler);
}

void AnsysFieldReader3D::seedOnInputSurface() {

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
        dvec3 lower(0, std::numeric_limits<double>::infinity(), 0);
        dvec3 upper(0, -std::numeric_limits<double>::infinity(), 0);
        dvec3 xstart(std::numeric_limits<double>::infinity(), 0, 0);
        dvec3 xend(-std::numeric_limits<double>::infinity(), 0, 0);
        for (size_t i = 0; i < verts->getSize(); i++) {
            auto v = verts->getAsDVec3(i);
            if (v.y < lower.y) lower = v;
            if (v.y > upper.y) upper = v;
            if (v.x < xstart.x) xstart = v;
            if (v.x > xend.x) xend = v;
        }
        result.up = upper - lower;
        auto n = glm::cross(result.up, xend - xstart);
        auto rightAxis = glm::normalize(glm::cross(result.up, n));
        dvec3 leftest = rightAxis * (std::numeric_limits<double>::infinity());
        dvec3 rightest = rightAxis * (-std::numeric_limits<double>::infinity());
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
    while (gridSize * gridSize < numSeeds_.get()) gridSize++;

    for (size_t i = 0; i < numSeeds_.get(); i++) {
        float x = (float)(i % gridSize) / (float)gridSize;
        float y = (float)(i / gridSize) / (float)gridSize;
        const auto p = vec3(pl.origin) + x * vec3(pl.up) + ((y * 1.2f - 0.5f) * vec3(pl.right));
        if (insideSeedSurface(p, surface, pl)) {
            seeds->push_back(p);
            colors.push_back(vec4(0, 1, 0, 1));
        }
    }

    std::default_random_engine generator;
    while (seeds->size() < numSeeds_.get()) {
        const float x = util::randomNumber<float>(generator);
        const float y = util::randomNumber<float>(generator);
        const auto p = vec3(pl.origin) + x * vec3(pl.up) + ((y * 1.2f - 0.5f) * vec3(pl.right));
        if (insideSeedSurface(p, surface, pl)) {
            seeds->push_back(p);
            colors.push_back(vec4(0, 1, 0, 1));
        }
    }

    streamlineSeeds_.setData(seeds);
    insideTest_.setData(pointMesh(seeds, colors));
};

std::shared_ptr<Mesh> AnsysFieldReader3D::pointCloudToMesh3D(PointCloud& pointcloud) {
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

        vertices[i] = vec3(pointcloud.points[i]);
        radii[i] = pointSize_.get() * (1.f + length(pointcloud.vel[i]) * velocityScaling_.get());

        if (selectedSubgroup == -1 || currentSubgroupId == selectedSubgroup) {
            colors[i] = colormap[currentSubgroupId];
        } else {
            colors[i] = vec4(0);
        }
    }

    return mesh;
}

void AnsysFieldReader3D::processPickEvent(Event* e) {
    if (!enableInteractive_) return;

    const auto mouseEvent = static_cast<MouseEvent*>(e);
    auto mousePos = vec2(mouseEvent->posNormalized());

    auto coords = meshMax_.get() - meshMin_.get();

    auto val = meshMin_.get() + vec3(mousePos * vec2(coords), insideTestPoint_.get().z);

    insideTestPoint_.set(val);

    /*const auto pt = mousePos;
    const auto camPos = cam_.getLookFrom();
    const auto nearPt = cam_.getWorldPosFromNormalizedDeviceCoords(vec3(pt * 2.f - 1.f, -1.f));
    const auto viewDir = glm::normalize(nearPt - camPos);
    const auto inter = Plane(insideTestPoint_.get(), -viewDir)
                           .getIntersection(nearPt, nearPt + viewDir * cam_.getFarPlaneDist());
    insideTestPoint_.set(inter.value_or(insideTestPoint_.get()));*/
}

}  // namespace inviwo

Real intersectPlaneBidirectional(Real3 pos, Real3 dir, Real3 middle, Real3 normal) {
    const Real a = dot(dir, normal);
    const Real b = dot(middle - pos, normal);
    if (a >= 0.0 && b <= 0.0 || a <= 0.0 && b >= 0.0) return std::numeric_limits<Real>::infinity();
    return b / a;
}