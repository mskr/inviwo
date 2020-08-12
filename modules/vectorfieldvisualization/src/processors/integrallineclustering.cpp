/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2016-2020 Inviwo Foundation
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

#include <modules/vectorfieldvisualization/processors/integrallineclustering.h>
#include <modules/vectorfieldvisualization/processors/3d/pathlines.h>
#include <modules/vectorfieldvisualization/processors/3d/streamlines.h>
#include <inviwo/core/util/colorbrewer.h>
#include <inviwo/core/util/colorconversion.h>

namespace inviwo {

const ProcessorInfo IntegralLineClustering::getProcessorInfo() const { return processorInfo_; }

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo IntegralLineClustering::processorInfo_{
    "org.inviwo.IntegralLineClustering",  // Class identifier
    "Integral Line Clustering",           // Display name
    "Vector Field Visualization",         // Category
    CodeState::Stable,                    // Code state
    Tags::CPU,                            // Tags
};

IntegralLineClustering::IntegralLineClustering()
    : Processor()
    , in_("in")
    , out_("out")
    , colors_("colors")
    , nClusters_("nCluster", "Clusters", 20, 1, 500)
    , init_("init", "Make Clusters")
    , compute_("compute", "-1 Cluster")
    , findClusterCount_("findClusterCount", "Find Cluster Count")
    , representatives_("rep", "Representatives")
    , distanceBasedRepresentative_("distRep", "Distance")
    , lengthBasedRepresentative_("lengthRep", "Length")
    , densityBasedRepresentative_("densityRep", "Density")
    , densityMapResolution_("densityMapRes", "Density Map", 100, 10, 500) {

    addPort(in_);
    addPort(out_);
    addPort(colors_);
    addProperties(nClusters_, init_, compute_, findClusterCount_, representatives_);
    representatives_.addProperties(distanceBasedRepresentative_, lengthBasedRepresentative_,
                                   densityBasedRepresentative_, densityMapResolution_);

    init_.onChange([&]() { ahc_.reset(); });
    compute_.onChange([&]() {
        if (in_.hasData()) {
            if (nClusters_.get() > 1) {
                ahc_.merge();
                nClusters_.set(ahc_.countClusters());
            }
        }
    });

    findClusterCount_.onChange([&]() {
        if (ahc_.ready()) {
            auto nClusters = ClusteringAlgorithms::AHC(ahc_).findBestClusterCount();
            LogInfo("Best estimated cluster count is " << nClusters);
        }
    });
}

void IntegralLineClustering::process() {

    if (in_.hasData()) {
        if (!ahc_.ready() || in_.isChanged()) {
            auto lines = in_.getData()->getVector();
            nClusters_.setMaxValue(lines.size());
            if (nClusters_.get() < lines.size()) {
                using namespace std::chrono;
                using Clock = high_resolution_clock;
                using T = time_point<Clock>;
                auto sec = [](T t0, T t1) { return duration_cast<seconds>((t1) - (t0)).count(); };
                auto t0 = Clock::now();

                ahc_.init(lines, LineSimilarity::MCPD);

                LogInfo("Distance matrix initialized in " << sec(t0, Clock::now()) << " seconds.");

                auto nMerges = lines.size() - nClusters_.get();
                for (size_t i = 0; i < nMerges; i++) ahc_.merge();
            } else {
                out_.setData(in_.getData());
                colors_.setData(std::make_shared<std::vector<vec4>>(
                    in_.getData()->getVector().size(), vec4(1)));
                return;
            }
        }
    }

    const auto out = std::make_shared<IntegralLineSet>(in_.getData()->getModelMatrix(),
                                                       in_.getData()->getWorldMatrix());
    const auto colors = std::make_shared<std::vector<vec4>>();
    std::vector<vec4> colormap;
    float colorstep = 1.f / nClusters_.get();
    for (int i = 0; i < nClusters_.get(); i++) {
        colormap.push_back(vec4(color::hsv2rgb(vec3(colorstep * i, 1.f, 1.f)), 1.f));
    }

    int count = 0;
    for (size_t i = 0; i < nClusters_.get(); i++) {
        const auto cluster = ahc_.getCluster(i);

        if (cluster.empty()) continue;

        if (distanceBasedRepresentative_.get()) {
            IntegralLine rep = ClusterRepresentatives::distanceBased(cluster, LineSimilarity::MCPD);
            out->push_back(rep, count++);
            colors->push_back(colormap[i]);
        }

        if (lengthBasedRepresentative_.get()) {
            IntegralLine rep = ClusterRepresentatives::lengthBased(cluster);
            out->push_back(rep, count++);
            colors->push_back(colormap[i]);
        }

        if (densityBasedRepresentative_.get()) {
            IntegralLine rep =
                ClusterRepresentatives::densityBased(cluster, densityMapResolution_.get());
            out->push_back(rep, count++);
            colors->push_back(colormap[i]);
        }

        if (!distanceBasedRepresentative_.get() && !lengthBasedRepresentative_.get() &&
            !densityBasedRepresentative_.get()) {
            for (size_t j = 0; j < cluster.size(); j++) {
                out->push_back(cluster[j], count++);
                colors->push_back(colormap[i]);
            }
        }
    }

    out_.setData(out);
    colors_.setData(colors);
}

namespace ClusterRepresentatives {
IntegralLine distanceBased(std::vector<IntegralLine> cluster,
                           std::function<float(IntegralLine, IntegralLine)> similarityFn) {
    IntegralLine rep = cluster[0];
    float min = std::numeric_limits<float>::max();
    for (const auto& line : cluster) {
        float sum = 0;
        for (const auto& other : cluster) {
            sum += similarityFn(line, other);
        }
        float avg = sum / cluster.size();
        if (avg < min) {
            min = avg;
            rep = line;
        }
    }
    return rep;
}

IntegralLine lengthBased(std::vector<IntegralLine> cluster) {
    IntegralLine rep = cluster[0];
    float max = 0;
    for (const auto& line : cluster) {
        const auto length = line.getLength();
        if (length > max) {
            max = length;
            rep = line;
        }
    }
    return rep;
}

IntegralLine densityBased(std::vector<IntegralLine> cluster, int maxRes) {
    vec3 min(std::numeric_limits<float>::max());
    vec3 max(.0f);
    for (const auto& line : cluster) {
        for (const auto& p : line.getPositions()) {
            if (p.x < min.x) min.x = p.x;
            if (p.y < min.y) min.y = p.y;
            if (p.z < min.z) min.z = p.z;
            if (p.x > max.x) max.x = p.x;
            if (p.y > max.y) max.y = p.y;
            if (p.z > max.z) max.z = p.z;
        }
    }

    const float step = std::max((max.x - min.x) / maxRes,
                                std::max((max.y - min.y) / maxRes, (max.z - min.z) / maxRes));
    if (max.z - min.z < step) {
        max.z = step / 2.f;
        min.z = -step / 2.f;
    }
    const AABB aabb{min, max};
    const Resolution res(aabb, step);
    const size_t volumeSize = res.x * res.y * res.z;

    std::vector<float> densities(volumeSize, .0f);

    linspace(aabb, step, PARALLEL, [&](vec3 p, Index i) {
        for (const auto& line : cluster) {
            for (const auto& lineP : line.getPositions()) {

                // FIXME exact line-voxel intersection
                if (glm::distance(p, vec3(lineP)) < step) densities[i] += 1.0f;
            }
        }
    });

    IntegralLine rep = cluster[0];
    float maxLineDensity = 0;
    for (const auto& line : cluster) {
        float lineDensity = .0f;
        for (const auto& lineP : line.getPositions()) {
            lineDensity += densities[mapNodeToIndex(lineP, aabb.min, res, step)];
        }
        if (lineDensity > maxLineDensity) {
            maxLineDensity = lineDensity;
            rep = line;
        }
    }

    // FIXME the representative line for a cluster changes non-deterministic
    return rep;
}

Node mapIndexToNode(Index i, Node min, Resolution res, R h_step) {
    const size_t lineSize = res.x;
    const size_t layerSize = lineSize * res.y;
    const size_t volumeSize = layerSize * res.z;

    const size_t numVoxelsPerLine = res.x;
    const size_t numLinesPerLayer = res.y;
    const size_t numLayers = res.z;

    R x = min.x + (i % numVoxelsPerLine) * h_step;
    R y = min.y + ((i / lineSize) % numLinesPerLayer) * h_step;
    R z = min.z + ((i / layerSize) % numLayers) * h_step;
    return {x, y, z};
}

Index mapNodeToIndex(Node p, Node min, Resolution res, R h_step) {
    const size_t lineSize = res.x;
    const size_t layerSize = lineSize * res.y;
    const size_t volumeSize = layerSize * res.z;

    const size_t voxelIndex = (size_t)glm::floor((p.x - min.x) / h_step);
    const size_t lineIndex = (size_t)glm::floor((p.y - min.y) / h_step);
    const size_t layerIndex = (size_t)glm::floor((p.z - min.z) / h_step);

    return voxelIndex + lineIndex * lineSize + layerIndex * layerSize;
}

void linspace(const AABB aabb, const R h_step, const ExecutionMode mode,
              const std::function<void(Node, Index)> fn) {

    const Resolution res(aabb, h_step);

    const size_t volumeSize = res.x * res.y * res.z;

    if (mode == SEQUENTIAL) {

        for (int i = 0; i < volumeSize; i++) {

            fn(mapIndexToNode(i, aabb.min, res, h_step), i);
        }
    }

    if (mode == PARALLEL) {
        const size_t numThreads = 6;

        std::vector<size_t> workloads(numThreads, volumeSize / numThreads);
        for (int i = 0; i < volumeSize % numThreads; i++) workloads[i]++;

        std::vector<std::thread> threads;

        Index start = 0, end = workloads[0];

        for (Index threadIndex = 0; threadIndex < workloads.size(); threadIndex++) {

            end = start + workloads[threadIndex];

            threads.push_back(std::thread([threadIndex, start, end, res, h_step, aabb, fn]() {
                for (Index i = start; i < end; i++) {

                    fn(mapIndexToNode(i, aabb.min, res, h_step), i);
                }
            }));

            start = end;
        }

        for (auto& thread : threads) thread.join();
    }
}

}  // namespace ClusterRepresentatives

}  // namespace inviwo
