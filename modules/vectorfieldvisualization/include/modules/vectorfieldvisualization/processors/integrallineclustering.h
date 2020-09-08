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

#ifndef IVW_INTEGRALLINECLUSTERING_H
#define IVW_INTEGRALLINECLUSTERING_H

#include <modules/vectorfieldvisualization/vectorfieldvisualizationmoduledefine.h>

#include <inviwo/core/common/inviwo.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/ports/dataoutport.h>
#include <inviwo/core/ports/datainport.h>

#include <inviwo/core/properties/stringproperty.h>
#include <inviwo/core/properties/ordinalproperty.h>

#include <inviwo/core/properties/boolproperty.h>
#include <inviwo/core/properties/boolcompositeproperty.h>
#include <inviwo/core/properties/buttonproperty.h>
#include <inviwo/core/properties/minmaxproperty.h>
#include <inviwo/core/properties/optionproperty.h>
#include <inviwo/core/properties/compositeproperty.h>

#include <modules/vectorfieldvisualization/datastructures/integralline.h>
#include <modules/vectorfieldvisualization/datastructures/integrallineset.h>
#include <inviwo/core/util/spatialsampler.h>

#include <vector>
#include <array>
#include <map>

namespace inviwo {

namespace LineSimilarity {
const auto MCPD = [](IntegralLine a, IntegralLine b) {
    const auto inner = [](std::vector<dvec3> P_a, std::vector<dvec3> P_b) {
        double sum = .0;
        for (const auto p_a : P_a) {
            double min = std::numeric_limits<float>::max();
            for (const auto p_b : P_b) {
                min = std::min(min, glm::distance(p_a, p_b));
            }
            sum += min;
        }
        return sum / P_a.size();
    };
    return (float)std::min(inner(a.getPositions(), b.getPositions()),
                           inner(b.getPositions(), a.getPositions()));
};
}

namespace ClusteringAlgorithms {
class AHC {
    std::vector<std::vector<IntegralLine>> clusters;
    std::vector<float> height;
    std::vector<std::vector<float>> distanceMatrix;

public:
    AHC() {}
    void init(std::vector<IntegralLine> lines,
              std::function<float(IntegralLine, IntegralLine)> similarityFn) {
        reset();
        height.resize(lines.size(), std::numeric_limits<float>::infinity());
        for (size_t i = 0; i < lines.size(); i++) {
            clusters.push_back({
                lines[i],
            });

            distanceMatrix.push_back(std::vector<float>(lines.size()));
            for (size_t j = 0; j < i; j++) {
                const auto dist = similarityFn(lines[i], lines[j]);
                distanceMatrix[i][j] = dist;
                distanceMatrix[j][i] = dist;
            }
        }
    }
    bool ready() { return !distanceMatrix.empty(); }
    void reset() {
        clusters.clear();
        height.clear();
        distanceMatrix.clear();
    }
    float merge() {
        float minDist = std::numeric_limits<float>::max();
        size_t xMin = 0, yMin = 0;
        for (size_t x = 0; x < distanceMatrix.size(); x++) {
            if (height[x] < std::numeric_limits<float>::infinity()) continue;
            for (size_t y = 0; y < x; y++) {
                if (height[y] < std::numeric_limits<float>::infinity()) continue;
                if (distanceMatrix[x][y] < minDist) {
                    minDist = distanceMatrix[x][y];
                    xMin = x;
                    yMin = y;
                }
            }
        }

        height[xMin] = minDist;

        clusters[yMin].insert(clusters[yMin].end(), clusters[xMin].begin(), clusters[xMin].end());
        clusters[xMin].resize(0);

        for (int i = 0; i < height.size(); i++) {
            distanceMatrix[i][yMin] = (distanceMatrix[i][xMin] + distanceMatrix[i][yMin]) / 2.f;
            distanceMatrix[yMin][i] = (distanceMatrix[xMin][i] + distanceMatrix[yMin][i]) / 2.f;
        }

        return minDist;
    }
    size_t findBestClusterCount() {
        std::vector<float> mergeDistances;
        for (size_t i = 0; i < distanceMatrix.size() - 1; i++) {
            mergeDistances.push_back(merge());
        }
        vec2 A(0, mergeDistances[0]);
        vec2 B(mergeDistances.size() - 1, mergeDistances.back());
        float max = 0;
        size_t nClusters = 0;
        for (size_t i = mergeDistances.size() / 2; i < mergeDistances.size(); i++) {
            vec2 C(i, mergeDistances[i]);
            float angle = glm::acos(glm::dot(glm::normalize(B - A), glm::normalize(C - A)));
            float d = glm::distance(A, C) * glm::sin(angle);
            if (d > max) {
                max = d;
                nClusters = mergeDistances.size() - i;
            }
        }
        return nClusters;
    }
    size_t countClusters() {
        size_t count = 0;
        for (const auto& cluster : clusters) {
            if (cluster.size() > 0) {
                count++;
            }
        }
        return count;
    }
    std::vector<IntegralLine> getCluster(size_t i) {
        size_t count = 0;
        for (const auto& cluster : clusters) {
            if (cluster.size() > 0) {
                if (i == count) {
                    return cluster;
                }
                count++;
            }
        }
        return {};
    }
};
}  // namespace ClusteringAlgorithms

namespace ClusterRepresentatives {
IntegralLine distanceBased(
    std::vector<IntegralLine> cluster,
    std::function<float(IntegralLine, IntegralLine)> similarityFn = LineSimilarity::MCPD);

IntegralLine lengthBased(std::vector<IntegralLine> cluster);

using R = float;
using Index = size_t;
using Node = glm::vec3;

const auto INF = std::numeric_limits<R>::infinity();

struct AABB {
    Node min, max;
};

struct Resolution {
    size_t x, y, z;
    Resolution(size_t x, size_t y, size_t z) : x(x), y(y), z(z) {}
    Resolution(AABB aabb, R step) {
        x = static_cast<size_t>((aabb.max.x - aabb.min.x) / step);
        y = static_cast<size_t>((aabb.max.y - aabb.min.y) / step);
        z = static_cast<size_t>((aabb.max.z - aabb.min.z) / step);
    }
};

enum ExecutionMode { SEQUENTIAL, PARALLEL };

Node mapIndexToNode(Index i, Node min, Resolution res, R h_step);

Index mapNodeToIndex(Node p, Node min, Resolution res, R h_step);

void linspace(const AABB aabb, const R h_step, const ExecutionMode mode,
              const std::function<void(Node, Index)> fn);

IntegralLine densityBased(std::vector<IntegralLine> cluster, int maxRes = 100);
}  // namespace ClusterRepresentatives

class IVW_MODULE_VECTORFIELDVISUALIZATION_API IntegralLineClustering : public Processor {
public:
    IntegralLineClustering();
    virtual ~IntegralLineClustering() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    IntegralLineSetInport in_;
    DataInport<SpatialSampler<3, 3, double>, 1> velocitySampler_;
    IntegralLineSetOutport out_;
    DataOutport<std::vector<vec4>> colors_;

    IntProperty nClusters_;
    ButtonProperty init_;
    ButtonProperty compute_;
    ButtonProperty findClusterCount_;

    CompositeProperty representatives_;
    BoolProperty distanceBasedRepresentative_;
    BoolProperty lengthBasedRepresentative_;
    BoolProperty diffBasedRepresentative_;
    BoolProperty densityBasedRepresentative_;
    IntProperty densityMapResolution_;

    ClusteringAlgorithms::AHC ahc_;

};

}  // namespace inviwo

#endif  // IVW_INTEGRALLINECLUSTERING_H
