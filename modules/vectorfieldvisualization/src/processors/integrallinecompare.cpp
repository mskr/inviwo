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

#include <modules/vectorfieldvisualization/processors/integrallinecompare.h>
#include <modules/vectorfieldvisualization/processors/3d/pathlines.h>
#include <modules/vectorfieldvisualization/processors/3d/streamlines.h>
#include <modules/vectorfieldvisualization/algorithms/integrallineoperations.h>
#include <modules/base/algorithm/meshutils.h>

namespace inviwo {

const ProcessorInfo IntegralLineCompare::getProcessorInfo() const { return processorInfo_; }

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo IntegralLineCompare::processorInfo_{
    "org.inviwo.IntegralLineCompare",  // Class identifier
    "Integral Line Compare",           // Display name
    "Vector Field Visualization",      // Category
    CodeState::Stable,                 // Code state
    Tags::CPU,                         // Tags
};

IntegralLineCompare::IntegralLineCompare()
    : Processor()
    , velocitySampler_("velocitySampler")
    , scalarSampler1_("scalarSampler1")
    , scalarSampler2_("scalarSampler2")
    , lines1_("lines1")
    , lines2_("lines2")
    , out_("out")
    , tubeMesh_("tubeMesh")
    , podMesh_("podMesh")
    , colors_("colors")
    , matchTolerance_("matchTolerance", "Match Tolerance", 0.3f, 0.0f, 0.3f)
    , noTriples_("noTriples", "No Triples")
    , tubes_("tubes", "Tubes")
    , referenceStreamlines_("referenceStreamlines", "Reference Streamlines")
    , referenceVis_("referenceVis", "Reference Streamines")
    , radiusScaling_("radiusScaling", "Radius Scaling")
    , minRadius_("minRadius", "Min Radius")
    , maxRadius_("maxRadius", "Max Radius")
    , precompute_("precompute", "Precompute")
    , compareMagnitude_("compareMagnitude", "Weight by Magnitude")
    , diverged_("diverged", "Diverged Lines, Point of Divergence")
    , splitThreshold_("splitThreshold", "Split Threshold")
    , podSize_("podSize", "POD Size", .1f, 0.f, 1.f)
    , podColor_("podColor", "POD Color", vec4(1.0f, 1.0f, 1.0f, 1.0f), vec4(0.0f), vec4(1.0f),
                vec4(0.01f), InvalidationLevel::InvalidOutput, PropertySemantics::Color)
    , earlyEndColor_("earlyEndColor", "Early End Color", vec4(1.0f, 1.0f, 1.0f, 1.0f), vec4(0.0f),
                     vec4(1.0f), vec4(0.01f), InvalidationLevel::InvalidOutput,
                     PropertySemantics::Color)
    , divergedLines_("divergedLines", "Line Size")
    , severity_("severity", "Severity")
    , severityMetric_("severityMetric", "Metric")
    , severityFilter_("severityFilter", "Filter", 1000, 1, 1000)
    , colorMaps_("colorMaps", "Colormaps")
    , enableThickness_("enableThickness", "Enable Thickness")
    , thickness_("thickness", "Thickness",
                 TransferFunction({{0.0f, vec4(0.0f, 0.1f, 1.0f, 1.0f)},
                                   {1.0f, vec4(1.0f, 0.03f, 0.03f, 1.0f)}}))
    , enableRunlength_("enableRunlength", "Enable Runlength")
    , runlength_("runlength", "Runlength",
                 TransferFunction({{0.0f, vec4(0.0f, 0.1f, 1.0f, 1.0f)},
                                   {1.0f, vec4(1.0f, 0.03f, 0.03f, 1.0f)}}))
    , enableScalar1_("enableScalar1", "Enable Scalar 1")
    , scalar1_("scalar1", "Scalar 1",
               TransferFunction(
                   {{0.0f, vec4(0.0f, 0.1f, 1.0f, 1.0f)}, {1.0f, vec4(1.0f, 0.03f, 0.03f, 1.0f)}}))
    , enableScalar2_("enableScalar2", "Enable Scalar 2")
    , scalar2_("scalar2", "Scalar 2",
               TransferFunction(
                   {{0.0f, vec4(0.0f, 0.1f, 1.0f, 1.0f)}, {1.0f, vec4(1.0f, 0.03f, 0.03f, 1.0f)}}))
    , enableScalarDiff_("enableScalarDiff", "Enable Scalar Diff")
    , scalarDiff_("scalarDiff", "Scalar Diff",
                  TransferFunction({{0.0f, vec4(0.0f, 0.1f, 1.0f, 1.0f)},
                                    {1.0f, vec4(1.0f, 0.03f, 0.03f, 1.0f)}})) {
    addPort(velocitySampler_);
    addPort(scalarSampler1_);
    addPort(scalarSampler2_);
    addPort(lines1_);
    addPort(lines2_);
    lines2_.setOptional(true);
    addPort(out_);
    addPort(tubeMesh_);
    addPort(podMesh_);
    addPort(colors_);

    addProperties(matchTolerance_, noTriples_, tubes_, referenceStreamlines_, referenceVis_,
                  diverged_, severity_, colorMaps_);
    referenceVis_.addProperties(radiusScaling_, minRadius_, maxRadius_, precompute_,
                                compareMagnitude_);
    diverged_.addProperties(splitThreshold_, podSize_, podColor_, earlyEndColor_, divergedLines_);
    severity_.addProperties(severityMetric_, severityFilter_);

    colorMaps_.addProperties(enableScalar1_, scalar1_, enableScalar2_, scalar2_, enableScalarDiff_,
                             scalarDiff_, enableThickness_, thickness_, enableRunlength_,
                             runlength_);

    severityMetric_.addOption("runlength", "Runlength", SeverenessMetric::Runlength);
    severityMetric_.addOption("runlengthUntilDiverged", "Runlength Until Diverged",
                              SeverenessMetric::RunlengthUntilDiverged);
    severityMetric_.addOption("thickness", "Thickness", SeverenessMetric::ThicknessSum);
    severityMetric_.addOption("lengthAndThickness", "Length And Thickness",
                              SeverenessMetric::LengthAndThickness);
    severityMetric_.set(SeverenessMetric::LengthAndThickness);
    severityMetric_.setCurrentStateAsDefault();

    runlength_.setReadOnly(!enableRunlength_);
    scalar1_.setReadOnly(!enableScalar1_);
    scalar2_.setReadOnly(!enableScalar2_);
    scalarDiff_.setReadOnly(!enableScalarDiff_);
    enableRunlength_.onChange([&]() { runlength_.setReadOnly(!enableRunlength_); });
    enableScalar1_.onChange([&]() { scalar1_.setReadOnly(!enableScalar1_); });
    enableScalar2_.onChange([&]() { scalar2_.setReadOnly(!enableScalar2_); });
    enableScalarDiff_.onChange([&]() { scalarDiff_.setReadOnly(!enableScalarDiff_); });

    using namespace std::chrono;
    using Clock = high_resolution_clock;
    using T = time_point<Clock>;
    auto sec = [](T t0, T t1) { return duration_cast<seconds>((t1) - (t0)).count(); };
    auto millis = [](T t0, T t1) { return duration_cast<milliseconds>((t1) - (t0)).count(); };

    auto sampleAlongRefLines = [&]() {
        auto t0 = Clock::now();
        if (!lines1_.hasData()) return;
        if (!velocitySampler_.hasData()) return;
        const auto sampler = velocitySampler_.getData();
        deviationsMagnitude_ = {};
        deviationsDirection_ = {};
        const auto set1 = lines1_.getData();
        float maxDeviationMagnitude = .0f;
        float maxSumMagnitude = .0f;
        for (const auto& refLine : set1->getVector()) {
            const auto& positions = refLine.getPositions();
            const auto& velocities = refLine.getMetaData<dvec3>("velocity");

            std::vector<dvec3> otherVelocities;

            std::vector<float> deviationsMagnitude;
            std::vector<float> deviationsDirection;
            std::vector<float> sumsMagnitude;

            for (size_t i = 0; i < positions.size(); i++) {

                otherVelocities.push_back(sampler->sample(positions[i]));

                // formula
                auto d = glm::abs(glm::length(otherVelocities[i]) - glm::length(velocities[i]));
                auto sum = glm::length(otherVelocities[i]) + glm::length(velocities[i]);

                deviationsMagnitude.push_back(d);
                sumsMagnitude.push_back(sum);
                if (d > maxDeviationMagnitude) maxDeviationMagnitude = d;
                if (sum > maxSumMagnitude) maxSumMagnitude = sum;
            }

            for (size_t i = 0; i < positions.size(); i++) {
                // formula
                deviationsDirection.push_back(
                    1.f - .5f * (1.f + glm::dot(glm::normalize(otherVelocities[i]),
                                                glm::normalize(velocities[i]))));
            }

            deviationsMagnitude_.push_back(deviationsMagnitude);
            deviationsDirection_.push_back(deviationsDirection);
        }

        maxRadius_.setMaxValue(maxDeviationMagnitude);

        LogInfo("Sample velocity along " << set1->size() << " ref lines took " << millis(t0, Clock::now()) << " ms.");
    };

    precompute_.onChange(sampleAlongRefLines);
    velocitySampler_.onChange(sampleAlongRefLines);
    lines1_.onChange(sampleAlongRefLines);

    auto getScalarsFrom1 = [&]() {
        auto t0 = Clock::now();
        if (!scalarSampler1_.hasData()) return;
        auto sampler = scalarSampler1_.getData();
        scalarsLineSet1_ = {};
        scalarsLineSetFrom2To1_ = {};
        if (lines1_.hasData()) {
            const auto set1 = lines1_.getData();
            float max = .0f;
            for (auto l : set1->getVector()) {
                auto positions = l.getPositions();
                std::vector<float> scalars;
                for (auto p : positions) {
                    auto s = sampler->sample(p);
                    scalars.push_back(s);
                    if (s > max) max = s;
                }
                scalarsLineSet1_.push_back(scalars);
            }
            for (auto& scalars : scalarsLineSet1_)
                for (auto& s : scalars) s /= max;
        }
        if (lines2_.hasData() && !referenceStreamlines_) {
            const auto set2 = *lines2_.getData();
            float max = .0f;
            for (auto l : set2.getVector()) {
                auto positions = l.getPositions();
                std::vector<float> scalars;
                for (auto p : positions) {
                    auto s = sampler->sample(p);
                    scalars.push_back(s);
                    if (s > max) max = s;
                }
                scalarsLineSetFrom2To1_.push_back(scalars);
            }
            for (auto& scalars : scalarsLineSetFrom2To1_)
                for (auto& s : scalars) s /= max;
        }

        LogInfo("Sample scalars in first field took " << millis(t0, Clock::now()) << " ms");
    };

    scalarSampler1_.onChange(getScalarsFrom1);
    lines1_
        .onChange(getScalarsFrom1);
    lines2_
        .onChange(getScalarsFrom1);

    auto getScalarsFrom2 = [&]() {
        auto t0 = Clock::now();
        if (!scalarSampler2_.hasData()) return;
        auto sampler = scalarSampler2_.getData();
        scalarsLineSet2_ = {};
        scalarsLineSetFrom1To2_ = {};
        if (lines2_.hasData() && !referenceStreamlines_) {
            const auto set2 = *lines2_.getData();
            float max = .0f;
            for (auto l : set2.getVector()) {
                auto positions = l.getPositions();
                std::vector<float> scalars;
                for (auto p : positions) {
                    auto s = sampler->sample(p);
                    scalars.push_back(s);
                    if (s > max) max = s;
                }
                scalarsLineSet2_.push_back(scalars);
            }
            for (auto& scalars : scalarsLineSet2_)
                for (auto& s : scalars) s /= max;
        }
        if (lines1_.hasData()) {
            const auto set1 = *lines1_.getData();
            float max = .0f;
            for (auto l : set1.getVector()) {
                auto positions = l.getPositions();
                std::vector<float> scalars;
                for (auto p : positions) {
                    auto s = sampler->sample(p);
                    scalars.push_back(s);
                    if (s > max) max = s;
                }
                scalarsLineSetFrom1To2_.push_back(scalars);
            }
            for (auto& scalars : scalarsLineSetFrom1To2_)
                for (auto& s : scalars) s /= max;
        }

        LogInfo("Sample scalars in second field took " << millis(t0, Clock::now()) << " ms");
    };

    scalarSampler2_.onChange(getScalarsFrom2);
    lines1_.onChange(getScalarsFrom2);
    lines2_.onChange(getScalarsFrom2);
}

void IntegralLineCompare::process() {
    const auto set1 = *lines1_.getData();
    const auto resultSet = std::make_shared<IntegralLineSet>(mat4(1));
    const auto colors = std::make_shared<std::vector<vec4>>();
    auto mesh = std::make_shared<MyLineMesh>();
    auto podMesh = std::make_shared<BasicMesh>();

    if (referenceStreamlines_) {

        // This is an alternative visualization.
        // It avoids error accumulation along streamlines.
        // Error accumulation is a problem when comparing two lines,
        // because noise at a single location in one flow field influences
        // the error against the other field for all following locations along the streamline.
        // Here we take only streamlines in the reference flow field.
        // At each point of a reference line, the other field is sampled and a local independent
        // error is computed.

        if (!velocitySampler_.hasData()) {
            LogWarn(
                "Need sampler of the other flow field to sample along the reference streamlines.");
            return;
        }

        if (deviationsMagnitude_.size() < set1.getVector().size()) {
            LogWarn("Please precompute.");
            return;
        }

        for (size_t i = 0; i < set1.getVector().size(); i++) {
            const auto& positions = set1.getVector()[i].getPositions();

            tubeGfx(
                mesh, positions,
                [&](int ii) {
                    // formula
                    float r = deviationsDirection_[i][ii];
                    if (compareMagnitude_) {
                        r /= (1.f + deviationsMagnitude_[i][ii]);
                    }
                    return glm::clamp(r * radiusScaling_.get(), minRadius_.get(), maxRadius_.get());
                },
                [&](int ii) {
                    float r = deviationsDirection_[i][ii];
                    if (compareMagnitude_) {
                        r /= (1.f + deviationsMagnitude_[i][ii]);
                    }
                    return colorMapping({scalarsLineSet1_[i][ii], 0, 1},
                                        {scalarsLineSetFrom1To2_[i][ii], 0, 1},
                                        {deviationsMagnitude_[i][ii], 0, 1}, {r, 0, 1});
                },
                [](int ii) { return vec3(0); });
        }
    } else if (lines2_.hasData()) {

        // Here starts the original visualization.

        const auto set2 = *lines2_.getData();

        auto distance = [](IntegralLine l1, IntegralLine l2) {
            float sum = .0f;
            for (int i = 0; i < 1; i++)
                sum += glm::distance(l1.getPositions()[i], l2.getPositions()[i]);
            return sum;
        };

        if (set1.size() != set2.size()) LogWarn("Comparing sets with different number of lines");

        std::vector<vec4> colormap;
        float colorstep = 1.f / set2.size();
        for (int i = 0; i < set2.size(); i++) {
            colormap.push_back(vec4(color::hsv2rgb(vec3(colorstep * i, 1.f, 1.f)), 1.f));
        }

        // Find pairs

        std::vector<LinePair> pairs;
        size_t numNoMatch = 0, numAlreadyMatched = 0, numPairs = 0;
        std::vector<size_t> taken;

        for (size_t li = 0; li < set1.size(); li++) {
            auto l = set1.getVector()[li];
            float minDist = std::numeric_limits<float>::infinity();
            size_t matchedIdx = 0;
            for (size_t i = 0; i < set2.size(); i++) {
                float d = distance(l, set2[i]);
                if (d < minDist) {
                    minDist = d;
                    matchedIdx = i;
                }
            }
            if (std::find(taken.begin(), taken.end(), matchedIdx) < taken.end()) {
                if (noTriples_) {
                    numAlreadyMatched++;
                    continue;
                }
            }
            if (minDist > matchTolerance_) {
                numNoMatch++;
                continue;
            }
            taken.push_back(matchedIdx);

            numPairs++;

            auto l_match = set2[matchedIdx];

            util::error(l, l_match);

            if (tubes_) {
                LinePair pair(l, l_match, scalarsLineSet1_[li], scalarsLineSetFrom1To2_[li],
                              scalarsLineSet2_[matchedIdx], scalarsLineSetFrom2To1_[matchedIdx]);
                pairs.push_back(pair);
            } else {
                resultSet->push_back(l, matchedIdx);
                resultSet->push_back(l_match, matchedIdx);
                colors->push_back(colormap[matchedIdx]);
                colors->push_back(colormap[matchedIdx]);
            }
        }

        LogInfo("Pairwise comparison: "
                << numNoMatch << " lines have no match, " << numAlreadyMatched
                << " lines found a match that was already taken, " << numPairs << " pairs found.");

        // Compute mean lines and property ranges

        std::vector<MeanLine> means;
        float minDeviation = std::numeric_limits<float>::infinity(), maxDeviation = 0.0f;
        float minRunlength = std::numeric_limits<float>::infinity(), maxRunlength = 0.0f;
        float minWallDistance = std::numeric_limits<float>::infinity(), maxWallDistance = 0.0f;

        for (LinePair& pair : pairs) {
            MeanLine mean = pair.meanLineUntil(splitThreshold_.get());

            for (const auto d : mean.deviations) {
                if (d < minDeviation) minDeviation = d;
                if (d > maxDeviation) maxDeviation = d;
            }

            auto l = mean.line.getLength();
            if (l < minRunlength) minRunlength = l;
            if (l > maxRunlength) maxRunlength = l;

            means.push_back(mean);
        }

        severityFilter_.setMaxValue(numPairs);

        std::sort(means.begin(), means.end(),
                  [&](MeanLine a, MeanLine b) { return severeness(a) > severeness(b); });

        // Create mesh to render

        for (size_t l_i = 0; l_i < severityFilter_.get(); l_i++) {
            MeanLine mean = means[l_i];

            if (mean.pair->bothExist()) {

                tubeGfx(
                    mesh, mean.line.getPositions(), [&](int i) { return mean.deviations[i]; },
                    [&](int i) {
                        return colorMapping(
                            {mean.pair->scalars1[i], 0, 1}, {mean.pair->scalars2[i], 0, 1},
                            {0, 0, 1}, {mean.deviations[i], minDeviation, maxDeviation},
                            {(float)mean.line.getLength(), minRunlength, maxRunlength});
                    },
                    [](int i) { return vec3(0); });

                BridgeFace bridge = mean.pair->bridgeFace();
                if (bridge.l1Normals.size() != mean.pair->l1.getPositions().size()) {
                    LogWarn("Bridge l1 normals " << bridge.l1Normals.size() << " instead of "
                                                 << mean.pair->l1.getPositions().size());
                }
                if (bridge.l2Normals.size() != mean.pair->l2.getPositions().size()) {
                    LogWarn("Bridge l2 normals " << bridge.l2Normals.size() << " instead of "
                                                 << mean.pair->l2.getPositions().size());
                }

                if (mean.isPairDiverged) {
                    podGfx(podMesh, vec3(mean.line.getPositions().back()), podSize_.get(),
                           podColor_.get());
                } else if (mean.isPairOfDifferentLength) {
                    podGfx(podMesh, vec3(mean.line.getPositions().back()), podSize_.get(),
                           earlyEndColor_.get());
                }

                if (divergedLines_.get() > 0.f) {
                    LinePair ends = mean.pair->lastPart(mean.endIndex);
                    auto color = colorMapping(
                        {mean.pair->scalars1.back(), 0, 1}, {mean.pair->scalars2.back(), 0, 1},
                        {0, 0, 1}, {mean.deviations.back(), minDeviation, maxDeviation},
                        {(float)mean.line.getLength(), minRunlength, maxRunlength});
                    tubeGfx(
                        mesh, ends.l1.getPositions(), [&](int i) { return divergedLines_.get(); },
                        [&](int i) { return color; },
                        [&](int i) { return /*bridge.l1Normals[mean.endIndex + i];*/ vec3(0); });

                    tubeGfx(
                        mesh, ends.l2.getPositions(), [&](int i) { return divergedLines_.get(); },
                        [&](int i) { return color; },
                        [&](int i) { return /*bridge.l2Normals[mean.endIndex + i];*/ vec3(0); });
                }
            }
        }
    }

    out_.setData(resultSet);
    colors_.setData(colors);
    tubeMesh_.setData(mesh);
    podMesh_.setData(podMesh);
}

float IntegralLineCompare::severeness(MeanLine mean) {

    // formula

    float sum = 0.f;
    for (auto d : mean.deviations) {
        sum += d;
    }

    switch (severityMetric_.get()) {
        case SeverenessMetric::Runlength:
            return 1.f / mean.line.getLength();
        case SeverenessMetric::RunlengthUntilDiverged:
            return mean.isPairDiverged ? 1.f / mean.line.getLength() : .0f;
        case SeverenessMetric::ThicknessSum:
            return sum;
        case SeverenessMetric::LengthAndThickness:
            return 1.f / mean.line.getLength() + sum;
        default:
            return 1.f / mean.line.getLength() + sum;
    }
}

vec4 IntegralLineCompare::colorMapping(BoundedFloat scalar1, BoundedFloat scalar2,
                                       BoundedFloat magdiff, BoundedFloat thickness,
                                       BoundedFloat runlength) {
    vec4 rgba = vec4(0);
    vec3 rgb = vec3(rgba);
    float a = rgba.a;
    if (enableScalar1_) {
        rgba = scalar1_.get().sample((scalar1.v - scalar1.min) / (scalar1.max - scalar1.min));

        // formula
        rgb += vec3(rgba) * rgba.a;
        a += rgba.a;
    }
    if (enableScalar2_) {
        rgba = scalar2_.get().sample((scalar2.v - scalar2.min) / (scalar2.max - scalar2.min));
        rgb += vec3(rgba) * rgba.a;
        a += rgba.a;
    }
    if (enableScalarDiff_) {
        rgba = scalarDiff_.get().sample((magdiff.v - magdiff.min) / (magdiff.max - magdiff.min));
        rgb += vec3(rgba) * rgba.a;
        a += rgba.a;
    }
    if (enableThickness_) {
        rgba = thickness_.get().sample((thickness.v - thickness.min) /
                                       (thickness.max - thickness.min));
        rgb += vec3(rgba) * rgba.a;
        a += rgba.a;
    }
    if (enableRunlength_) {
        rgba = runlength_.get().sample((runlength.v - runlength.min) /
                                       (runlength.max - runlength.min));
        rgb += vec3(rgba) * rgba.a;
        a += rgba.a;
    }
    return vec4(rgb, a);
}

void IntegralLineCompare::tubeGfx(std::shared_ptr<MyLineMesh> mesh, std::vector<dvec3> vertices,
                                  std::function<float(int)> radius, std::function<vec4(int)> color,
                                  std::function<vec3(int)> halftubeNormal) {
    if (vertices.size() <= 1) return;

    auto meshIndices = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::StripAdjacency);

    for (size_t i = 0; i < vertices.size(); i++) {
        if (i == 0) {
            vec3 dir = glm::normalize(vertices[i + 1] - vertices[i]);
            meshIndices->add(
                mesh->addVertex(vertices[0], radius(0), dir, color(0), halftubeNormal(0)));
            meshIndices->add(
                mesh->addVertex(vertices[0], radius(0), dir, color(0), halftubeNormal(0)));
        } else if (i > 0 && i < (vertices.size() - 1)) {
            vec3 dir = glm::normalize(vertices[i + 1] - vertices[i]);
            vec3 prevDir = glm::normalize(vertices[i] - vertices[i - 1]);
            vec3 normal = glm::normalize(dir + prevDir);
            meshIndices->add(
                mesh->addVertex(vertices[i], radius(i), normal, color(i), halftubeNormal(i)));
        } else {
            vec3 prevDir = glm::normalize(vertices[i] - vertices[i - 1]);
            meshIndices->add(
                mesh->addVertex(vertices.back(), radius(i), prevDir, color(i), halftubeNormal(i)));
            meshIndices->add(
                mesh->addVertex(vertices.back(), radius(i), prevDir, color(i), halftubeNormal(i)));
        }
    }
}

void IntegralLineCompare::podGfx(std::shared_ptr<BasicMesh> mesh, vec3 pos, float radius,
                                 vec4 color) {
    meshutil::sphere(pos, radius, color, mesh);
}

}  // namespace inviwo
