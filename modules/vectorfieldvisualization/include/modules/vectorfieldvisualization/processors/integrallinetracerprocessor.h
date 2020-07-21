/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2018-2020 Inviwo Foundation
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

#ifndef IVW_INTEGRALLINETRACERPROCESSOR_H
#define IVW_INTEGRALLINETRACERPROCESSOR_H

#include <modules/vectorfieldvisualization/vectorfieldvisualizationmoduledefine.h>
#include <inviwo/core/common/inviwo.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/processors/processortraits.h>

#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/properties/compositeproperty.h>
#include <inviwo/core/properties/boolcompositeproperty.h>
#include <inviwo/core/properties/boolproperty.h>
#include <inviwo/core/properties/optionproperty.h>

#include <inviwo/core/ports/datainport.h>
#include <inviwo/core/ports/dataoutport.h>
#include <inviwo/core/ports/imageport.h>

#include <inviwo/core/util/utilities.h>
#include <inviwo/core/util/foreach.h>
#include <modules/vectorfieldvisualization/algorithms/integrallineoperations.h>
#include <modules/vectorfieldvisualization/integrallinetracer.h>
#include <modules/vectorfieldvisualization/ports/seedpointsport.h>
#include <inviwo/core/util/colorbrewer.h>

namespace inviwo {

template <typename Tracer>
class IntegralLineTracerProcessor : public Processor {
public:
    IntegralLineTracerProcessor();
    virtual ~IntegralLineTracerProcessor();

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;

private:
    DataInport<typename Tracer::Sampler, 0> samplers_;
    SeedPointsInport<Tracer::Sampler::SpatialDimensions> seeds_;
    DataInport<typename Tracer::Sampler, 0> annotationSamplers_;

    IntegralLineSetOutport lines_;
    DataOutport<std::vector<vec4>> colors_;

    IntegralLineProperties properties_;

    CompositeProperty metaData_;
    BoolProperty calculateCurvature_;
    BoolProperty calculateTortuosity_;

    enum class CompareMode { None, MostDivergingLineBundle };

    TemplateOptionProperty<CompareMode> compareMode_;
};

template <typename Tracer>
IntegralLineTracerProcessor<Tracer>::IntegralLineTracerProcessor()
    : samplers_("sampler")
    , seeds_("seeds")
    , annotationSamplers_("annotationSamplers")
    , lines_("lines")
    , colors_("colors")
    , properties_("properties", "Properties")

    , metaData_("metaData", "Meta Data")
    , calculateCurvature_("calculateCurvature", "Calculate Curvature", false)
    , calculateTortuosity_("calculateTortuosity", "Calculate Tortuosity", false)
    , compareMode_("compareMode", "Compare Mode",
                   {{"none", "None", CompareMode::None},
                    {"mostDivergingLineBundle", "Most Diverging Line Bundle",
                     CompareMode::MostDivergingLineBundle}}) {
    addPort(samplers_);
    addPort(seeds_);
    addPort(annotationSamplers_);
    addPort(lines_);
    addPort(colors_);

    addProperty(properties_);
    addProperty(metaData_);
    metaData_.addProperty(calculateCurvature_);
    metaData_.addProperty(calculateTortuosity_);

    addProperty(compareMode_);

    properties_.normalizeSamples_.set(!Tracer::IsTimeDependent);
    properties_.normalizeSamples_.setCurrentStateAsDefault();

    annotationSamplers_.setOptional(true);
}

template <typename Tracer>
IntegralLineTracerProcessor<Tracer>::~IntegralLineTracerProcessor() {}

template <typename Tracer>
void IntegralLineTracerProcessor<Tracer>::process() {
    auto samplers = samplers_.getSourceVectorData();

    std::vector<std::string> dataSetNames;
    std::vector<Tracer> tracers;
    for (const auto &sampler : samplers) {
        dataSetNames.push_back(sampler.first->getProcessor()->getDisplayName());

        Tracer tracer(sampler.second, properties_);

        for (auto meta : annotationSamplers_.getSourceVectorData()) {
            auto key = meta.first->getProcessor()->getIdentifier();
            key = util::stripIdentifier(key);
            tracer.addMetaDataSampler(key, meta.second);
        }

        tracers.push_back(tracer);
    }

    const auto modelMatrix = samplers[0].second->getModelMatrix();
    const auto worldMatrix = samplers[0].second->getWorldMatrix();

    auto result = std::make_shared<IntegralLineSet>(modelMatrix, worldMatrix);

    auto colors = std::make_shared<std::vector<vec4>>();

    double maxDivg = 0;

    auto clamp = [](size_t x, size_t a, size_t b) { return x < a ? a : x > b ? b : x; };

    auto calcBundleDivg = [clamp](std::vector<IntegralLine> &bundle) {
        double divg = 0;
        for (const auto line_i : bundle) {
            const auto points_i = line_i.getPositions();
            for (const auto line_j : bundle) {
                const auto points_j = line_j.getPositions();

                const auto size = std::max(points_i.size(), points_j.size());

                for (size_t p = 0; p < size; p++) {
                    const auto p_i = points_i[clamp(p, 0, points_i.size() - 1)];
                    const auto p_j = points_j[clamp(p, 0, points_j.size() - 1)];
                    divg += glm::distance(p_i, p_j);
                }
            }
        }
        return divg;
    };

    size_t lineID = 0;

    std::mutex mutex;
    for (const auto &seeds : seeds_) {
        util::forEachParallel(*seeds, [&](const auto &seed, size_t i) {
            auto bundle = std::make_shared<IntegralLineSet>(modelMatrix, worldMatrix);

            for (size_t sampler_i = 0; sampler_i < tracers.size(); sampler_i++) {
                auto tracer = tracers[sampler_i];
                IntegralLine line = tracer.traceFrom(seed);
                line.setName(dataSetNames[sampler_i]);

                auto size = line.getPositions().size();
                if (size > 1) {
                    std::lock_guard<std::mutex> lock(mutex);
                    bundle->push_back(std::move(line), lineID++);
                }
            }

            double divg = calcBundleDivg(bundle->getVector());

            switch (compareMode_.get()) {
                case CompareMode::MostDivergingLineBundle: {
                    std::lock_guard<std::mutex> lock(mutex);
                    if (divg > maxDivg) {
                        result = bundle;
                        maxDivg = divg;

                        const auto colormap =
                            colorbrewer::getColormap(colorbrewer::Family::Accent, bundle->size());
                        colors->clear();
                        for (const auto c : colormap) colors->push_back(vec4(c));
                    }
                    lineID = 0;
                    break;
                }
                default: {
                    const auto colormap =
                        colorbrewer::getColormap(colorbrewer::Family::Accent, bundle->size());
                    for (size_t i = 0; i < bundle->size(); i++) {
                        std::lock_guard<std::mutex> lock(mutex);
                        result->push_back(bundle->at(i), lineID);
                        colors->push_back(colormap[i]);
                    }
                }
            }
        });
    }

    if (calculateCurvature_) {
        util::curvature(*result);
    }
    if (calculateTortuosity_) {
        util::tortuosity(*result);
    }

    lines_.setData(result);
    colors_.setData(colors);
}  // namespace inviwo

using StreamLines2D = IntegralLineTracerProcessor<StreamLine2DTracer>;
using StreamLines3D = IntegralLineTracerProcessor<StreamLine3DTracer>;
using PathLines3D = IntegralLineTracerProcessor<PathLine3DTracer>;

template <>
struct ProcessorTraits<StreamLines2D> {
    static ProcessorInfo getProcessorInfo() {
        return {
            "org.inviwo.StreamLines2D",  // Class identifier
            "Stream Lines 2D",           // Display name
            "Integral Line Tracer",      // Category
            CodeState::Stable,           // Code state
            Tags::CPU                    // Tags
        };
    }
};

template <>
struct ProcessorTraits<StreamLines3D> {
    static ProcessorInfo getProcessorInfo() {
        return {
            "org.inviwo.StreamLines3D",  // Class identifier
            "Stream Lines 3D",           // Display name
            "Integral Line Tracer",      // Category
            CodeState::Stable,           // Code state
            Tags::CPU                    // Tags
        };
    }
};

template <>
struct ProcessorTraits<PathLines3D> {
    static ProcessorInfo getProcessorInfo() {
        return {
            "org.inviwo.PathLines3D",  // Class identifier
            "Path Lines 3D",           // Display name
            "Integral Line Tracer",    // Category
            CodeState::Stable,         // Code state
            Tags::CPU                  // Tags
        };
    }
};

template <typename Tracer>
const ProcessorInfo IntegralLineTracerProcessor<Tracer>::getProcessorInfo() const {
    return ProcessorTraits<IntegralLineTracerProcessor<Tracer>>::getProcessorInfo();
}

}  // namespace inviwo

#endif  // IVW_INTEGRALLINETRACERPROCESSOR_H
