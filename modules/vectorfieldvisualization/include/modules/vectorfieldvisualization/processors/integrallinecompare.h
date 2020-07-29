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

#ifndef IVW_INTEGRALLINECOMPARE_H
#define IVW_INTEGRALLINECOMPARE_H

#include <modules/vectorfieldvisualization/vectorfieldvisualizationmoduledefine.h>

#include <inviwo/core/common/inviwo.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/ports/dataoutport.h>
#include <inviwo/core/ports/meshport.h>

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

namespace inviwo {

struct LinePair {
    IntegralLine &l1, l2;
    LinePair(IntegralLine& l1, IntegralLine& l2) : l1(l1), l2(l2) {}
    IntegralLine meanLine() {
        IntegralLine result;
        auto p1 = l1.getPositions();
        auto p2 = l2.getPositions();
        auto size = std::min(p1.size(), p2.size());
        for (size_t i = 0; i < size; i++) {
            result.getPositions().emplace_back(util::glm_convert<dvec3>((p1[i] + p2[i]) / 2.0));
        }
        return result;
    }
};

class IVW_MODULE_VECTORFIELDVISUALIZATION_API IntegralLineCompare : public Processor {
public:
    IntegralLineCompare();
    virtual ~IntegralLineCompare() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    IntegralLineSetInport lines1_;
    IntegralLineSetInport lines2_;
    IntegralLineSetOutport out_;
    MeshOutport tubeMesh_;
    DataOutport<std::vector<vec4>> colors_;

    FloatProperty matchTolerance_;
    BoolProperty noTriples_;
    BoolProperty tubes_;
};

}  // namespace inviwo

#endif  // IVW_INTEGRALLINECOMPARE_H
