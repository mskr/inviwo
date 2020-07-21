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

#include <modules/vectorfieldvisualization/processors/2d/seedpointgenerator2d.h>
#include <inviwo/core/util/zip.h>
#include <modules/base/algorithm/randomutils.h>
#include <inviwo/core/interaction/events/mouseevent.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo SeedPointGenerator2D::processorInfo_{
    "org.inviwo.SeedPointGenerator2D",  // Class identifier
    "Seed Point Generator 2D",          // Display name
    "Data Creation",                    // Category
    CodeState::Stable,                  // Code state
    "CPU, Seed Points, Generator",      // Tags
};
const ProcessorInfo SeedPointGenerator2D::getProcessorInfo() const { return processorInfo_; }

SeedPointGenerator2D::SeedPointGenerator2D()
    : Processor()
    , seeds_("seeds")

    , generator_("generator", "Generator",
                 {{"random", "Random", Generator::Random},
                  {"haltonSequence", "Halton Sequence", Generator::HaltonSequence},
                  {"pick", "Pick", Generator::Pick}})

    , numPoints_("numPoints", "Number of points", 100, 1, 1000)
    , haltonXBase_("haltonXBase", "Base for x values", 2, 2, 32)
    , haltonYBase_("haltonYBase", "Base for y values", 3, 2, 32)

    , randomness_("randomness", "Randomness")
    , useSameSeed_("useSameSeed", "Use same seed", true)
    , seed_("seed", "Seed", 1, 0, 1000)
    , hoverEvents_(
          "hoverEvents", "Hover Events", [this](Event* e) { processPickEvent(e); },
          MouseButton::None, MouseState::Move)
    , clickEvents_(
          "clickEvents", "Click Events", [this](Event* e) { processPickEvent(e); },
          MouseButton::Left, MouseState::Press)
    , seedMin_("seedMin", "Seed Min", vec2(0), vec2(-1000), vec2(1000))
    , seedMax_("seedMax", "Seed Max", vec2(1), vec2(-1000), vec2(1000))
    , pickedSeed_("pickedSeed", "Seed", vec2(0), seedMin_.get(), seedMax_.get())
    , savedSeeds_("savedSeeds", "Saved Seeds",
                  std::make_unique<FloatVec2Property>("point", "Point", vec2(0), seedMin_.get(),
                                                      seedMax_.get()),
                  100)
    , rd_()
    , mt_(rd_())

{
    addPort(seeds_);

    addProperty(generator_);
    addProperty(numPoints_);
    addProperty(haltonXBase_);
    addProperty(haltonYBase_);
    addProperty(randomness_);
    randomness_.addProperty(useSameSeed_);
    randomness_.addProperty(seed_);
    addProperties(hoverEvents_, clickEvents_, seedMin_, seedMax_, pickedSeed_, savedSeeds_);

    useSameSeed_.onChange([&]() { seed_.setVisible(useSameSeed_.get()); });

    auto typeOnChange = [&]() {
        haltonXBase_.setVisible(generator_.getSelectedValue() == Generator::HaltonSequence);
        haltonYBase_.setVisible(generator_.getSelectedValue() == Generator::HaltonSequence);

        randomness_.setVisible(generator_.getSelectedValue() == Generator::Random);

        haltonXBase_.setVisible(generator_.getSelectedValue() != Generator::Pick);
        haltonYBase_.setVisible(generator_.getSelectedValue() != Generator::Pick);
        randomness_.setVisible(generator_.getSelectedValue() != Generator::Pick);
        numPoints_.setVisible(generator_.getSelectedValue() != Generator::Pick);
        useSameSeed_.setVisible(generator_.getSelectedValue() != Generator::Pick);

        hoverEvents_.setVisible(generator_.getSelectedValue() == Generator::Pick);
        clickEvents_.setVisible(generator_.getSelectedValue() == Generator::Pick);
        seedMin_.setVisible(generator_.getSelectedValue() == Generator::Pick);
        seedMax_.setVisible(generator_.getSelectedValue() == Generator::Pick);
        pickedSeed_.setVisible(generator_.getSelectedValue() == Generator::Pick);
    };

    generator_.onChange(typeOnChange);

    seedMin_.onChange([&]() { pickedSeed_.setMinValue(seedMin_.get()); });
    seedMax_.onChange([&]() { pickedSeed_.setMaxValue(seedMax_.get()); });
}

void SeedPointGenerator2D::process() {
    auto seeds = std::make_shared<std::vector<vec2>>();

    switch (generator_.get()) {
        case Generator::Random: {
            seeds->reserve(numPoints_.get());
            std::uniform_real_distribution<float> dis(0, 1);
            seeds->resize(numPoints_.get());
            util::randomSequence<float>(reinterpret_cast<float*>(seeds->data()),
                                        numPoints_.get() * 2, mt_, dis);
            break;
        }
        case Generator::HaltonSequence: {
            seeds->reserve(numPoints_.get());
            auto x = util::haltonSequence<float>(haltonXBase_.get(), numPoints_);
            auto y = util::haltonSequence<float>(haltonYBase_.get(), numPoints_);
            for (auto&& it : util::zip(x, y)) {
                seeds->emplace_back(get<0>(it), get<1>(it));
            }
            break;
        }

        case Generator::Pick: {
            for (const auto s : savedSeeds_) {
                if (auto seed = dynamic_cast<FloatVec2Property*>(s)) {
                    seeds->push_back(seed->get());
                }
            }
            seeds->push_back(pickedSeed_.get());
        }

        default:
            break;
    }

    seeds_.setData(seeds);
}

void SeedPointGenerator2D::processPickEvent(Event* e) {
    static int count = 0;

    const auto mouseEvent = static_cast<MouseEvent*>(e);
    const auto mousePos = vec2(mouseEvent->posNormalized());

    auto val = seedMin_.get() + mousePos * (seedMax_.get() - seedMin_.get());

    pickedSeed_.set(val);

    if (mouseEvent->button() == MouseButton::Left) {
        savedSeeds_.addProperty(new FloatVec2Property("point" + std::to_string(count++),
                                                      "Point" + std::to_string(count), val,
                                                      seedMin_.get(), seedMax_.get()),
                                true);
    }

    process();
}

}  // namespace inviwo
