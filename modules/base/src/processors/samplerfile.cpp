/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2017-2020 Inviwo Foundation
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

#include <modules/base/processors/samplerfile.h>
#include <inviwo/core/util/imagesampler.h>
#include <inviwo/core/datastructures/image/imageram.h>

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

#define R double

#define Index size_t

#include <limits>

#define INF std::numeric_limits<R>::infinity()

struct Node {
    R x, y, z;
    bool operator==(Node other) {
        return this->x == other.x && this->y == other.y && this->z == other.z;
    }
};

#include <vector>

enum QuantityType { SCALAR, VEC2, VEC3, MAX_COMPONENTS };

struct QuantityValue {
    R components[MAX_COMPONENTS];
    QuantityValue(R a = 0, R b = 0, R c = 0) {
        for (Index i = 0; i < MAX_COMPONENTS; i++)
            components[i] = (i == 0) ? a : (i == 1) ? b : (i == 2) ? c : 0;
    }
    QuantityValue operator*(R weight) const {
        QuantityValue result;
        for (Index i = 0; i < MAX_COMPONENTS; i++)
            result.components[i] = this->components[i] * weight;
        return result;
    }
    void operator+=(QuantityValue other) {
        for (Index i = 0; i < MAX_COMPONENTS; i++) this->components[i] += other.components[i];
    }
    R operator[](size_t i) const { return components[i]; }
};

#include <map>
#include <string>

struct SelectedQuantity {
    std::string name;
    QuantityType type;
    Index valueRefs[MAX_COMPONENTS];
    std::vector<QuantityValue> values;
    SelectedQuantity(std::map<std::string, Index> quantities, std::string quantity);
    size_t numComponents() const { return type == SCALAR ? 1 : type == VEC2 ? 2 : 3; }
};

SelectedQuantity::SelectedQuantity(std::map<std::string, Index> quantities, std::string quantity) {
    this->name = quantity;

    if (quantity[0] != '[') {
        this->type = SCALAR;
        try {
            this->valueRefs[0] = quantities.at(quantity);
        } catch (const std::out_of_range) {
            printf("Selected quantity not found.\n");
            exit(1);
        }
    } else {
        const auto components = strings(quantity.substr(1, quantity.length() - 2), ',');

        if (components.size() == 2) {
            this->type = VEC2;
        } else if (components.size() == 3) {
            this->type = VEC3;
        } else {
            printf("Only up to %u-component vectors are supported.\n", MAX_COMPONENTS);
            exit(1);
        }

        for (Index i = 0; i < components.size(); i++) {
            try {
                this->valueRefs[i] = quantities.at(components[i]);
            } catch (const std::out_of_range) {
                printf("One of the components of the selected quantity was not found.\n");
                exit(1);
            }
        }
    }
}

struct AABB {
  Node min, max;
  AABB(std::vector<Node> nodes);
  Node getCenter() const;
};

AABB::AABB(std::vector<Node> nodes) : min{ INF, INF, INF }, max{ -INF, -INF, -INF } {
  for (const auto node : nodes) {
    if (node.x < min.x) min.x = node.x;
    if (node.y < min.y) min.y = node.y;
    if (node.z < min.z) min.z = node.z;
    if (node.x > max.x) max.x = node.x;
    if (node.y > max.y) max.y = node.y;
    if (node.z > max.z) max.z = node.z;
  }
}

Node AABB::getCenter() const {
  return {
    min.x + (max.x - min.x) / 2.0,
    min.y + (max.y - min.y) / 2.0,
    min.z + (max.z - min.z) / 2.0
  };
}

R distance(Node a, Node b) {
  Node ab = { b.x - a.x, b.y - a.y, b.z - a.z };
  return std::sqrt(ab.x * ab.x + ab.y * ab.y + ab.z * ab.z);
}

std::shared_ptr<inviwo::Image> read(const std::string file, const std::string quantity, const inviwo::size2_t resolution) {

    std::ifstream in(file);

    const auto fileName = strings(strings(strings(file, '/').back(), '\\').back(), '.')[0];

    printf("%s\n", fileName.c_str());

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

    // Get input data:

    SelectedQuantity selectedQuantity(quantities, quantity);

    std::vector<Node> nodes;

    for (std::string data; std::getline(in, data);) {

        std::vector<R> numbers;

        for (const auto ascii : strings(data)) {
            numbers.push_back(atof(ascii.c_str()));
        }

        nodes.push_back({numbers[1], numbers[2], sim2D ? 0.0 : numbers[3]});

        if (selectedQuantity.type == SCALAR) {
            selectedQuantity.values.push_back({numbers[selectedQuantity.valueRefs[0]]});
        }
        if (selectedQuantity.type == VEC2) {
            selectedQuantity.values.push_back(
                {numbers[selectedQuantity.valueRefs[0]], numbers[selectedQuantity.valueRefs[1]]});
        }
        if (selectedQuantity.type == VEC3) {
            selectedQuantity.values.push_back({numbers[selectedQuantity.valueRefs[0]],
                                               numbers[selectedQuantity.valueRefs[1]],
                                               numbers[selectedQuantity.valueRefs[2]]});
        }
    }

    printf("%zu nodes found.\n", nodes.size());

    assert(nodes.size() == selectedQuantity.values.size());

    const AABB aabb(nodes);

    const size_t W = 200, H = 200;
    
    using namespace inviwo;

    auto layer = std::make_shared<Layer>(
        resolution, DataVec2Float32::get(), LayerType::Color,
        SwizzleMask{ImageChannel::Red, ImageChannel::Green, ImageChannel::Zero, ImageChannel::One});
    auto image = std::make_shared<Image>(layer);

    for (size_t i = 0; i < W; i++) {
        for (size_t j = 0; j < H; j++) {
            R x = (aabb.max.x - aabb.min.x) * ((R)i / W);
            R y = (aabb.max.y - aabb.min.y) * ((R)j / H);
            size_t nearestNeighbor = 0;
            for (size_t k = 0; k < nodes.size(); k++) {
                if (distance(nodes[k], {x, y, 0}) < distance(nodes[nearestNeighbor], {x, y, 0})) {
                    nearestNeighbor = k;
                }
            }
            
        }
    }
}
}  // namespace reading

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo SamplerFile::processorInfo_{
    "org.inviwo.SamplerFile",  // Class identifier
    "Sampler File",            // Display name
    "Spatial Sampler",         // Category
    CodeState::Experimental,   // Code state
    Tags::None,                // Tags
};
const ProcessorInfo SamplerFile::getProcessorInfo() const { return processorInfo_; }

SamplerFile::SamplerFile()
    : Processor()
    , sampler_("sampler")
    , file_("filename", "File")
    , quantity_("quantity", "Quantity", "[x-velocity, y-velocity, z-velocity]")
    , readButton_("readButton", "Read") {
    addPort(sampler_);
    addProperties(file_, quantity_, readButton_);

    readButton_.onChange([this]() { reading::read(this->file_.get(), this->quantity_.get(), size2_t(200, 200)); });
}

void SamplerFile::process() {
    LogInfo(file_.get());
    // sampler_.setData(sampler);
}

}  // namespace inviwo
