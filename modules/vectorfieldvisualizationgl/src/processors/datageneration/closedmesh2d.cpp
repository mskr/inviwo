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

#include <modules/vectorfieldvisualizationgl/processors/datageneration/closedmesh2d.h>

#include <inviwo/core/datastructures/geometry/typedmesh.h>

namespace inviwo {

const ProcessorInfo ClosedMesh2D::processorInfo_{
    "org.inviwo.ClosedMesh2D",  // Class identifier
    "Closed Mesh 2D",           // Display name
    "Data Creation",            // Category
    CodeState::Stable,          // Code state
    Tags::GL,                   // Tags
};

const ProcessorInfo ClosedMesh2D::getProcessorInfo() const { return processorInfo_; }

ClosedMesh2D::ClosedMesh2D() : Processor(), in_("inputMesh"), out_("outputMesh") {
    addPort(in_);
    addPort(out_);
}

ClosedMesh2D::~ClosedMesh2D() = default;

void ClosedMesh2D::initializeResources() {}

void ClosedMesh2D::process() {
    const auto mesh = in_.getData();

    const BufferRAM* meshNormals = nullptr;

    const auto buffers = mesh->getBuffers();
    for (const auto buf : buffers) {
        if (buf.first.type == BufferType::NormalAttrib) {
            meshNormals = buf.second->getRepresentation<BufferRAM>();
            break;
        }
    }

    std::vector<TriangleEdgeRefs> triEdges;

    for (auto& ib : mesh->getIndexBuffers()) {
        if (ib.first.dt == DrawType::Triangles) {
            meshutil::forEachTriangle(ib.first, *ib.second,
                                      [&triEdges](uint32_t i0, uint32_t i1, uint32_t i2) {
                                          triEdges.push_back({{i0, i1}, {i1, i2}, {i2, i0}});
                                      });
        }
    }

    const BufferRAM* verts = nullptr;
    for (const auto& buf : mesh->getBuffers()) {
        if (buf.first.type == BufferType::PositionAttrib) {
            verts = buf.second->getRepresentation<BufferRAM>();
            break;
        }
    }
    if (verts == nullptr) throw Exception("Empty mesh");

    auto equal = [](dvec3 a0, dvec3 a1, dvec3 b0, dvec3 b1) {
        auto e = .0001;
        return (glm::all(glm::epsilonEqual(a0, a1, e)) && glm::all(glm::epsilonEqual(b0, b1, e))) ||
               (glm::all(glm::epsilonEqual(a0, b1, e)) && glm::all(glm::epsilonEqual(b0, a1, e)));
    };

    std::vector<EdgeRef> outerEdges;

    for (size_t i = 0; i < triEdges.size(); i++) {
        bool sharedA = false, sharedB = false, sharedC = false;
        for (size_t j = 0; j < triEdges.size(); j++) {
            if (i != j) {
                if (triEdges[i].a == triEdges[j].a)
                    sharedA = true;
                else if (equal(verts->getAsDVec3(triEdges[i].a.a),
                               verts->getAsDVec3(triEdges[j].a.a),
                               verts->getAsDVec3(triEdges[i].a.b),
                               verts->getAsDVec3(triEdges[j].a.b)))
                    sharedA = true;
                if (triEdges[i].a == triEdges[j].b)
                    sharedA = true;
                else if (equal(verts->getAsDVec3(triEdges[i].a.a),
                               verts->getAsDVec3(triEdges[j].b.a),
                               verts->getAsDVec3(triEdges[i].a.b),
                               verts->getAsDVec3(triEdges[j].b.b)))
                    sharedA = true;
                if (triEdges[i].a == triEdges[j].c)
                    sharedA = true;
                else if (equal(verts->getAsDVec3(triEdges[i].a.a),
                               verts->getAsDVec3(triEdges[j].c.a),
                               verts->getAsDVec3(triEdges[i].a.b),
                               verts->getAsDVec3(triEdges[j].c.b)))
                    sharedA = true;

                if (triEdges[i].b == triEdges[j].a)
                    sharedB = true;
                else if (equal(verts->getAsDVec3(triEdges[i].b.a),
                               verts->getAsDVec3(triEdges[j].a.a),
                               verts->getAsDVec3(triEdges[i].b.b),
                               verts->getAsDVec3(triEdges[j].a.b)))
                    sharedB = true;
                if (triEdges[i].b == triEdges[j].b)
                    sharedB = true;
                else if (equal(verts->getAsDVec3(triEdges[i].b.a),
                               verts->getAsDVec3(triEdges[j].b.a),
                               verts->getAsDVec3(triEdges[i].b.b),
                               verts->getAsDVec3(triEdges[j].b.b)))
                    sharedB = true;
                if (triEdges[i].b == triEdges[j].c)
                    sharedB = true;
                else if (equal(verts->getAsDVec3(triEdges[i].b.a),
                               verts->getAsDVec3(triEdges[j].c.a),
                               verts->getAsDVec3(triEdges[i].b.b),
                               verts->getAsDVec3(triEdges[j].c.b)))
                    sharedB = true;

                if (triEdges[i].c == triEdges[j].a)
                    sharedC = true;
                else if (equal(verts->getAsDVec3(triEdges[i].c.a),
                               verts->getAsDVec3(triEdges[j].a.a),
                               verts->getAsDVec3(triEdges[i].c.b),
                               verts->getAsDVec3(triEdges[j].a.b)))
                    sharedC = true;
                if (triEdges[i].c == triEdges[j].b)
                    sharedC = true;
                else if (equal(verts->getAsDVec3(triEdges[i].c.a),
                               verts->getAsDVec3(triEdges[j].b.a),
                               verts->getAsDVec3(triEdges[i].c.b),
                               verts->getAsDVec3(triEdges[j].b.b)))
                    sharedC = true;
                if (triEdges[i].c == triEdges[j].c)
                    sharedC = true;
                else if (equal(verts->getAsDVec3(triEdges[i].c.a),
                               verts->getAsDVec3(triEdges[j].c.a),
                               verts->getAsDVec3(triEdges[i].c.b),
                               verts->getAsDVec3(triEdges[j].c.b)))
                    sharedC = true;

                if (sharedA && sharedB && sharedC) break;
            }
        }

        if (!sharedA) outerEdges.push_back(triEdges[i].a);
        if (!sharedB) outerEdges.push_back(triEdges[i].b);
        if (!sharedC) outerEdges.push_back(triEdges[i].c);
    }

    std::vector<vec3> vertices, normals;

    for (const auto& edge : outerEdges) {
        for (const auto& buf : mesh->getBuffers()) {
            if (buf.first.type == BufferType::PositionAttrib) {
                const auto v0 = (buf.second->getRepresentation<BufferRAM>()->getAsDVec3(edge.a));
                const auto v1 = (buf.second->getRepresentation<BufferRAM>()->getAsDVec3(edge.b));
                vertices.push_back((v0));
                vertices.push_back((v1));

                vec3 n;
                if (meshNormals != nullptr)
                    n = glm::normalize(glm::cross((v1 - v0), meshNormals->getAsDVec3(0)));
                else
                    n = glm::normalize(glm::cross((v1 - v0), dvec3(0, 0, 1)));

                normals.push_back(n);
                normals.push_back(n);

                break;
            }
        }
    }

    auto result = lineMeshFromVertexData(vertices, normals);
    result->setModelMatrix(mesh->getModelMatrix());
    result->setWorldMatrix(mesh->getWorldMatrix());

    out_.setData(result);
}

std::shared_ptr<Mesh> ClosedMesh2D::lineMeshFromVertexData(std::vector<vec3>& vertices,
                                                           std::vector<vec3>& normals) {

    using MyLineMesh = TypedMesh<buffertraits::PositionsBuffer, buffertraits::NormalBuffer,
                                 buffertraits::ColorsBuffer>;

    auto result = std::make_shared<MyLineMesh>();

    auto ib = result->addIndexBuffer(DrawType::Lines, ConnectivityType::None);

    for (size_t i = 0; i < vertices.size(); i++) {
        ib->add(result->addVertex(vertices[i], normals[i], vec4(1)));
    }

    return result;
}

}  // namespace inviwo