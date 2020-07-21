# Name: PythonExample 

import inviwopy as ivw

import numpy

class PythonExample(ivw.Processor):
    def __init__(self, id, name):
        ivw.Processor.__init__(self, id, name)
        self.outport = ivw.data.MeshOutport("outport")
        self.addOutport(self.outport)

        self.slider = ivw.properties.IntProperty("slider", "slider", 0, 0, 100, 1)
        self.addProperty(self.slider)

        self.file = ivw.properties.FileProperty("filename", "Volume file", "", "volume")
        self.addProperty(self.file)

    @staticmethod
    def processorInfo():
        return ivw.ProcessorInfo(
            classIdentifier = "org.inviwo.PythonExample",
            displayName = "Python Example", 
            category = "Python",
            codeState = ivw.CodeState.Stable,
            tags = ivw.Tags.PY
   		)

    def getProcessorInfo(self):
        return PythonExample.processorInfo()

    def initializeResources(self):
        print("init")

    def process(self):
        print("process: ", self.slider.value)

        mesh = ivw.data.Mesh()
        mesh.addBuffer(ivw.data.BufferType.PositionAttrib, ivw.data.Buffer(
            numpy.array([0,0,0]).astype(numpy.float32)))
        mesh.addBuffer(ivw.data.BufferType.RadiiAttrib, ivw.data.Buffer(
            numpy.array([100]).astype(numpy.float32)))
        mesh.addBuffer(ivw.data.BufferType.ColorAttrib, ivw.data.Buffer(
            numpy.array([1,1,1,1]).astype(numpy.float32)))

        self.outport.setData(mesh)