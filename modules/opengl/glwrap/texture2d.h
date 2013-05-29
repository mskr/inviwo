#ifndef IVW_TEXTURE2D_H
#define IVW_TEXTURE2D_H

#include <modules/opengl/openglmoduledefine.h>
#include <inviwo/core/common/inviwo.h>
#include <modules/opengl/inviwoopengl.h>
#include <modules/opengl/ext/tgaload/tgaload.h>


namespace inviwo {

class IVW_MODULE_OPENGL_API Texture2D {

public:
    Texture2D(uvec2 dimensions, GLFormats::GLFormat glFormat, GLenum filtering);
    Texture2D(uvec2 dimensions, GLint format, GLint internalformat, GLenum dataType, GLenum filtering);
    virtual ~Texture2D();

    Texture2D* clone() const;
    GLuint getID() const { return id_; }

    GLuint getNChannels() const { return numChannels_; }
    GLuint getSizeInBytes() const { return byteSize_; }
    

    GLenum getFormat() const { return format_; }
    GLenum getDataType() const { return dataType_; }

    void bind() const;

    void upload(const void* data);
    
    /**
     * Download texture data to preallocated memory.
     * 
     * @param data (void *) Preallocated pointer that will contain texture data after function returns.
     * @return (void)
     */
    void download(void* data); 

    void unbind() const;

    int getWidth() const{ return dimensions_.x; }
    int getHeight() const{ return dimensions_.y; }
    void setWidth(int x) { dimensions_.x = x; }
    void setHeight(int y) { dimensions_.y = y; }
    void resize(uvec2 dimension);

protected:
    void setNChannels();
    void setSizeInBytes();

private:
    uvec2 dimensions_;
    GLenum format_;
    GLenum internalformat_;
    GLenum dataType_;
    GLenum filtering_;

    GLuint id_;

    GLuint byteSize_;
    GLuint numChannels_;
};

} // namespace

#endif // IVW_TEXTURE2D_H
