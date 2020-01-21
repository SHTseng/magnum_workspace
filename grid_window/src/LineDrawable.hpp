#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Primitives/Line.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/Shaders/Flat.h>

using namespace Magnum;
using Object3D = SceneGraph::Object<SceneGraph::MatrixTransformation3D>;

class LineDrawable : public Magnum::SceneGraph::Drawable3D
{
    public:
        explicit LineDrawable(Object3D& object, Shaders::Flat3D& shader, SceneGraph::DrawableGroup3D* drawables,
            Vector3&& start, Vector3&& end)
            : SceneGraph::Drawable3D{object, drawables}
            , _shader(shader){
            // _mesh = MeshTools::compile(Primitives::line3D({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}));
            _mesh = MeshTools::compile(Primitives::line3D(start, end));
        }

        virtual ~LineDrawable() = default;

    private:
        void draw(const Matrix4& transformation, SceneGraph::Camera3D& camera) override{
            _shader.setColor({1.0f, 0.0f, 0.0f}) // 0x100000_rgbf
                .setTransformationProjectionMatrix(camera.projectionMatrix()*transformation);;

            _mesh.draw(_shader);
        }

        GL::Mesh _mesh;
        Shaders::Flat3D& _shader;
};