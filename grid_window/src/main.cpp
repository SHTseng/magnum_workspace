/*
    This file is part of Magnum.

    Original authors — credit is appreciated but not required:

        2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019 —
            Vladimír Vondruš <mosra@centrum.cz>
        2018 — scturtle <scturtle@gmail.com>

    This is free and unencumbered software released into the public domain.

    Anyone is free to copy, modify, publish, use, compile, sell, or distribute
    this software, either in source code form or as a compiled binary, for any
    purpose, commercial or non-commercial, and by any means.

    In jurisdictions that recognize copyright laws, the author or authors of
    this software dedicate any and all copyright interest in the software to
    the public domain. We make this dedication for the benefit of the public
    at large and to the detriment of our heirs and successors. We intend this
    dedication to be an overt act of relinquishment in perpetuity of all
    present and future rights to this software under copyright law.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
    THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
    IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <Magnum/Image.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/PixelFormat.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/FunctionsBatch.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/MeshTools/Interleave.h>
#include <Magnum/MeshTools/CompressIndices.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Grid.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Object.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/Shaders/Flat.h>
#include <Magnum/Shaders/Phong.h>
#include <Magnum/Timeline.h>
#include <Magnum/Trade/MeshData3D.h>

#include "LineDrawable.hpp"

#include <iostream>
#include <memory>

namespace Magnum {

using Object3D = SceneGraph::Object<SceneGraph::MatrixTransformation3D>;
using Scene3D = SceneGraph::Scene<SceneGraph::MatrixTransformation3D>;
using Object3DPtr = std::unique_ptr<Object3D>;

using namespace Math::Literals;

constexpr int windowWidth = 1280;
constexpr int windowHeight = 720;
constexpr int gridCount = 20;

class AppView: public Platform::Application {
    public:
        explicit AppView(const Arguments& arguments);

    private:
        void objectPosUpdate(const Float duration);

        void addCube(const float length, const Matrix4& transformation, const Color4& color);
        // void addSephere(const int diameter, const Matrix4& transformation, const Color4& color);

        Float depthAt(const Vector2i& windowPosition);
        Vector3 unproject(const Vector2i& windowPosition, Float depth) const;

        void keyPressEvent(KeyEvent& event) override;
        void mousePressEvent(MouseEvent& event) override;
        void mouseMoveEvent(MouseMoveEvent& event) override;
        void mouseScrollEvent(MouseScrollEvent& event) override;
        void drawEvent() override;

        Shaders::Flat3D _flatShader{NoCreate};
        Shaders::Phong _phongShader{NoCreate};
        GL::Mesh _mesh{NoCreate}, _grid{NoCreate};

        Scene3D _scene;
        SceneGraph::DrawableGroup3D _drawables;
        Object3D* _cameraObject;
        Object3D* _lines;
        Object3D* _moveObject;
        SceneGraph::Camera3D* _camera;

        Timeline _timeline;

        Float _lastDepth;
        Vector2i _lastPosition{-1};
        Vector3 _rotationPoint, _translationPoint;
};

class FlatDrawable: public SceneGraph::Drawable3D {
    public:
        explicit FlatDrawable(Object3D& object, Shaders::Flat3D& shader, GL::Mesh& mesh, SceneGraph::DrawableGroup3D& drawables)
            : SceneGraph::Drawable3D{object, &drawables}
            , _shader(shader)
            , _mesh(mesh)
        {}

    private:
        void draw(const Matrix4& transformation, SceneGraph::Camera3D& camera) override {
            _shader.setColor(0x747474_rgbf)
                .setTransformationProjectionMatrix(camera.projectionMatrix()*transformation);
            _mesh.draw(_shader);
        }

        Shaders::Flat3D& _shader;
        GL::Mesh& _mesh;
};

class CubeDrawable: public SceneGraph::Drawable3D {
  public:
    explicit CubeDrawable(Object3D& object, Shaders::Phong& shader, SceneGraph::DrawableGroup3D* drawables, const Color4& color)
        : SceneGraph::Drawable3D{object, drawables}
        , _shader(shader)
        , _color(color)
    {
      _mesh = MeshTools::compile(Primitives::cubeSolid());
    }

  private:
    void draw(const Matrix4& transformationMatrix, SceneGraph::Camera3D& camera) override {
      using namespace Math::Literals;

      _shader.setDiffuseColor(_color)
             .setTransformationMatrix(transformationMatrix)
             .setNormalMatrix(transformationMatrix.rotationScaling())
             .setProjectionMatrix(camera.projectionMatrix());
      _mesh.draw(_shader);
    }

    GL::Mesh _mesh;
    Shaders::Phong& _shader;
    Color4 _color;
};

AppView::AppView(const Arguments& arguments): Platform::Application{arguments, NoCreate} {
    /* Try 8x MSAA, fall back to zero samples if not possible. Enable only 2x
       MSAA if we have enough DPI. */
    {
        const Vector2 dpiScaling = this->dpiScaling({});
        Configuration conf;
        conf.setTitle("3D Interactive Window")
            .setSize({windowWidth, windowHeight}, dpiScaling); //conf.size()
        GLConfiguration glConf;
        glConf.setSampleCount(dpiScaling.max() < 2.0f ? 8 : 2);
        if(!tryCreate(conf, glConf))
            create(conf, glConf.setSampleCount(0));
    }

    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);

    /* Shaders, renderer setup */
    _flatShader = Shaders::Flat3D{};
    _phongShader = Shaders::Phong{};
    _phongShader.setAmbientColor(0x111111_rgbf)
                .setSpecularColor(0x330000_rgbf)
                .setLightPosition({10.0f, 15.0f, 5.0f});

    /* Cubes */
    addCube(0.25f, Matrix4::translation({ 0.0f,  10.0f, 0.0f}), 0xa5c9ea_rgbf);
    addCube(0.1f, Matrix4::translation({0.0f,  0.0f, 0.0f})*Matrix4::rotationY(45.0_degf), 0xf7dc6f_rgbf);
    addCube(0.5f, Matrix4::translation({10.0f,  0.0f, 0.0f}), 0xb0e7cc_rgbf);
    addCube(0.5f, Matrix4::translation({0.0f,  0.0f, 10.0f}), 0xd7bde2_rgbf);

    // _lines = new Object3D{&_scene};
    // new LineDrawable{*_lines, _flatShader, &_drawables};

    /* Grid */
    _grid = MeshTools::compile(Primitives::grid3DWireframe({gridCount, gridCount}));
    auto grid = new Object3D{&_scene};
    // (*grid).translate(Vector3{0.5f, 0.5f, 0.0f}).rotateX(90.0_degf).scale(Vector3{8.0f});
    (*grid).translate(Vector3{0.0f, 0.0f, 0.0f}).rotateX(90.0_degf).scale(Vector3{8.0f});
    new FlatDrawable{*grid, _flatShader, _grid, _drawables};

    /* Set up the camera */
    _cameraObject = new Object3D{&_scene};
    (*_cameraObject).translate(Vector3::zAxis(5.0f))
                    .rotateX(-15.0_degf)
                    .rotateY(30.0_degf);
    _camera = new SceneGraph::Camera3D{*_cameraObject};
    _camera->setProjectionMatrix(Matrix4::perspectiveProjection(
        45.0_degf, Vector2{windowSize()}.aspectRatio(), 0.01f, 100.0f));

    /* Initialize initial depth to the value at scene center */
    _lastDepth = ((_camera->projectionMatrix()*_camera->cameraMatrix()).transformPoint({}).z() + 1.0f)*0.5f;

    _timeline.start();
}

void AppView::objectPosUpdate(const Float duration){
    Object3D* o = _scene.children().first();
    o->translate(Vector3::xAxis(1.0f*duration));
    // for(const auto& obj : _scene.children() ){
    //     auto val = obj.transformation().translation().dot();
    //     std::cout << val << " ";
    // }
    // std::cout << "\n";
}

void AppView::addCube(const float length, const Matrix4& transformation, const Color4& color){
    Object3D* o = new Object3D{&_scene};
    (*o).setTransformation(transformation).scale(Vector3(length));

    new CubeDrawable{*o, _phongShader, &_drawables, color};
}

Float AppView::depthAt(const Vector2i& windowPosition) {
    /* First scale the position from being relative to window size to being
       relative to framebuffer size as those two can be different on HiDPI
       systems */
    const Vector2i position = windowPosition*Vector2{framebufferSize()}/Vector2{windowSize()};
    const Vector2i fbPosition{position.x(), GL::defaultFramebuffer.viewport().sizeY() - position.y() - 1};

    GL::defaultFramebuffer.mapForRead(GL::DefaultFramebuffer::ReadAttachment::Front);
    Image2D data = GL::defaultFramebuffer.read(
        Range2Di::fromSize(fbPosition, Vector2i{1}).padded(Vector2i{2}),
        {GL::PixelFormat::DepthComponent, GL::PixelType::Float});

    return Math::min<Float>(Containers::arrayCast<const Float>(data.data()));
}

Vector3 AppView::unproject(const Vector2i& windowPosition, Float depth) const {
    /* We have to take window size, not framebuffer size, since the position is
       in window coordinates and the two can be different on HiDPI systems */
    const Vector2i viewSize = windowSize();
    const Vector2i viewPosition{windowPosition.x(), viewSize.y() - windowPosition.y() - 1};
    const Vector3 in{2*Vector2{viewPosition}/Vector2{viewSize} - Vector2{1.0f}, depth*2.0f - 1.0f};

    /*
        Use the following to get global coordinates instead of camera-relative:

        (_cameraObject->absoluteTransformationMatrix()*_camera->projectionMatrix().inverted()).transformPoint(in)
    */
    return _camera->projectionMatrix().inverted().transformPoint(in);
}

void AppView::keyPressEvent(KeyEvent& event) {
    /* Reset the transformation to the original view */
    if(event.key() == KeyEvent::Key::NumZero) {
        (*_cameraObject)
            .resetTransformation()
            .translate(Vector3::zAxis(5.0f))
            .rotateX(-15.0_degf)
            .rotateY(30.0_degf);
        redraw();
        return;

    /* Axis-aligned view */
    } else if(event.key() == KeyEvent::Key::NumOne ||
              event.key() == KeyEvent::Key::NumThree ||
              event.key() == KeyEvent::Key::NumSeven)
    {
        /* Start with current camera translation with the rotation inverted */
        const Vector3 viewTranslation = _cameraObject->transformation().rotationScaling().inverted()*_cameraObject->transformation().translation();

        /* Front/back */
        const Float multiplier = event.modifiers() & KeyEvent::Modifier::Ctrl ? -1.0f : 1.0f;

        Matrix4 transformation;
        if(event.key() == KeyEvent::Key::NumSeven) /* Top/bottom */
            transformation = Matrix4::rotationX(-90.0_degf*multiplier);
        else if(event.key() == KeyEvent::Key::NumOne) /* Front/back */
            transformation = Matrix4::rotationY(90.0_degf - 90.0_degf*multiplier);
        else if(event.key() == KeyEvent::Key::NumThree) /* Right/left */
            transformation = Matrix4::rotationY(90.0_degf*multiplier);
        else CORRADE_ASSERT_UNREACHABLE();

        _cameraObject->setTransformation(transformation*Matrix4::translation(viewTranslation));
        redraw();
    }
}

void AppView::mousePressEvent(MouseEvent& event) {
    /* Due to compatibility reasons, scroll is also reported as a press event,
       so filter that out. Could be removed once MouseEvent::Button::Wheel is
       gone from Magnum. */
    if(event.button() != MouseEvent::Button::Left &&
       event.button() != MouseEvent::Button::Middle)
        return;

    const Float currentDepth = depthAt(event.position());
    const Float depth = currentDepth == 1.0f ? _lastDepth : currentDepth;
    _translationPoint = unproject(event.position(), depth);
    /* Update the rotation point only if we're not zooming against infinite
       depth or if the original rotation point is not yet initialized */
    if(currentDepth != 1.0f || _rotationPoint.isZero()) {
        _rotationPoint = _translationPoint;
        _lastDepth = depth;
    }
}

void AppView::mouseMoveEvent(MouseMoveEvent& event) {
    if(_lastPosition == Vector2i{-1}) _lastPosition = event.position();
    const Vector2i delta = event.position() - _lastPosition;
    _lastPosition = event.position();

    if(!event.buttons()) return;

    /* Translate */
    if(event.modifiers() & MouseMoveEvent::Modifier::Shift) {
        const Vector3 p = unproject(event.position(), _lastDepth);
        _cameraObject->translateLocal(_translationPoint - p); /* is Z always 0? */
        _translationPoint = p;

    /* Rotate around rotation point */
    } else _cameraObject->transformLocal(
        Matrix4::translation(_rotationPoint)*
        Matrix4::rotationX(-0.01_radf*delta.y())*
        Matrix4::rotationY(-0.01_radf*delta.x())*
        Matrix4::translation(-_rotationPoint));

    redraw();
}

void AppView::mouseScrollEvent(MouseScrollEvent& event) {
    const Float currentDepth = depthAt(event.position());
    const Float depth = currentDepth == 1.0f ? _lastDepth : currentDepth;
    const Vector3 p = unproject(event.position(), depth);
    /* Update the rotation point only if we're not zooming against infinite
       depth or if the original rotation point is not yet initialized */
    if(currentDepth != 1.0f || _rotationPoint.isZero()) {
        _rotationPoint = p;
        _lastDepth = depth;
    }

    const Float direction = event.offset().y();
    if(!direction) return;

    /* Move towards/backwards the rotation point in cam coords */
    _cameraObject->translateLocal(_rotationPoint*direction*0.1f);

    event.setAccepted();
    redraw();
}

void AppView::drawEvent() {
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);

    _camera->draw(_drawables);

    // std::cout << _timeline.previousFrameDuration() << std::endl;
    objectPosUpdate(_timeline.previousFrameDuration());

    swapBuffers();
    _timeline.nextFrame();
    redraw(); // must?
}

}

MAGNUM_APPLICATION_MAIN(Magnum::AppView)