#include <Corrade/Utility/Arguments.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Math/ConfigurationValue.h>
#include <Magnum/Math/DualComplex.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Primitives/Square.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/TranslationRotationScalingTransformation2D.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/Shaders/Flat.h>
#include <Magnum/Trade/MeshData2D.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include <assert.h>
#include <algorithm>
#include <string>
#include <vector>

namespace Magnum{

struct Map{
    unsigned int width;
    unsigned int height;
    std::vector<int> data;
};

typedef SceneGraph::Object<SceneGraph::TranslationRotationScalingTransformation2D> Object2D;
typedef SceneGraph::Scene<SceneGraph::TranslationRotationScalingTransformation2D> Scene2D;

class SquareDrawable : public SceneGraph::Drawable2D{
    public:
        explicit SquareDrawable(Object2D& object, GL::Mesh& mesh, Shaders::Flat2D& shader, const Color4& color, SceneGraph::DrawableGroup2D& drawables)
            : SceneGraph::Drawable2D{object, &drawables}, _mesh(mesh), _shader(shader), _color{color}
        {}

    private:
        void draw(const Matrix3& transformationMatrix, SceneGraph::Camera2D& camera) override{
            _shader.setTransformationProjectionMatrix(camera.projectionMatrix()*transformationMatrix)
                   .setColor(_color);
            _mesh.draw(_shader);
        }

        GL::Mesh& _mesh;
        Shaders::Flat2D& _shader;
        Color4 _color;
};

class MapViewer final: public Platform::Application {

    public:
        explicit MapViewer(const Arguments& arguments);

        virtual ~MapViewer() = default;

    private:

        static Map loadImageMap(const std::string& fileName);

        void drawEvent() override;

        Scene2D _scene;
        Object2D* _cameraObject;
        SceneGraph::Camera2D* _camera;
        SceneGraph::DrawableGroup2D _drawables;

        GL::Mesh _mesh{NoCreate};
        Shaders::Flat2D _shader{NoCreate};
};

MapViewer::MapViewer(const Arguments& arguments)
    : Platform::Application{arguments, NoCreate}{
    {
        const Vector2 dpiScaling = this->dpiScaling({});
        Configuration conf;
        conf.setTitle("Map Viewer")
            .setSize({1280, 720}, dpiScaling); //conf.size()
        GLConfiguration glConf;
        glConf.setSampleCount(dpiScaling.max() < 2.0f ? 8 : 2);
        if(!tryCreate(conf, glConf))
            create(conf, glConf.setSampleCount(0));
    }

    /* Configure camera */
    _cameraObject = new Object2D{&_scene};
    _camera = new SceneGraph::Camera2D{*_cameraObject};
    _camera->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
        .setProjectionMatrix(Matrix3::projection({20.0f, 20.0f}))
        .setViewport(GL::defaultFramebuffer.viewport().size());

    _shader = Shaders::Flat2D{};
    _mesh = MeshTools::compile(Primitives::squareSolid());

    auto box = new Object2D{&_scene};
    box->setScaling({0.5f, 0.5f});
    new SquareDrawable{*box, _mesh, _shader, {1.0f, 0.0f, 0.0f, 1.0f}, _drawables}; //0x2f83cc_rgbf

    Map m = loadImageMap("../resource/willowgarage_small.pgm");
    std::transform(m.data.begin(), m.data.end(), m.data.begin(), [](int num)->int{
        // std::cout << num << " ";
        return num < 254 ? 1 : 0;
    });
    // std::cout << std::endl;

    std::for_each(m.data.begin(), m.data.end(), [](int& num){ std::cout << num << " "; });
    std::cout << std::endl;
}

Map MapViewer::loadImageMap(const std::string& fileName){
    cv::Mat img = cv::imread(fileName);
    assert(!img.empty() && "Map image not found");
    if(!img.data){
        std::cout << "Error: the image wasn't correctly loaded." << std::endl;
        return {};
    }

    cv::Mat grayImg(img.size(), CV_8UC1);
    cv::cvtColor(img, grayImg, cv::COLOR_RGB2GRAY);
    // cv::imshow("Image from OpenCV", img);

    Map map;
    map.height = grayImg.rows;
    map.width = grayImg.cols;
    map.data.reserve(grayImg.rows * grayImg.cols);

    // image format:
    // ( col 1 ***** ... col n ***** )
    for(int i = 0; i < grayImg.rows; ++i){
        for(int j = 0; j < grayImg.cols; ++j){
            // unsigned char c = grayImg.at<uchar>(i, j);
            map.data.push_back(static_cast<int>(grayImg.at<uchar>(i, j)));
        }
    }
    // std::cout << std::endl;

    // cv::imshow("Image from OpenCV", grayImg);
    // cv::imwrite("gray_image.pgm", grayImg);

    return map;
}

void MapViewer::drawEvent(){
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);
    swapBuffers();
    redraw();
}
} // end of namespace Magnum

MAGNUM_APPLICATION_MAIN(Magnum::MapViewer)