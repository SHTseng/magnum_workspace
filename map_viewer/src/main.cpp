#include <Magnum/Platform/Sdl2Application.h>

namespace Magnum{

class MapViewer final: public Platform::Application {

    public:
        explicit MapViewer(const Arguments& arguments);

        virtual ~MapViewer() = default;

    private:
        void drawEvent() override
        {}
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
}
} // end of namespace Magnum

MAGNUM_APPLICATION_MAIN(Magnum::MapViewer)