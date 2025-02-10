/**
 * \file EnvireMarsGraphics.cpp
 * \author Malte Langosz
 *
 */

/* Conventions:
 *   - the includes should be defined in the header file
 *   - atomic variables shoud use snake_case
 *   - instances of classes should use camel_case
 *   - method names are camel_case
 *   - always use braces
 *   - braces start in new line
 *   - indent with four tabs
 */
#include "EnvireMarsGraphics.hpp"

//#include "PhysicsMapper.h"

#include <mars_utils/mathUtils.h>
#include <mars_utils/misc.h>
#include <lib_manager/LibInterface.hpp>
#include <lib_manager/LibManager.hpp>

#include <mars_interfaces/Logging.hpp>
#include <mars_interfaces/MARSDefs.h>
#include <mars_interfaces/terrainStruct.h>

typedef envire::core::GraphTraits::vertex_descriptor VertexDesc;
using namespace configmaps;
namespace mars
{
    namespace envire_mars_graphics
    {
        using namespace utils;
        using namespace interfaces;

        EnvireMarsGraphics::EnvireMarsGraphics(lib_manager::LibManager *theManager) :
            lib_manager::LibInterface{theManager},
            envireGraph{ControlCenter::envireGraph},
            graphTreeView{ControlCenter::graphTreeView}
        {
            init();
        }

        EnvireMarsGraphics::EnvireMarsGraphics(lib_manager::LibManager *theManager,
                                               std::shared_ptr<envire::core::EnvireGraph> envireGraph,
                                               std::shared_ptr<envire::core::TreeView> graphTreeView) :
            lib_manager::LibInterface{theManager},
            envireGraph{envireGraph},
            graphTreeView{graphTreeView}
        {
            init();
        }

        void EnvireMarsGraphics::init(void)
        {
            libManager->acquireLibrary("mars_core");
            showGui = true;
            showCollisions = false;
            showAnchor = false;
            vizTime = avgVizTime = 0;
            colTime = avgColTime = 0;
            frameTime = avgFrameTime = 0;
            anchorTime = avgAnchorTime = 0;
            avgTimeCount = 0;

            dataBroker = libManager->getLibraryAs<data_broker::DataBrokerInterface>("data_broker");
            dbPackageMapping.add("vizTime", &avgVizTime);
            dbPackageMapping.add("colTime", &avgColTime);
            dbPackageMapping.add("frameTime", &avgFrameTime);
            dbPackageMapping.add("anchorTime", &avgAnchorTime);
            if(dataBroker)
            {
                const auto groupName = std::string{"EnvireMarsGraphcis"};
                const auto dataName  = std::string{"debugTime"};
                // initialize the dataBroker Package
                data_broker::DataPackage dbPackage;
                dbPackageMapping.writePackage(&dbPackage);
                dataBroker->pushData(groupName, dataName, dbPackage, nullptr, data_broker::DATA_PACKAGE_READ_FLAG);

                // register as producer
                constexpr int update_period{100};
                dataBroker->registerTimedProducer(this, groupName, dataName, "_REALTIME_", update_period);
            }

            graphics = libManager->getLibraryAs<GraphicsManagerInterface>("mars_graphics");
            if(graphics)
            {
                graphics->addGraphicsUpdateInterface(this);
            }

            cfg = libManager->getLibraryAs<cfg_manager::CFGManagerInterface>("cfg_manager");
            if(cfg)
            {
                // TODO: add frame visualisation showFrames into the menu
                constexpr int visual_rep_value{1};
                cfgVisRep = cfg->getOrCreateProperty("Simulator", "visual rep.", visual_rep_value, this);
                showGui = cfgVisRep.iValue & 1;
                showCollisions = cfgVisRep.iValue & 2;
                showAnchor = cfgVisRep.iValue & 4;
            }

            // TODO: parse the graph and search for frames, etc.

            //GraphEventDispatcher::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::Link>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::Inertial>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Heightfield>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Plane>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Box>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Capsule>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Cylinder>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Mesh>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Sphere>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::joints::Fixed>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::joints::Revolute>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::types::joints::Continuous>>::subscribe(envireGraph.get());
        }

        EnvireMarsGraphics::~EnvireMarsGraphics()
        {
            // unsubscribe from envire graph
            GraphItemEventDispatcher<envire::core::Item<::envire::types::Link>>::unsubscribe();
            GraphItemEventDispatcher<envire::core::Item<::envire::types::Inertial>>::unsubscribe();
            GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Heightfield>>::unsubscribe();
            GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Plane>>::unsubscribe();
            GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Box>>::unsubscribe();
            GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Capsule>>::unsubscribe();
            GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Cylinder>>::unsubscribe();
            GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Mesh>>::unsubscribe();
            GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Sphere>>::unsubscribe();
            GraphItemEventDispatcher<envire::core::Item<::envire::types::joints::Fixed>>::unsubscribe();
            GraphItemEventDispatcher<envire::core::Item<::envire::types::joints::Revolute>>::unsubscribe();
            GraphItemEventDispatcher<envire::core::Item<::envire::types::joints::Continuous>>::unsubscribe();

            libManager->releaseLibrary("mars_core");
            if(graphics)
            {
                libManager->releaseLibrary("mars_graphics");
            }
            if(cfg)
            {
                libManager->releaseLibrary("cfg_manager");
            }
            if(dataBroker)
            {
                const auto groupName = std::string{"EnvireMarsGraphics"};
                const auto dataName  = std::string{"debugTime"};
                dataBroker->unregisterTimedProducer(this, groupName, dataName, "_REALTIME_");
                libManager->releaseLibrary("data_broker");
            }
        }


        void EnvireMarsGraphics::preGraphicsUpdate(void)
        {
            long long myTime;
            long long timeDiff;
            if(dataBroker)
            {
                myTime = utils::getTime();
            }
            if(showGui)
            {
                for(const auto &it: visualMap)
                {
                    if (it.second)
                    {
                        graphics->setDrawObjectPos(it.first, it.second->getPosition());
                        graphics->setDrawObjectRot(it.first, it.second->getRotation());
                    }
                }
            }
            if(dataBroker)
            {
                timeDiff = getTimeDiff(myTime);
                myTime += timeDiff;
                vizTime += static_cast<double>(timeDiff);
            }
            if (showCollisions)
            {
                for(const auto &it: collisionMap)
                {
                    if (it.second)
                    {
                        graphics->setDrawObjectPos(it.first, it.second->getPosition());
                        graphics->setDrawObjectRot(it.first, it.second->getRotation());
                    }
                }
            }
            if(dataBroker)
            {
                timeDiff = getTimeDiff(myTime);
                myTime += timeDiff;
                colTime += static_cast<double>(timeDiff);
            }
            // todo: check if frames have to be updated
            // ideally: frames are always updated and anchors, visuals, and collisions are moved by frame
            // TODO: add menu showFrame
            for (const auto &it : frameMap)
            {
                if (it.second)
                {
                    graphics->setDrawObjectPos(it.first, it.second->getPosition());
                    graphics->setDrawObjectRot(it.first, it.second->getRotation());
                }
            }
            if(dataBroker)
            {
                timeDiff = getTimeDiff(myTime);
                myTime += timeDiff;
                frameTime += static_cast<double>(timeDiff);
            }
            if(showAnchor)
            {
                for(const auto &it: anchorMap)
                {
                    if (it.second)
                    {
                        graphics->setDrawObjectPos(it.first, it.second->getPosition());
                    }
                }
            }
            if(dataBroker)
            {
                timeDiff = getTimeDiff(myTime);
                myTime += timeDiff;
                anchorTime += static_cast<double>(timeDiff);
            }
            constexpr int averaging_period_length{100};
            if(++avgTimeCount == averaging_period_length)
            {
                avgVizTime = vizTime/avgTimeCount;
                avgColTime = colTime/avgTimeCount;
                avgFrameTime = frameTime/avgTimeCount;
                avgAnchorTime = anchorTime/avgTimeCount;
                avgTimeCount = 0;
                vizTime = 0;
                colTime = 0;
                frameTime = 0;
                anchorTime = 0;
            }
        }

        void EnvireMarsGraphics::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property)
        {
            if(_property.paramId == cfgVisRep.paramId)
            {
                if(graphics)
                {
                    showGui = _property.iValue & 1;
                    showCollisions = _property.iValue & 2;
                    showAnchor = _property.iValue & 4;
                    for(const auto &it: visualMap)
                    {
                        graphics->setDrawObjectShow(it.first, showGui);
                    }
                    for (const auto &it: collisionMap)
                    {
                        graphics->setDrawObjectShow(it.first, showCollisions);
                    }
                    for(const auto &it: anchorMap)
                    {
                        graphics->setDrawObjectShow(it.first, showAnchor);
                    }
                }
                return;
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::Link>>& e)
        {
            const auto& transform = envireGraph->getTransform(SIM_CENTER_FRAME_NAME, e.frame);
            if(graphics)
            {
                // crate empty object to be able to use the frame debug option of mars_graphics
                ConfigMap config;
                config["origname"] = "empty";
                config["filename"] = "PRIMITIVE";
                config["name"] = e.frame;
                config["type"] = "empty";
                config["createFrame"] = true;
                interfaces::NodeData nodeData;
                nodeData.fromConfigMap(&config, "");
                nodeData.name += "_frame";
                const auto& drawID = graphics->addDrawObject(nodeData, 0);
                graphics->setDrawObjectPos(drawID, transform.transform.translation);
                graphics->setDrawObjectRot(drawID, transform.transform.orientation);

                // the AbsolutePose is added by creating a new frame in the graph,
                // so the AbsolutePose Item already exists when a new visual item is added into the graph
                if (envireGraph->containsItems<envire::core::Item<interfaces::AbsolutePose>>(e.frame))
                {
                    // CAUTION: we assume that there is only one AbsolutePose in the frame
                    // so we get the first item
                    // TODO: add handling/warning if there is multiple AbsolutePose for some reason
                    const auto& it = envireGraph->getItem<envire::core::Item<interfaces::AbsolutePose>>(e.frame);
                    frameMap[drawID] = &(it->getData());
                }
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::Inertial>>& e)
        {
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Heightfield>>& e)
        {
            if (!graphics)
            {
                return;
            }

            auto& geometry = e.item->getData();
            auto config = geometry.getFullConfigMap();

            if (e.item->getTag() == "visual")
            {
                LOG_INFO("VISUAL Heightfield: add: %s", e.frame.c_str());
                createVisual(config, e.frame);
            }
            else if (e.item->getTag() == "collision")
            {
                LOG_INFO("COLLISION Heightfield: add: %s", e.frame.c_str());
                config["physicmode"] = config["type"];
                createCollision(config, e.frame);
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Plane>>& e)
        {
            if (!graphics)
            {
                return;
            }

            auto& geometry = e.item->getData();
            auto config = geometry.getFullConfigMap();

            config["origname"] = config["type"];
            config["filename"] = "PRIMITIVE";
            config["extend"]["x"] = config["size"]["x"];
            config["extend"]["y"] = config["size"]["y"];

            if (e.item->getTag() == "visual")
            {
                LOG_INFO("VISUAL Box: add: %s", e.frame.c_str());
                createVisual(config, e.frame);
            }
            else if (e.item->getTag() == "collision")
            {
                LOG_INFO("COLLISION Box: add: %s", e.frame.c_str());
                config["physicmode"] = config["type"];
                createCollision(config, e.frame);
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Box>>& e)
        {
            if (!graphics)
            {
                return;
            }

            auto& geometry = e.item->getData();
            auto config = geometry.getFullConfigMap();

            config["origname"] = config["type"];
            config["filename"] = "PRIMITIVE";
            config["extend"]["x"] = config["size"]["x"];
            config["extend"]["y"] = config["size"]["y"];
            config["extend"]["z"] = config["size"]["z"];

            if (e.item->getTag() == "visual")
            {
                LOG_INFO("VISUAL Box: add: %s", e.frame.c_str());
                createVisual(config, e.frame);
            }
            else if (e.item->getTag() == "collision")
            {
                LOG_INFO("COLLISION Box: add: %s", e.frame.c_str());
                config["physicmode"] = config["type"];
                createCollision(config, e.frame);
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Capsule>>& e)
        {
            if(!graphics)
            {
                return;
            }

            auto& geometry = e.item->getData();
            auto config = geometry.getFullConfigMap();

            config["origname"] = config["type"];
            config["filename"] = "PRIMITIVE";
            config["extend"]["x"] = config["radius"];
            config["extend"]["y"] = config["length"];

            if (e.item->getTag() == "visual")
            {
                LOG_INFO("VISUAL Capsule: add: %s", e.frame.c_str());
                createVisual(config, e.frame);
            }
            else if (e.item->getTag() == "collision")
            {
                LOG_INFO("COLLISION Capsule: add: %s", e.frame.c_str());
                config["physicmode"] = config["type"];
                createCollision(config, e.frame);
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Cylinder>>& e)
        {
            if(!graphics)
            {
                return;
            }

            auto& geometry = e.item->getData();
            auto config = geometry.getFullConfigMap();

            config["origname"] = config["type"];
            config["filename"] = "PRIMITIVE";
            config["extend"]["x"] = config["radius"];
            config["extend"]["y"] = config["length"];

            if (e.item->getTag() == "visual")
            {
                LOG_INFO("VISUAL Cylinder: add: %s", e.frame.c_str());
                createVisual(config, e.frame);
            }
            else if (e.item->getTag() == "collision")
            {
                LOG_INFO("COLLISION Cylinder: add: %s", e.frame.c_str());
                config["physicmode"] = config["type"];
                createCollision(config, e.frame);
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Mesh>>& e)
        {
            if(!graphics)
            {
                return;
            }

            auto& geometry = e.item->getData();
            auto config = geometry.getFullConfigMap();

            config["origname"] = "";
            config["visualscale"]["x"] = config["scale"]["x"];
            config["visualscale"]["y"] = config["scale"]["y"];
            config["visualscale"]["z"] = config["scale"]["z"];

            if (e.item->getTag() == "visual")
            {
                LOG_INFO("VISUAL Mesh: add: %s", e.frame.c_str());
                createVisual(config, e.frame);
            }
            else if (e.item->getTag() == "collision")
            {
                LOG_INFO("COLLISION Mesh: add: %s", e.frame.c_str());
                config["physicmode"] = config["type"];
                createCollision(config, e.frame);
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Sphere>>& e)
        {
            if(!graphics)
            {
                return;
            }

            auto& geometry = e.item->getData();
            auto config = geometry.getFullConfigMap();

            config["origname"] = config["type"];
            config["filename"] = "PRIMITIVE";
            config["extend"]["x"] = config["radius"];

            if (e.item->getTag() == "visual")
            {
                LOG_INFO("VISUAL Sphere: add: %s", e.frame.c_str());
                createVisual(config, e.frame);
            }
            else if (e.item->getTag() == "collision")
            {
                LOG_INFO("COLLISION Sphere: add: %s", e.frame.c_str());
                config["physicmode"] = config["type"];
                createCollision(config, e.frame);
            }
        }

        void EnvireMarsGraphics::createVisual(configmaps::ConfigMap &config, envire::core::FrameId frameId)
        {
            interfaces::NodeData nodeData;
            nodeData.fromConfigMap(&config, "");

            // TODO: do we want to set default material in smurf if we dont have one
            ConfigMap material;
            if (config.hasKey("material"))
            {
                material = config["material"];
            }
            else
            {
                // TODO: Use named colors instead of magic numbers
                material["name"] = "visual";
                material["diffuseColor"]["a"] = 1.0;
                material["diffuseColor"]["r"] = 0.7;
                material["diffuseColor"]["g"] = 0.39;
                material["diffuseColor"]["b"] = 0.3;
                material["specularColor"]["a"] = 1.0;
                material["specularColor"]["r"] = 0.0;
                material["specularColor"]["g"] = 0.0;
                material["specularColor"]["b"] = 0.0;
                material["ambientColor"]["a"] = 1.0;
                material["ambientColor"]["r"] = 0.7;
                material["ambientColor"]["g"] = 0.59;
                material["ambientColor"]["b"] = 0.5;
            }
            nodeData.material.fromConfigMap(&material, "");

            // check if we have to load additional data:
            if(nodeData.terrain)
            {
                if(!nodeData.terrain->pixelData)
                {
                    if(config.hasKey("filePrefix"))
                    {
                        std::string path = config["filePrefix"];
                        nodeData.terrain->srcname = utils::pathJoin(path, nodeData.terrain->srcname);
                    }
                    LOG_INFO("Load heightmap pixelData... from %s", nodeData.terrain->srcname.c_str());
                    ControlCenter::loadCenter->loadHeightmap->readPixelData(nodeData.terrain);
                    if(!nodeData.terrain->pixelData)
                    {
                        LOG_ERROR("EnvireMarsGraphics::addNode: could not load image for terrain");
                    }
                }
            }

            nodeData.name += "_visual";
            const auto& drawID = graphics->addDrawObject(nodeData, showGui);
            const auto& transform = envireGraph->getTransform(SIM_CENTER_FRAME_NAME, frameId);
            const auto& p = transform.transform.translation;
            const auto& q = transform.transform.orientation;
            graphics->setDrawObjectPos(drawID, p);
            graphics->setDrawObjectRot(drawID, q);

            // the AbsolutePose is added by creating a new frame in the graph,
            // so the AbsolutePose Item already exists when a new visual item is added into the graph
            if (envireGraph->containsItems<envire::core::Item<interfaces::AbsolutePose>>(frameId))
            {
                // CAUTION: we assume that there is only one AbsolutePose in the frame
                // so we get the first item
                // TODO: add handling/warning if there is multiple AbsolutePose for some reason
                const auto& it = envireGraph->getItem<envire::core::Item<interfaces::AbsolutePose>>(frameId);
                visualMap[drawID] = &(it->getData());
            }
        }

        void EnvireMarsGraphics::createCollision(configmaps::ConfigMap &config, envire::core::FrameId frameId)
        {
            // map["movable"] = true;
            interfaces::NodeData nodeData;
            nodeData.fromConfigMap(&config, "");

            // TODO: Use named colors instead of magic numbers
            configmaps::ConfigMap material;
            material["name"] = "collision";
            material["diffuseColor"]["a"] = 1.0;
            material["diffuseColor"]["r"] = 0.7;
            material["diffuseColor"]["g"] = 0.39;
            material["diffuseColor"]["b"] = 0.3;
            material["specularColor"]["a"] = 1.0;
            material["specularColor"]["r"] = 0.0;
            material["specularColor"]["g"] = 0.0;
            material["specularColor"]["b"] = 0.0;
            material["ambientColor"]["a"] = 1.0;
            material["ambientColor"]["r"] = 0.7;
            material["ambientColor"]["g"] = 0.59;
            material["ambientColor"]["b"] = 0.5;
            material["shininess"] = 0.;
            material["transparency"] = 0.3;

            nodeData.material.fromConfigMap(&material, "");

            nodeData.name += "_collision";

            // check if we have to load additional data:
            if(nodeData.terrain)
            {
                if(!nodeData.terrain->pixelData)
                {
                    if(config.hasKey("filePrefix"))
                    {
                        std::string path = config["filePrefix"];
                        nodeData.terrain->srcname = utils::pathJoin(path, nodeData.terrain->srcname);
                    }
                    LOG_INFO("Load heightmap pixelData... from %s", nodeData.terrain->srcname.c_str());
                    ControlCenter::loadCenter->loadHeightmap->readPixelData(nodeData.terrain);
                    if(!nodeData.terrain->pixelData)
                    {
                        LOG_ERROR("EnvireMarsGraphics::addNode: could not load image for terrain");
                    }
                }
            }

            const auto& drawID = graphics->addDrawObject(nodeData, showCollisions);
            const auto& transform = envireGraph->getTransform(SIM_CENTER_FRAME_NAME, frameId);
            const auto& p = transform.transform.translation;
            const auto& q = transform.transform.orientation;
            graphics->setDrawObjectPos(drawID, p);
            graphics->setDrawObjectRot(drawID, q);

            // the AbsolutePose is added by creating a new frame in the graph,
            // so the AbsolutePose Item already exists when a new visual item is added into the graph
            if (envireGraph->containsItems<envire::core::Item<interfaces::AbsolutePose>>(frameId))
            {
                // CAUTION: we assume that there is only one AbsolutePose in the frame
                // so we get the first item
                // TODO: add handling/warning if there is multiple AbsolutePose for some reason
                const auto& it = envireGraph->getItem<envire::core::Item<interfaces::AbsolutePose>>(frameId);
                collisionMap[drawID] = &(it->getData());
            }
        }

        // TODO: Use template method
        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::joints::Fixed>>& e)
        {
            if(graphics)
            {
                const auto& joint = e.item->getData();
                createJoint(joint.getName(), e.frame);
            }
        }

        // TODO: Use template method
        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::joints::Revolute>>& e)
        {
            if(graphics)
            {
                const auto& joint = e.item->getData();
                createJoint(joint.getName(), e.frame);
            }
        }

        // TODO: Use template method
        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::joints::Continuous>>& e)
        {
            if(graphics)
            {
                const auto& joint = e.item->getData();
                createJoint(joint.getName(), e.frame);
            }
        }

        void EnvireMarsGraphics::createJoint(const std::string &jointName, envire::core::FrameId frameId)
        {
            const auto& transform = envireGraph->getTransform(SIM_CENTER_FRAME_NAME, frameId);
            const auto& p = transform.transform.translation;

            // TODO: Use named constants instead of magic numbers
            ConfigMap config, material;
            config["origname"] = "sphere";
            config["filename"] = "PRIMITIVE";
            config["name"] = jointName;
            config["type"] = "sphere";
            config["extend"]["x"] = 0.025;
            config["extend"]["y"] = 0.0;
            config["extend"]["z"] = 0.0;
            material["name"] << "anchor";
            material["diffuseColor"]["r"] << 0.3;
            material["diffuseColor"]["g"] << 0.3;
            material["diffuseColor"]["b"] << 0.7;
            material["diffuseColor"]["a"] << 1.0;
            material["shininess"] << 0.0;
            material["transparency"] << 0.5;

            interfaces::NodeData nodeData;
            nodeData.fromConfigMap(&config, "");
            nodeData.material.fromConfigMap(&material, "");
            nodeData.name += "_anchor";
            const auto& drawID = graphics->addDrawObject(nodeData, showAnchor);
            graphics->setDrawObjectPos(drawID, p);
            graphics->setDrawObjectRot(drawID, transform.transform.orientation);

            // the AbsolutePose is added by creating a new frame in the graph,
            // so the AbsolutePose Item already exists when a new visual item is added into the graph
            if (envireGraph->containsItems<envire::core::Item<interfaces::AbsolutePose>>(frameId))
            {
                // CAUTION: we assume that there is only one AbsolutePose in the frame
                // so we get the first item
                // TODO: add handling/warning if there is multiple AbsolutePose for some reason
                const auto& it = envireGraph->getItem<envire::core::Item<interfaces::AbsolutePose>>(frameId);
                anchorMap[drawID] = &(it->getData());
            }
        }

        void EnvireMarsGraphics::produceData(const data_broker::DataInfo &info, data_broker::DataPackage *dbPackage, int callbackParam)
        {
            dbPackageMapping.writePackage(dbPackage);
        }

    } // end of namespace envire_mars_graphics
} // end of namespace mars

DESTROY_LIB(mars::envire_mars_graphics::EnvireMarsGraphics);
CREATE_LIB(mars::envire_mars_graphics::EnvireMarsGraphics);
