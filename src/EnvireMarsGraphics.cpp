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

typedef envire::core::GraphTraits::vertex_descriptor VertexDesc;

namespace mars
{
    namespace envire_mars_graphics
    {

        using std::string;
        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;

        EnvireMarsGraphics::EnvireMarsGraphics(lib_manager::LibManager *theManager) :
            lib_manager::LibInterface(theManager)
        {
            envireGraph = ControlCenter::envireGraph;
            graphTreeView = ControlCenter::graphTreeView;
            init();
        }

        EnvireMarsGraphics::EnvireMarsGraphics(lib_manager::LibManager *theManager,
                                               std::shared_ptr<envire::core::EnvireGraph> envireGraph,
                                               std::shared_ptr<envire::core::TreeView> graphTreeView) :
            lib_manager::LibInterface(theManager), envireGraph(envireGraph), graphTreeView(graphTreeView)
        {
            init();
        }

        void EnvireMarsGraphics::init(void)
        {
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
                std::string groupName, dataName;
                groupName = "EnvireMarsGraphcis";
                dataName = "debugTime";
                // initialize the dataBroker Package
                data_broker::DataPackage dbPackage;
                dbPackageMapping.writePackage(&dbPackage);
                dataBroker->pushData(groupName, dataName, dbPackage, NULL,
                                     data_broker::DATA_PACKAGE_READ_FLAG);
                // register as producer
                dataBroker->registerTimedProducer(this, groupName, dataName,
                                                  "_REALTIME_", 100);
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
                cfgVisRep = cfg->getOrCreateProperty("Simulator", "visual rep.",
                                                     (int)1, this);
                showGui = cfgVisRep.iValue & 1;
                showCollisions = cfgVisRep.iValue & 2;
                showAnchor = cfgVisRep.iValue & 4;
            }

            // todo: parse the graph and search for frames, etc.

            //GraphEventDispatcher::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::Link>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::Inertial>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::geometry::Box>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::geometry::Capsule>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::geometry::Cylinder>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::geometry::Mesh>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::geometry::Sphere>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::joints::Fixed>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::joints::Revolute>>::subscribe(envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::joints::Continuous>>::subscribe(envireGraph.get());
        }

        EnvireMarsGraphics::~EnvireMarsGraphics()
        {
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
                std::string groupName, dataName;
                groupName = "EnvireMarsGraphics";
                dataName = "debugTime";
                dataBroker->unregisterTimedProducer(this, groupName, dataName,
                                                    "_REALTIME_");
                libManager->releaseLibrary("data_broker");
            }
        }


        void EnvireMarsGraphics::preGraphicsUpdate(void)
        {
            long myTime;
            long timeDiff;
            if(dataBroker)
            {
                myTime = utils::getTime();
            }
            if(showGui)
            {
                for(auto &it: visualMap)
                {
                    if (it.second)
                    {
                        graphics->setDrawObjectPos(it.first, it.second->position);
                        graphics->setDrawObjectRot(it.first, it.second->rotation);
                    }
                }
            }
            if(dataBroker)
            {
                timeDiff = getTimeDiff(myTime);
                myTime += timeDiff;
                vizTime += timeDiff;
            }
            if (showCollisions)
            {
                for(auto &it: collisionMap)
                {
                    if (it.second)
                    {
                        graphics->setDrawObjectPos(it.first, it.second->position);
                        graphics->setDrawObjectRot(it.first, it.second->rotation);
                    }
                }
            }
            if(dataBroker)
            {
                timeDiff = getTimeDiff(myTime);
                myTime += timeDiff;
                colTime += timeDiff;
            }
            // todo: check if frames have to be updated
            // ideally: frames are always updated and anchors, visuals, and collisions are moved by frame
            // TODO: add menu showFrame
            for (auto &it : frameMap)
            {
                if (it.second)
                {
                    graphics->setDrawObjectPos(it.first, it.second->position);
                    graphics->setDrawObjectRot(it.first, it.second->rotation);
                }
            }
            if(dataBroker)
            {
                timeDiff = getTimeDiff(myTime);
                myTime += timeDiff;
                frameTime += timeDiff;
            }
            if(showAnchor)
            {
                for(auto &it: anchorMap)
                {
                    if (it.second)
                    {
                        graphics->setDrawObjectPos(it.first, it.second->position);
                    }
                }
            }
            if(dataBroker)
            {
                timeDiff = getTimeDiff(myTime);
                myTime += timeDiff;
                anchorTime += timeDiff;
            }
            if(++avgTimeCount == 100)
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
                    for(auto &it: visualMap)
                    {
                        graphics->setDrawObjectShow(it.first, showGui);
                    }
                    for (auto &it: collisionMap)
                    {
                        graphics->setDrawObjectShow(it.first, showCollisions);
                    }
                    for(auto &it: anchorMap)
                    {
                        graphics->setDrawObjectShow(it.first, showAnchor);
                    }
                }
                return;
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::Link>>& e)
        {
            envire::core::Transform t = envireGraph->getTransform(SIM_CENTER_FRAME_NAME, e.frame);
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
                unsigned long drawID = graphics->addDrawObject(nodeData, 0);
                graphics->setDrawObjectPos(drawID, t.transform.translation);
                graphics->setDrawObjectRot(drawID, t.transform.orientation);

                // the AbsolutePose is added by creating a new frame in the graph,
                // so the AbsolutePose Item already exists when a new visual item is added into the graph
                if (envireGraph->containsItems<envire::core::Item<interfaces::AbsolutePose>>(e.frame))
                {
                    // CAUTION: we assume that there is only one AbsolutePose in the frame
                    // so we get the first item
                    // TODO: add handling/warning if there is multiple AbsolutePose for some reason
                    envire::core::EnvireGraph::ItemIterator<envire::core::Item<interfaces::AbsolutePose>> it = envireGraph->getItem<envire::core::Item<interfaces::AbsolutePose>>(e.frame);
                    frameMap[drawID] = &(it->getData());
                }
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::Inertial>>& e)
        {
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::geometry::Box>>& e)
        {
            if (!graphics) {
                return;
            }

            envire::base_types::geometry::Box& geometry = e.item->getData();
            ConfigMap config = geometry.getFullConfigMap();

            config["origname"] = config["type"];
            config["filename"] = "PRIMITIVE";
            config["extend"]["x"] = config["size"]["x"];
            config["extend"]["y"] = config["size"]["y"];
            config["extend"]["z"] = config["size"]["z"];

            if (e.item->getTag() == "visual")
            {
                LOG_INFO("VISUAL Box: add: %s", e.frame.c_str());
                createVisual(config, e.frame);
            } else if (e.item->getTag() == "collision")
            {
                LOG_INFO("COLLISION Box: add: %s", e.frame.c_str());
                config["physicmode"] = config["type"];
                createCollision(config, e.frame);
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::geometry::Capsule>>& e)
        {
            if(!graphics)
            {
                return;
            }

            envire::base_types::geometry::Capsule& geometry = e.item->getData();
            ConfigMap config = geometry.getFullConfigMap();

            config["origname"] = config["type"];
            config["filename"] = "PRIMITIVE";
            config["extend"]["x"] = config["radius"];
            config["extend"]["y"] = config["length"];

            if (e.item->getTag() == "visual")
            {
                LOG_INFO("VISUAL Capsule: add: %s", e.frame.c_str());
                createVisual(config, e.frame);
            } else if (e.item->getTag() == "collision")
            {
                LOG_INFO("COLLISION Capsule: add: %s", e.frame.c_str());
                config["physicmode"] = config["type"];
                createCollision(config, e.frame);
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::geometry::Cylinder>>& e)
        {
            if(!graphics)
            {
                return;
            }

            envire::base_types::geometry::Cylinder& geometry = e.item->getData();
            ConfigMap config = geometry.getFullConfigMap();

            config["origname"] = config["type"];
            config["filename"] = "PRIMITIVE";
            config["extend"]["x"] = config["radius"];
            config["extend"]["y"] = config["length"];

            if (e.item->getTag() == "visual")
            {
                LOG_INFO("VISUAL Cylinder: add: %s", e.frame.c_str());
                createVisual(config, e.frame);
            } else if (e.item->getTag() == "collision")
            {
                LOG_INFO("COLLISION Cylinder: add: %s", e.frame.c_str());
                config["physicmode"] = config["type"];
                createCollision(config, e.frame);
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::geometry::Mesh>>& e)
        {
            if(!graphics)
            {
                return;
            }

            envire::base_types::geometry::Mesh& geometry = e.item->getData();
            ConfigMap config = geometry.getFullConfigMap();

            config["origname"] = "";
            config["visualscale"]["x"] = config["scale"]["x"];
            config["visualscale"]["y"] = config["scale"]["y"];
            config["visualscale"]["z"] = config["scale"]["z"];

            if (e.item->getTag() == "visual")
            {
                LOG_INFO("VISUAL Mesh: add: %s", e.frame.c_str());
                createVisual(config, e.frame);
            } else if (e.item->getTag() == "collision")
            {
                LOG_INFO("COLLISION Mesh: add: %s", e.frame.c_str());
                config["physicmode"] = config["type"];
                createCollision(config, e.frame);
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::geometry::Sphere>>& e)
        {
            if(!graphics)
            {
                return;
            }

            envire::base_types::geometry::Sphere& geometry = e.item->getData();
            ConfigMap config = geometry.getFullConfigMap();

            config["origname"] = config["type"];
            config["filename"] = "PRIMITIVE";
            config["extend"]["x"] = config["radius"];

            if (e.item->getTag() == "visual")
            {
                LOG_INFO("VISUAL Sphere: add: %s", e.frame.c_str());
                createVisual(config, e.frame);
            } else if (e.item->getTag() == "collision")
            {
                LOG_INFO("COLLISION Sphere: add: %s", e.frame.c_str());
                config["physicmode"] = config["type"];
                createCollision(config, e.frame);
            }
        }

        void EnvireMarsGraphics::createVisual(configmaps::ConfigMap &config, envire::core::FrameId frameId) {
            interfaces::NodeData nodeData;
            nodeData.fromConfigMap(&config, "");
            // TODO: do we want to set default material in smurf if we dont have one
            ConfigMap material;
            if (config.hasKey("material")) {
                material = config["material"];
            } else {
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

            unsigned long drawID = graphics->addDrawObject(nodeData, showGui);
            envire::core::Transform t = envireGraph->getTransform(SIM_CENTER_FRAME_NAME, frameId);
            utils::Vector p = t.transform.translation;
            utils::Quaternion q = t.transform.orientation;
            graphics->setDrawObjectPos(drawID, p);
            graphics->setDrawObjectRot(drawID, q);

            // the AbsolutePose is added by creating a new frame in the graph,
            // so the AbsolutePose Item already exists when a new visual item is added into the graph
            if (envireGraph->containsItems<envire::core::Item<interfaces::AbsolutePose>>(frameId))
            {
                // CAUTION: we assume that there is only one AbsolutePose in the frame
                // so we get the first item
                // TODO: add handling/warning if there is multiple AbsolutePose for some reason
                envire::core::EnvireGraph::ItemIterator<envire::core::Item<interfaces::AbsolutePose>> it = envireGraph->getItem<envire::core::Item<interfaces::AbsolutePose>>(frameId);
                visualMap[drawID] = &(it->getData());
            }
        }

        void EnvireMarsGraphics::createCollision(configmaps::ConfigMap &config, envire::core::FrameId frameId)
        {
            // map["movable"] = true;
            interfaces::NodeData nodeData;
            nodeData.fromConfigMap(&config, "");

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

            unsigned long drawID = graphics->addDrawObject(nodeData, showCollisions);
            envire::core::Transform t = envireGraph->getTransform(SIM_CENTER_FRAME_NAME, frameId);
            utils::Vector p = t.transform.translation;
            utils::Quaternion q = t.transform.orientation;
            graphics->setDrawObjectPos(drawID, p);
            graphics->setDrawObjectRot(drawID, q);

            // the AbsolutePose is added by creating a new frame in the graph,
            // so the AbsolutePose Item already exists when a new visual item is added into the graph
            if (envireGraph->containsItems<envire::core::Item<interfaces::AbsolutePose>>(frameId))
            {
                // CAUTION: we assume that there is only one AbsolutePose in the frame
                // so we get the first item
                // TODO: add handling/warning if there is multiple AbsolutePose for some reason
                envire::core::EnvireGraph::ItemIterator<envire::core::Item<interfaces::AbsolutePose>> it = envireGraph->getItem<envire::core::Item<interfaces::AbsolutePose>>(frameId);
                collisionMap[drawID] = &(it->getData());
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::joints::Fixed>>& e)
        {
            if(graphics)
            {
                envire::base_types::joints::Fixed &joint = e.item->getData();
                createJoint(joint.name, e.frame);
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::joints::Revolute>>& e)
        {
            if(graphics)
            {
                envire::base_types::joints::Revolute &joint = e.item->getData();
                createJoint(joint.name, e.frame);
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::joints::Continuous>>& e)
        {
            if(graphics)
            {
                envire::base_types::joints::Continuous &joint = e.item->getData();
                createJoint(joint.name, e.frame);
            }
        }

        void EnvireMarsGraphics::createJoint(const std::string &jointName, envire::core::FrameId frameId)
        {
            envire::core::Transform t = envireGraph->getTransform(SIM_CENTER_FRAME_NAME, frameId);
            Vector p = t.transform.translation;

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
            unsigned long drawID = graphics->addDrawObject(nodeData, showAnchor);
            graphics->setDrawObjectPos(drawID, p);
            graphics->setDrawObjectRot(drawID, t.transform.orientation);

            // the AbsolutePose is added by creating a new frame in the graph,
            // so the AbsolutePose Item already exists when a new visual item is added into the graph
            if (envireGraph->containsItems<envire::core::Item<interfaces::AbsolutePose>>(frameId))
            {
                // CAUTION: we assume that there is only one AbsolutePose in the frame
                // so we get the first item
                // TODO: add handling/warning if there is multiple AbsolutePose for some reason
                envire::core::EnvireGraph::ItemIterator<envire::core::Item<interfaces::AbsolutePose>> it = envireGraph->getItem<envire::core::Item<interfaces::AbsolutePose>>(frameId);
                anchorMap[drawID] = &(it->getData());
            }
        }

        void EnvireMarsGraphics::produceData(const data_broker::DataInfo &info,
                                             data_broker::DataPackage *dbPackage,
                                             int callbackParam)
        {
            dbPackageMapping.writePackage(dbPackage);
        }

    } // end of namespace envire_mars_graphics

} // end of namespace mars

DESTROY_LIB(mars::envire_mars_graphics::EnvireMarsGraphics);
CREATE_LIB(mars::envire_mars_graphics::EnvireMarsGraphics);
