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

#include <smurf/Robot.hpp>

#include "EnvireMarsGraphics.hpp"

//#include "PhysicsMapper.h"

#include <mars/utils/mathUtils.h>
#include <mars/utils/misc.h>
#include <lib_manager/LibInterface.hpp>
#include <lib_manager/LibManager.hpp>

#include <mars_interfaces/Logging.hpp>

#define SIM_CENTER_FRAME_NAME "world"
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
            showGui = true;
            showCollisions = false;
            showAnchor = false;
            vizTime = avgVizTime = 0;
            frameTime = avgFrameTime = 0;
            anchorTime = avgAnchorTime = 0;
            avgTimeCount = 0;

            dataBroker = libManager->getLibraryAs<data_broker::DataBrokerInterface>("data_broker");
            dbPackageMapping.add("vizTime", &avgVizTime);
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

            //GraphEventDispatcher::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::smurf::Frame>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::smurf::Inertial>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::smurf::Joint>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::smurf::Visual>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::Link>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::Inertial>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::geometry::Box>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::geometry::Capsule>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::geometry::Cylinder>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::geometry::Mesh>>::subscribe(ControlCenter::envireGraph.get());
            GraphItemEventDispatcher<envire::core::Item<::envire::base_types::geometry::Sphere>>::subscribe(ControlCenter::envireGraph.get());
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
                    if (it.second.first)
                    {
                        utils::Vector p = it.second.first->position+it.second.first->rotation*it.second.second.translation();
                        graphics->setDrawObjectPos(it.first, p);
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
                avgFrameTime = frameTime/avgTimeCount;
                avgAnchorTime = anchorTime/avgTimeCount;
                avgTimeCount = 0;
                vizTime = 0;
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
                    for(auto &it: anchorMap)
                    {
                        graphics->setDrawObjectShow(it.first, showAnchor);
                    }
                }
                return;
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Frame>>& e)
        {
            envire::core::Transform t = ControlCenter::envireGraph->getTransform(SIM_CENTER_FRAME_NAME, e.frame);
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
                DynamicObjectItem *objectItem = NULL;
                try
                {
                    envire::core::EnvireGraph::ItemIterator<envire::core::Item<DynamicObjectItem>> it = ControlCenter::envireGraph->getItem<envire::core::Item<DynamicObjectItem>>(e.frame);
                    objectItem = &(it->getData());
                }
                catch (...)
                {
                    objectItem = NULL;
                }

                // the AbsolutePose is added by creating a new frame in the graph,
                // so the AbsolutePose Item already exists when a new visual item is added into the graph
                if (ControlCenter::envireGraph->containsItems<envire::core::Item<interfaces::AbsolutePose>>(e.frame))
                {
                    // CAUTION: we assume that there is only one AbsolutePose in the frame
                    // so we get the first item
                    // TODO: add handling/warning if there is multiple AbsolutePose for some reason
                    envire::core::EnvireGraph::ItemIterator<envire::core::Item<interfaces::AbsolutePose>> it = ControlCenter::envireGraph->getItem<envire::core::Item<interfaces::AbsolutePose>>(e.frame);
                    frameMap[drawID] = &(it->getData());
                }
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::Link>>& e)
        {
            envire::core::Transform t = ControlCenter::envireGraph->getTransform(SIM_CENTER_FRAME_NAME, e.frame);
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
                if (ControlCenter::envireGraph->containsItems<envire::core::Item<interfaces::AbsolutePose>>(e.frame))
                {
                    // CAUTION: we assume that there is only one AbsolutePose in the frame
                    // so we get the first item
                    // TODO: add handling/warning if there is multiple AbsolutePose for some reason
                    envire::core::EnvireGraph::ItemIterator<envire::core::Item<interfaces::AbsolutePose>> it = ControlCenter::envireGraph->getItem<envire::core::Item<interfaces::AbsolutePose>>(e.frame);
                    frameMap[drawID] = &(it->getData());
                }
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Inertial>>& e)
        {
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::Inertial>>& e)
        {
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Joint>>& e)
        {
            if(graphics)
            {
                urdf::JointSharedPtr joint = e.item->getData().getJointModel();

                // deprecated: config["anchorpos"] = "node2"; // always use the child_link as the anchor since joint and child_link are in the same frame

                envire::core::Transform t = ControlCenter::envireGraph->getTransform(SIM_CENTER_FRAME_NAME, e.frame);
                Vector p = t.transform.translation;
                const Eigen::Affine3d &transform = e.item->getData().getParentToJointOrigin();
                p += t.transform.orientation*transform.translation();

                ConfigMap config, material;
                config["origname"] = "sphere";
                config["filename"] = "PRIMITIVE";
                config["name"] = joint->name;
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
                if (ControlCenter::envireGraph->containsItems<envire::core::Item<interfaces::AbsolutePose>>(e.frame))
                {
                    // CAUTION: we assume that there is only one AbsolutePose in the frame
                    // so we get the first item
                    // TODO: add handling/warning if there is multiple AbsolutePose for some reason
                    envire::core::EnvireGraph::ItemIterator<envire::core::Item<interfaces::AbsolutePose>> it = ControlCenter::envireGraph->getItem<envire::core::Item<interfaces::AbsolutePose>>(e.frame);
                    anchorMap[drawID] = std::make_pair(&(it->getData()), transform);
                }

            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Visual>>& e)
        {
            // create node data from smurf::Visual
            // store the item with draw id in map
            if(!graphics)
            {
                return;
            }
            LOG_INFO("SMURF::Visual add: %s", e.frame.c_str());
            unsigned long drawID;

            bool create = false;
            smurf::Visual& visual = e.item->getData();
            ConfigMap node = visual.getConfigMap();

            // TODO: we do this since we use everywhere NodeData
            // we should get rid of NodeData and use ConfigMap
            switch(visual.geometry->type)
            {
            case smurf::Geometry::BOX:
            {
                node["origname"] = node["type"];
                node["filename"] = "PRIMITIVE";
                node["extend"]["x"] = node["size"]["x"];
                node["extend"]["y"] = node["size"]["y"];
                node["extend"]["z"] = node["size"]["z"];
                create = true;
                break;
            }
            case smurf::Geometry::SPHERE:
            {
                node["origname"] = node["type"];
                node["filename"] = "PRIMITIVE";
                node["extend"]["x"] = node["radius"];
                create = true;
                break;
            }
            case smurf::Geometry::CAPSULE:
            {
                node["origname"] = node["type"];
                node["filename"] = "PRIMITIVE";
                node["extend"]["x"] = node["radius"];
                node["extend"]["y"] = node["length"];
                create = true;
                break;
            }
            case smurf::Geometry::CYLINDER:
            {
                node["origname"] = node["type"];
                node["filename"] = "PRIMITIVE";
                node["extend"]["x"] = node["radius"];
                node["extend"]["y"] = node["length"];
                create = true;
                break;
            }
            case smurf::Geometry::MESH:
            {
                node["origname"] = "";
                node["visualscale"]["x"] = node["scale"]["x"];
                node["visualscale"]["y"] = node["scale"]["y"];
                node["visualscale"]["z"] = node["scale"]["z"];
                create = true;
                break;
            }
            default:
            {
                LOG_ERROR("smurf::Visual Added  unsupported type! %s", smurf::Geometry::toString(visual.geometry->type).c_str());
                return;
            }
            }
            if(create)
            {
                interfaces::NodeData nodeData;
                nodeData.fromConfigMap(&node, "");
                // TODO: do we want to set default material in smurf if we dont have one
                ConfigMap material;
                if (visual.material != nullptr) {
                    material = visual.material->getConfigMap();
                    nodeData.material.fromConfigMap(&material, "");
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
                drawID = graphics->addDrawObject(nodeData, showGui);

                // the AbsolutePose is added by creating a new frame in the graph,
                // so the AbsolutePose Item already exists when a new visual item is added into the graph
                if (ControlCenter::envireGraph->containsItems<envire::core::Item<interfaces::AbsolutePose>>(e.frame))
                {
                    // CAUTION: we assume that there is only one AbsolutePose in the frame
                    // so we get the first item
                    // TODO: add handling/warning if there is multiple AbsolutePose for some reason
                    envire::core::EnvireGraph::ItemIterator<envire::core::Item<interfaces::AbsolutePose>> it = ControlCenter::envireGraph->getItem<envire::core::Item<interfaces::AbsolutePose>>(e.frame);
                    visualMap[drawID] = &(it->getData());
                }

                // TODO: maybe we have to set the pose from AbsolutePose
                envire::core::Transform t = ControlCenter::envireGraph->getTransform(SIM_CENTER_FRAME_NAME, e.frame);
                utils::Vector p = t.transform.translation;
                utils::Quaternion q = t.transform.orientation;
                graphics->setDrawObjectPos(drawID, p);
                graphics->setDrawObjectRot(drawID, q);
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::geometry::Box>>& e)
        {
            if (e.item->getTag() != "visual")
                return;
            // create node data from smurf::Visual
            // store the item with draw id in map
            if(!graphics)
            {
                return;
            }
            LOG_INFO("SMURF::Visual add: %s", e.frame.c_str());
            unsigned long drawID;

            bool create = false;
            envire::base_types::geometry::Box& geometry = e.item->getData();
            ConfigMap node = geometry.getFullConfigMap();

            node["origname"] = node["type"];
            node["filename"] = "PRIMITIVE";
            node["extend"]["x"] = node["size"]["x"];
            node["extend"]["y"] = node["size"]["y"];
            node["extend"]["z"] = node["size"]["z"];
            create = true;

            // TODO: we do this since we use everywhere NodeData
            // we should get rid of NodeData and use ConfigMap
            if(create)
            {
                createVisual(node, e.frame);
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::geometry::Capsule>>& e)
        {
            if (e.item->getTag() != "visual")
                return;
            // create node data from smurf::Visual
            // store the item with draw id in map
            if(!graphics)
            {
                return;
            }
            LOG_INFO("SMURF::Visual add: %s", e.frame.c_str());
            unsigned long drawID;

            bool create = false;
            envire::base_types::geometry::Capsule& geometry = e.item->getData();
            ConfigMap node = geometry.getFullConfigMap();

            node["origname"] = node["type"];
            node["filename"] = "PRIMITIVE";
            node["extend"]["x"] = node["radius"];
            node["extend"]["y"] = node["length"];
            create = true;

            // TODO: we do this since we use everywhere NodeData
            // we should get rid of NodeData and use ConfigMap
            if(create)
            {
                createVisual(node, e.frame);
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::geometry::Cylinder>>& e)
        {
            if (e.item->getTag() != "visual")
                return;
            // create node data from smurf::Visual
            // store the item with draw id in map
            if(!graphics)
            {
                return;
            }
            LOG_INFO("SMURF::Visual add: %s", e.frame.c_str());
            unsigned long drawID;

            bool create = false;
            envire::base_types::geometry::Cylinder& geometry = e.item->getData();
            ConfigMap node = geometry.getFullConfigMap();

            node["origname"] = node["type"];
            node["filename"] = "PRIMITIVE";
            node["extend"]["x"] = node["radius"];
            node["extend"]["y"] = node["length"];
            create = true;

            // TODO: we do this since we use everywhere NodeData
            // we should get rid of NodeData and use ConfigMap
            if(create)
            {
                createVisual(node, e.frame);
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::geometry::Mesh>>& e)
        {
            if (e.item->getTag() != "visual")
                return;
            // create node data from smurf::Visual
            // store the item with draw id in map
            if(!graphics)
            {
                return;
            }
            LOG_INFO("SMURF::Visual add: %s", e.frame.c_str());
            unsigned long drawID;

            bool create = false;
            envire::base_types::geometry::Mesh& geometry = e.item->getData();
            ConfigMap node = geometry.getFullConfigMap();

            node["origname"] = "";
            node["visualscale"]["x"] = node["scale"]["x"];
            node["visualscale"]["y"] = node["scale"]["y"];
            node["visualscale"]["z"] = node["scale"]["z"];
            create = true;

            // TODO: we do this since we use everywhere NodeData
            // we should get rid of NodeData and use ConfigMap
            if(create)
            {
                createVisual(node, e.frame);
            }
        }

        void EnvireMarsGraphics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::geometry::Sphere>>& e)
        {
            if (e.item->getTag() != "visual")
                return;
            // create node data from smurf::Visual
            // store the item with draw id in map
            if(!graphics)
            {
                return;
            }
            LOG_INFO("SMURF::Visual add: %s", e.frame.c_str());
            unsigned long drawID;

            bool create = false;
            envire::base_types::geometry::Sphere& geometry = e.item->getData();
            ConfigMap node = geometry.getFullConfigMap();

            node["origname"] = node["type"];
            node["filename"] = "PRIMITIVE";
            node["extend"]["x"] = node["radius"];
            create = true;

            // TODO: we do this since we use everywhere NodeData
            // we should get rid of NodeData and use ConfigMap
            if(create)
            {
                createVisual(node, e.frame);
            }
        }

        void EnvireMarsGraphics::createVisual(configmaps::ConfigMap &node, envire::core::FrameId frameId) {
            interfaces::NodeData nodeData;
            nodeData.fromConfigMap(&node, "");
            // TODO: do we want to set default material in smurf if we dont have one
            ConfigMap material;
            if (node.hasKey("material")) {
                material = node["material"];
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
            envire::core::Transform t = ControlCenter::envireGraph->getTransform(SIM_CENTER_FRAME_NAME, frameId);
            utils::Vector p = t.transform.translation;
            utils::Quaternion q = t.transform.orientation;
            graphics->setDrawObjectPos(drawID, p);
            graphics->setDrawObjectRot(drawID, q);

            // the AbsolutePose is added by creating a new frame in the graph,
            // so the AbsolutePose Item already exists when a new visual item is added into the graph
            if (ControlCenter::envireGraph->containsItems<envire::core::Item<interfaces::AbsolutePose>>(frameId))
            {
                // CAUTION: we assume that there is only one AbsolutePose in the frame
                // so we get the first item
                // TODO: add handling/warning if there is multiple AbsolutePose for some reason
                envire::core::EnvireGraph::ItemIterator<envire::core::Item<interfaces::AbsolutePose>> it = ControlCenter::envireGraph->getItem<envire::core::Item<interfaces::AbsolutePose>>(frameId);
                visualMap[drawID] = &(it->getData());
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
