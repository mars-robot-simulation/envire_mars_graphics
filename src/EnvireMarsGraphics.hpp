/**
 * \file EnvireMarsGraphics.hpp
 * \author Malte Langosz
 * \brief Plugin class to debug visualizations for physics representation based on envire items
 *
 */

#pragma once
#include <mars/utils/Vector.h>


#include <mars/cfg_manager/CFGManagerInterface.h>
#include <envire_core/items/Item.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/TreeView.hpp>
#include <envire_core/events/GraphEventDispatcher.hpp>
#include <envire_core/events/GraphItemEventDispatcher.hpp>
#include <mars_interfaces/graphics/GraphicsManagerInterface.h>
#include <mars_interfaces/sim/DynamicObject.hpp>

#include <mars/data_broker/ProducerInterface.h>
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackageMapping.h>

#include <mars_ode_collision/objects/Object.hpp>

#include <iostream>

#include <envire_base_types/Link.hpp>
#include <envire_base_types/Inertial.hpp>
#include <envire_base_types/geometry/Box.hpp>
#include <envire_base_types/geometry/Capsule.hpp>
#include <envire_base_types/geometry/Cylinder.hpp>
#include <envire_base_types/geometry/Mesh.hpp>
#include <envire_base_types/geometry/Sphere.hpp>
#include <envire_base_types/joints/Fixed.hpp>
#include <envire_base_types/joints/Revolute.hpp>
#include <envire_base_types/joints/Continuous.hpp>

#include <mars_interfaces/sim/AbsolutePose.hpp>

namespace mars
{
    namespace envire_mars_graphics
    {

        class TmpMap
        {
        public:
            envire::core::FrameId frame;
            smurf::Visual *visual;
            interfaces::DynamicObjectItem *dynamicObject;
        };

        class EnvireMarsGraphics : public lib_manager::LibInterface,
                                   public interfaces::GraphicsUpdateInterface,
                                   public cfg_manager::CFGClient,
                                   public data_broker::ProducerInterface,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::smurf::Frame>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::smurf::Inertial>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::smurf::Joint>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::smurf::Visual>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::base_types::Link>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::base_types::Inertial>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::base_types::geometry::Box>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::base_types::geometry::Capsule>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::base_types::geometry::Cylinder>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::base_types::geometry::Mesh>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::base_types::geometry::Sphere>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::base_types::joints::Fixed>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::base_types::joints::Revolute>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::base_types::joints::Continuous>>
        {

        public:
            EnvireMarsGraphics(lib_manager::LibManager *theManager); ///< Constructor of the \c class Simulator.
            virtual ~EnvireMarsGraphics();

            // --- LibInterface ---
            int getLibVersion() const override
            {
                return 1;
            }

            const std::string getLibName() const override
            {
                return std::string("envire_mars_graphics");
            }

            CREATE_MODULE_INFO();

            // graphics callback
            virtual void preGraphicsUpdate(void) override;

            virtual void cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) override;

            // ## DataBroker callbacks ##
            virtual void produceData(const data_broker::DataInfo &info,
                                     data_broker::DataPackage *package,
                                     int callbackParam) override;

            // envire callbacks
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Frame>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Inertial>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Joint>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Visual>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::Link>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::Inertial>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::geometry::Box>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::geometry::Capsule>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::geometry::Cylinder>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::geometry::Mesh>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::geometry::Sphere>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::joints::Fixed>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::joints::Revolute>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::base_types::joints::Continuous>>& e) override;

            void createVisual(configmaps::ConfigMap &node, envire::core::FrameId frameId);
            void createJoint(const std::string &jointName, envire::core::FrameId frameId);

        private:
            data_broker::DataBrokerInterface *dataBroker;
            interfaces::GraphicsManagerInterface *graphics;
            cfg_manager::CFGManagerInterface *cfg;
            data_broker::DataPackageMapping dbPackageMapping;
            //std::map<unsigned long, TmpMap> visualFrameMap;
            std::map<unsigned long, interfaces::AbsolutePose*> visualMap, frameMap, anchorMap;
            //std::map<unsigned long, std::pair<envire::core::FrameId, Eigen::Affine3d>> visualAnchorMap;
            cfg_manager::cfgPropertyStruct cfgVisRep;
            bool showGui, showCollisions, showAnchor;
            double vizTime, avgVizTime, frameTime, avgFrameTime, anchorTime, avgAnchorTime;
            int avgTimeCount;
        };

    } // end of namespace envire_ode_physics_viz
} // end of namespace mars
